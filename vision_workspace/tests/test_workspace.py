"""Synthetic polygons only. No camera, robot, SDK or measured robot points."""
import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from waste_sorting.decision import assess_target, classify_detection
from waste_sorting.interfaces import DryRunRobot, record_preview
from waste_sorting.zones import (
    display_to_original, load_workspace, new_workspace, overlay,
    polygon_array, save_workspace, validate_workspace, zone_at_pixel,
)


def mock_workspace():
    c = new_workspace(400, 200, {"kind": "synthetic_test"})
    # Physical LEFT deliberately has larger image X than physical RIGHT.
    for name, x in zip(c["zones"], (300, 200, 100, 0)):
        c["zones"][name]["polygon_px"] = [[x, 10], [x+70, 10], [x+70, 90], [x, 90]]
        c["zones"][name]["physical_identity_confirmed"] = True
    c["camera_pose_revalidated"] = True
    return c


def detection(cls="plastic_bottle", confidence=0.91):
    return {"class_name": cls, "confidence": confidence,
            "bbox": [10, 20, 30, 40], "center_pixel": [20, 30]}


class WorkspaceTests(unittest.TestCase):
    def test_roundtrip_requires_fresh_camera_validation(self):
        c = mock_workspace()
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "zones.json"
            save_workspace(path, c)
            loaded = load_workspace(path, [400, 200])
            self.assertEqual(loaded["zones"], c["zones"])
            self.assertFalse(loaded["camera_pose_revalidated"])
            self.assertTrue(c["camera_pose_revalidated"])
            with self.assertRaises(ValueError):
                load_workspace(path, [800, 400])

    def test_pixel_scaling_and_bounds(self):
        for scale in (0.5, 0.85, 1.0, 1.5):
            size = [round(1280*scale), round(720*scale)]
            self.assertEqual(display_to_original(size[0]//2, size[1]//2, [1280,720], size), [640,360])
        self.assertIsNone(display_to_original(-1, 10, [1280,720], [640,360]))
        self.assertEqual(display_to_original(639, 359, [1280,720], [640,360]), [1278,718])

    def test_physical_identity_not_image_x(self):
        c = mock_workspace()
        self.assertEqual(zone_at_pixel(c, [330, 30]), ["LEFT_CELL"])
        self.assertEqual(zone_at_pixel(c, [130, 30]), ["RIGHT_CELL"])
        self.assertEqual(zone_at_pixel(c, [390, 150]), [])

    def test_bad_polygons(self):
        for pts in ([[0,0],[70,90],[70,0],[0,90]], [[0,0],[1,1],[2,2]],
                    [[0,0],[20,0],[0,0]], [[-1,0],[20,0],[0,20]],
                    [[0,0],[float("nan"),0],[0,20]], [[0,0],[500,0],[0,20]]):
            with self.subTest(pts=pts), self.assertRaises(ValueError):
                polygon_array(pts, [400,200])

    def test_overlap_and_obsolete_zone_rejected(self):
        c = mock_workspace()
        c["zones"]["RIGHT_CELL"]["polygon_px"] = c["zones"]["LEFT_CELL"]["polygon_px"]
        with self.assertRaises(ValueError):
            validate_workspace(c)
        c = mock_workspace()
        c["zones"]["OLD_RECTANGLE"] = copy.deepcopy(c["zones"]["LEFT_CELL"])
        with self.assertRaises(ValueError):
            validate_workspace(c)

    def test_shared_boundary_is_ambiguous(self):
        c = mock_workspace()
        c["zones"]["MIDDLE_CELL"]["polygon_px"] = [[230,10],[300,10],[300,90],[230,90]]
        validate_workspace(c)
        self.assertEqual(zone_at_pixel(c, [300,30]), ["LEFT_CELL", "MIDDLE_CELL"])

    def test_overlay_keeps_original_image(self):
        frame = np.zeros((200,400,3), np.uint8)
        rendered = overlay(frame, mock_workspace())
        self.assertEqual(rendered.shape, frame.shape)
        self.assertTrue(rendered.any())
        self.assertFalse(frame.any())

    def test_missing_or_unconfirmed_zones(self):
        with self.assertRaises(ValueError):
            validate_workspace(new_workspace(400,200), require_complete=True)
        c = mock_workspace()
        c["zones"]["LEFT_CELL"]["physical_identity_confirmed"] = False
        self.assertEqual(assess_target(detection(), c)["status"], "waiting_workspace_validation")

    def test_malformed_flags(self):
        c = mock_workspace()
        c["camera_pose_revalidated"] = "true"
        with self.assertRaises(ValueError):
            validate_workspace(c)


class DecisionTests(unittest.TestCase):
    def test_all_class_routes(self):
        for cls, target in (("plastic_bottle","LEFT_CELL"), ("aluminum_can","LEFT_CELL"),
                            ("sponge","RIGHT_CELL"), ("toothbrush","RIGHT_CELL")):
            result = classify_detection(detection(cls))
            self.assertEqual(result["target_zone"], target)
            self.assertEqual(result["status"], "ready_for_localization")

    def test_unknown_and_low_confidence(self):
        self.assertEqual(classify_detection(detection("unknown"))["status"], "rejected_unknown_class")
        self.assertEqual(classify_detection(detection(confidence=0.59))["status"], "rejected_low_confidence")
        self.assertEqual(classify_detection(detection(confidence=0.60))["status"], "ready_for_localization")

    def test_invalid_detector_inputs(self):
        for value in (None, [], {}, {"class_name": []}, detection(confidence=float("nan")),
                      detection(confidence=True), detection(confidence=1.2)):
            self.assertTrue(classify_detection(value)["status"].startswith("rejected"))
        for patch in ({"center_pixel":[100,100]}, {"bbox":[30,20,10,40]}, {"bbox":[0,0,1,float("inf")]}):
            self.assertEqual(classify_detection(dict(detection(), **patch))["status"], "rejected_detection")

    def test_workspace_camera_depth_calibration_gates(self):
        c = mock_workspace()
        c["camera_pose_revalidated"] = False
        self.assertEqual(assess_target(detection(),c)["status"], "waiting_camera_revalidation")
        c["camera_pose_revalidated"] = True
        for xyz in (None, [0,0,0], [0,0,-1], [0,0,float("nan")], [0,0,True]):
            self.assertEqual(assess_target(detection(),c,xyz)["status"], "waiting_valid_depth")
        result = assess_target(detection(), c, [0.1,0.2,1.0])
        self.assertEqual(result["status"], "waiting_base_calibration")
        self.assertIsNone(result["base_xyz_m"])
        self.assertFalse(result["robot_execution_enabled"])

    def test_pick_zone_and_bounds(self):
        c = mock_workspace()
        d = dict(detection(), center_pixel=[130,30], bbox=[120,20,140,40])
        self.assertEqual(assess_target(d,c)["status"], "rejected_outside_pick_zone")
        d = dict(detection(), bbox=[10,20,500,40])
        self.assertEqual(assess_target(d,c)["status"], "rejected_detection_outside_image")

    def test_execution_and_gripper_always_refused(self):
        robot = DryRunRobot()
        with self.assertRaises(RuntimeError):
            robot.execute({"enabled":True})
        with self.assertRaises(RuntimeError):
            robot.command("close")
        preview = robot.preview(detection(), mock_workspace(), [0,0,1])
        self.assertIsNone(preview["trajectory"])
        self.assertEqual(preview["execution_status"], "disabled")
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"preview.jsonl"
            record_preview(path, preview)
            self.assertEqual(json.loads(path.read_text()), preview)


if __name__ == "__main__":
    unittest.main()
