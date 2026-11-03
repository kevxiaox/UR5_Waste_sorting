"""No hardware: depth rejection gates and synchronized SDK retrieval contract."""
from types import SimpleNamespace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from waste_sorting.camera import ZedSource
from waste_sorting.localization import DepthPolicy, sample_surface, sample_tool_point, summarize_samples
from waste_sorting.zones import new_workspace
from pick_depth_check import save_measurement


def fixture():
    workspace = new_workspace(40,30)
    workspace["camera_pose_revalidated"] = True
    workspace["zones"]["PICK_ZONE"]["polygon_px"] = [[2,2],[36,2],[36,26],[2,26]]
    xyz = np.empty((30,40,3), np.float32)
    xyz[:] = [0.1,0.2,1.0]
    return workspace, xyz


class DepthTests(unittest.TestCase):
    def test_tool_point_outside_pick_does_not_relax_sorting(self):
        c, xyz = fixture()
        c["zones"]["PICK_ZONE"]["polygon_px"] = [[12,2],[36,2],[36,26],[12,26]]
        self.assertEqual(sample_surface(xyz,[6,10],c)["status"], "rejected_outside_pick_zone")
        candidate = sample_tool_point(xyz,[6,10],c)
        self.assertEqual(candidate["status"], "valid_camera_sample")
        self.assertFalse(candidate["calibration_pair"])
        self.assertEqual(candidate["measurement_scope"], "tool_point_diagnostic")

    def test_tool_point_preserves_revalidation_and_depth_gates(self):
        c, xyz = fixture()
        self.assertEqual(sample_tool_point(xyz,[0,10],c)["status"], "rejected_patch_at_image_edge")
        self.assertEqual(sample_tool_point(xyz[:20],[10,10],c)["status"], "rejected_xyz_shape")
        xyz[10,10] = [0,0,np.nan]
        self.assertEqual(sample_tool_point(xyz,[10,10],c)["status"], "rejected_invalid_center_depth")
        c["camera_pose_revalidated"] = False
        self.assertEqual(sample_tool_point(xyz,[10,10],c)["status"], "waiting_camera_revalidation")

    def test_tool_point_record_is_not_a_calibration_pair(self):
        c, xyz = fixture()
        sample = sample_tool_point(xyz,[10,10],c)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp) / "tool_sample"
            summary, rejections = save_measurement(
                folder, np.zeros((30,40,3),np.uint8), c, "test-only-hash", [10,10],
                [sample]*20, DepthPolicy(), tool_point=True)
            payload = json.loads((folder/"measurement.json").read_text(encoding="utf-8"))
            self.assertEqual(payload["type"], "unpaired_tool_point_diagnostic")
            self.assertFalse(payload["calibration_pair"])
            self.assertFalse(payload["rigid_point_identity_confirmed"])
            self.assertFalse(summary["robot_execution_enabled"])
            self.assertIsNone(summary["base_xyz_m"])
            self.assertEqual(rejections, {})

    def test_keeps_actual_center_and_valid_fraction(self):
        c, xyz = fixture()
        xyz[10,10] = [0.11,0.22,1.005]
        result = sample_surface(xyz, [10,10], c)
        self.assertEqual(result["status"], "valid_camera_sample")
        np.testing.assert_allclose(result["camera_xyz_m"], xyz[10,10])
        self.assertEqual(result["patch_valid_fraction"], 1.0)

    def test_invalid_center_not_replaced_by_nearby_table(self):
        c, xyz = fixture()
        for center in ([0,0,np.nan], [0,0,-1], [np.inf,0,1]):
            xyz[10,10] = center
            result = sample_surface(xyz, [10,10], c)
            self.assertEqual(result["status"], "rejected_invalid_center_depth")
            self.assertIsNone(result["camera_xyz_m"])

    def test_sparse_patch_and_mixed_surface(self):
        c, xyz = fixture()
        xyz[8:11,8:13] = np.nan
        xyz[10,10] = [0,0,1]
        self.assertEqual(sample_surface(xyz,[10,10],c)["status"], "rejected_sparse_depth")
        c, xyz = fixture()
        xyz[8:10,8:13,2] = 1.1
        self.assertEqual(sample_surface(xyz,[10,10],c)["status"], "rejected_depth_discontinuity")

    def test_verification_shape_pixel_and_edge_gates(self):
        c, xyz = fixture()
        for pixel, status in (([0,0],"rejected_outside_pick_zone"),
                              ([3,10],"rejected_patch_at_zone_edge"),
                              ([100,10],"rejected_pixel")):
            self.assertEqual(sample_surface(xyz,pixel,c)["status"], status)
        self.assertEqual(sample_surface(xyz[:20],[10,10],c)["status"], "rejected_xyz_shape")
        c["camera_pose_revalidated"] = False
        self.assertEqual(sample_surface(xyz,[10,10],c)["status"], "waiting_camera_revalidation")

    def test_twenty_frames_and_invalid_fraction(self):
        c, xyz = fixture()
        good = sample_surface(xyz,[10,10],c)
        bad = dict(good, status="rejected_invalid_center_depth", camera_xyz_m=None)
        self.assertEqual(summarize_samples([good]*19)["status"], "rejected_insufficient_frames")
        self.assertEqual(summarize_samples([good]*11+[bad]*9)["status"], "rejected_insufficient_frames")
        result = summarize_samples([good]*12+[bad]*8)
        self.assertEqual(result["status"], "camera_measurement_stable")
        self.assertEqual(result["valid_frames"], 12)
        self.assertFalse(result["robot_execution_enabled"])
        self.assertIsNone(result["base_xyz_m"])

    def test_temporal_variation_rejected(self):
        c, xyz = fixture()
        a = sample_surface(xyz,[10,10],c)
        b = dict(a, camera_xyz_m=[0.1,0.2,1.1])
        self.assertEqual(summarize_samples([a,b]*10)["status"], "rejected_unstable_depth")


class FakeMat:
    def __init__(self):
        self.array = None

    def get_data(self):
        return self.array


class FakeCamera:
    """Values encode grab index; first XYZ retrieval fails on purpose."""
    def __init__(self):
        self.grabs = 0
        self.measures = 0

    def grab(self, runtime):
        self.grabs += 1
        return 0

    def retrieve_image(self, mat, view):
        mat.array = np.full((2,3,4), self.grabs, np.uint8)
        return 0

    def retrieve_measure(self, mat, measure):
        self.measures += 1
        mat.array = np.full((2,3,4), self.grabs, np.float32)
        return 1 if self.measures == 1 else 0


class CameraContractTests(unittest.TestCase):
    def source(self):
        source = ZedSource()
        source.camera = FakeCamera()
        source.sl = SimpleNamespace(ERROR_CODE=SimpleNamespace(SUCCESS=0),
                                    VIEW=SimpleNamespace(LEFT=0), MEASURE=SimpleNamespace(XYZ=0))
        source.runtime = object()
        source.image, source.cloud = FakeMat(), FakeMat()
        return source

    def test_rgb_read_does_not_request_xyz(self):
        source = self.source()
        self.assertEqual(source.read().shape, (2,3,3))
        self.assertEqual(source.camera.measures, 0)

    def test_failed_xyz_is_discarded_and_next_pair_is_synchronized(self):
        source = self.source()
        bgr, xyz = source.read_with_xyz()
        self.assertEqual(source.camera.grabs, 2)
        self.assertTrue((bgr == 2).all())
        self.assertTrue((xyz == 2).all())
        self.assertEqual(xyz.shape, (2,3,3))
        source.cloud.array[:] = 99
        self.assertTrue((xyz == 2).all())

    def test_disconnection_retries_are_bounded(self):
        source = self.source()
        source.camera.grab = lambda runtime: 1
        with self.assertRaisesRegex(RuntimeError, "60 times"):
            source.read_with_xyz()


if __name__ == "__main__":
    unittest.main()
