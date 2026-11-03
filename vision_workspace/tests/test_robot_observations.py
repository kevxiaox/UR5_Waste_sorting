"""Synthetic RTDE packets and poses only. No hardware, sockets or robot motion."""
import copy
import json
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from pick_depth_check import save_measurement
from waste_sorting.localization import DepthPolicy, sample_tool_point
from waste_sorting.observations import compare_robot_bracket, flange_from_state, pose_matrix
from waste_sorting.rtde_readonly import OUTPUT_NAMES, OUTPUT_TYPES, encode_packet, read_robot_state
from waste_sorting.zones import new_workspace


def server_packet(kind, data=b""):
    return struct.pack("!HB",len(data)+3,kind)+data


class FakeConnection:
    def __init__(self, data):
        self.data = bytearray(data)
        self.writes = []
        self.closed = False

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True

    def settimeout(self, timeout):
        if timeout <= 0:
            raise TimeoutError("expired")

    def recv(self, size):
        # Deliberately fragment every header and every double.
        count = min(size,2,len(self.data))
        chunk = bytes(self.data[:count])
        del self.data[:count]
        return chunk

    def sendall(self, data):
        self.writes.append(data)


def responses(values=None, recipe_types=OUTPUT_TYPES):
    if values is None:
        values = [100.0]+[0.0]*6+[0.1,0.2,0.7,0,0,0]+[0,0,0.15,0,0,0]
    return b"".join([
        server_packet(86,b"\x01"), server_packet(118,struct.pack("!4I",3,15,5,0)),
        server_packet(79,b"\x07"+recipe_types), server_packet(83,b"\x01"),
        server_packet(85,b"\x07"+struct.pack("!19d",*values)),
    ])


def robot_state(host_time=10.0, controller_time=100.0):
    return {"robot_ip":"test-only", "controller_version":"3.15.5.0",
            "controller_timestamp_s":controller_time, "host_sample_monotonic_s":host_time,
            "actual_q_rad":[0.0]*6,
            "actual_tcp_pose_base_m_rotvec_rad":[0.1,0.2,0.7,0,0,0],
            "active_tcp_offset_flange_m_rotvec_rad":[0,0,0.15,0,0,0]}


class ReadOnlyProtocolTests(unittest.TestCase):
    def test_fragmented_replies_and_output_only_requests(self):
        connection = FakeConnection(responses())
        with patch("waste_sorting.rtde_readonly.socket.create_connection", return_value=connection):
            state = read_robot_state("test-only")
        self.assertTrue(connection.closed)
        self.assertEqual([data[2] for data in connection.writes],[86,118,79,83])
        self.assertEqual(connection.writes[2][3:],struct.pack("!d",10.0)+OUTPUT_NAMES)
        self.assertEqual(state["controller_version"],"3.15.5.0")
        self.assertEqual(state["actual_q_rad"],[0.0]*6)
        self.assertEqual(state["active_tcp_offset_flange_m_rotvec_rad"][2],0.15)
        self.assertFalse(state["robot_execution_enabled"])

    def test_control_messages_refused(self):
        for kind in (73,85,80,0,255):
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                encode_packet(kind)

    def test_invalid_recipe_closes_connection_without_start(self):
        connection = FakeConnection(responses(recipe_types=b"NOT_FOUND"))
        with patch("waste_sorting.rtde_readonly.socket.create_connection", return_value=connection):
            with self.assertRaises(ValueError):
                read_robot_state("test-only")
        self.assertTrue(connection.closed)
        self.assertEqual([data[2] for data in connection.writes],[86,118,79])

    def test_nonfinite_state_rejected(self):
        values = [float("nan")]+[0.0]*18
        connection = FakeConnection(responses(values=values))
        with patch("waste_sorting.rtde_readonly.socket.create_connection", return_value=connection):
            with self.assertRaisesRegex(ValueError,"Non-finite"):
                read_robot_state("test-only")
        self.assertTrue(connection.closed)

    def test_disconnect_and_protocol_rejection(self):
        for data, error in ((b"",ConnectionError),(server_packet(86,b"\x00"),ValueError)):
            connection = FakeConnection(data)
            with patch("waste_sorting.rtde_readonly.socket.create_connection", return_value=connection):
                with self.assertRaises(error):
                    read_robot_state("test-only")
            self.assertTrue(connection.closed)


class ObservationTests(unittest.TestCase):
    def test_flange_transform_removes_offset_in_rotated_frame(self):
        state = robot_state()
        state["actual_tcp_pose_base_m_rotvec_rad"] = [0.1,0.2,0.7,0,np.pi/2,0]
        flange = flange_from_state(state)
        np.testing.assert_allclose(flange[:3,3],[-0.05,0.2,0.7],atol=1e-12)
        np.testing.assert_allclose(flange @ pose_matrix(state["active_tcp_offset_flange_m_rotvec_rad"]),
                                   pose_matrix(state["actual_tcp_pose_base_m_rotvec_rad"]),atol=1e-12)

    def test_stationary_endpoints_only(self):
        result = compare_robot_bracket(robot_state(),robot_state(12,102),10.2,11.7)
        self.assertEqual(result["status"],"robot_bracket_consistent")
        self.assertFalse(result["motion_between_endpoints_observed"])

    def test_position_rotation_and_joint_changes(self):
        for field,index,delta in (("actual_tcp_pose_base_m_rotvec_rad",0,0.002),
                                  ("actual_tcp_pose_base_m_rotvec_rad",5,0.01),
                                  ("actual_q_rad",0,0.01)):
            after = robot_state(12,102)
            after[field][index] += delta
            self.assertEqual(compare_robot_bracket(robot_state(),after,10.2,11.7)["status"],
                             "rejected_robot_pose_changed")

    def test_identity_time_and_tcp_change(self):
        for patch_values,expected in (
            ({"robot_ip":"other"},"rejected_robot_identity_or_time"),
            ({"controller_timestamp_s":99},"rejected_robot_identity_or_time"),
            ({"host_sample_monotonic_s":25},"rejected_bracket_timing"),
            ({"active_tcp_offset_flange_m_rotvec_rad":[0,0,0.16,0,0,0]},"rejected_tcp_offset_changed")):
            after = dict(robot_state(12,102),**patch_values)
            self.assertEqual(compare_robot_bracket(robot_state(),after,10.2,11.7)["status"],expected)
        self.assertEqual(compare_robot_bracket(robot_state(),robot_state(12,102),9.9,11.7)["status"],
                         "rejected_bracket_timing")

    def test_equivalent_rotation_vectors_compare_as_matrices(self):
        before, after = robot_state(), robot_state(12,102)
        before["actual_tcp_pose_base_m_rotvec_rad"][3:] = [0,0,3.0]
        after["actual_tcp_pose_base_m_rotvec_rad"][3:] = [0,0,3.0-2*np.pi]
        self.assertEqual(compare_robot_bracket(before,after,10.2,11.7)["status"],"robot_bracket_consistent")

    def test_saved_candidate_never_contains_known_feature_base_coordinates(self):
        config = new_workspace(40,30)
        config["camera_pose_revalidated"] = True
        xyz = np.zeros((30,40,3),np.float32)
        xyz[...,2] = 1
        samples = [sample_tool_point(xyz,[10,10],config)]*20
        bracket = compare_robot_bracket(robot_state(),robot_state(12,102),10.2,11.7)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)/"candidate"
            summary,_ = save_measurement(folder,np.zeros((30,40,3),np.uint8),config,"mock-only",[10,10],
                                       samples,DepthPolicy(),tool_point=True,robot_bracket=bracket,
                                       feature_id="synthetic_fixed_point")
            payload = json.loads((folder/"measurement.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["observation_status"],"candidate_observation_recorded")
            self.assertFalse(payload["calibration_pair"])
            self.assertFalse(payload["rigid_point_identity_confirmed"])
            self.assertFalse(summary["robot_execution_enabled"])
            self.assertIsNone(summary["base_xyz_m"])
            self.assertEqual(payload["robot_bracket"],bracket)


if __name__ == "__main__":
    unittest.main()
