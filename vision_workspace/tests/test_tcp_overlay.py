import unittest
from unittest.mock import Mock, patch

import numpy as np

from waste_sorting.tcp_overlay import capture_overlay, project_point, axial_hypothesis


class TcpOverlayTests(unittest.TestCase):
    def states(self):
        return [{'robot_ip': 'offline', 'controller_version': '3.15.5.0',
                 'controller_timestamp_s': 100+i, 'host_sample_monotonic_s': 10+i,
                 'actual_q_rad': [.1]*6, 'actual_qd_rad_s': [0.]*6,
                 'actual_tcp_pose_base_m_rotvec_rad': [0, 0, 1.15, 0, 0, 0],
                 'active_tcp_offset_flange_m_rotvec_rad': [0, 0, .15, 0, 0, 0],
                 'runtime_state': 1, 'robot_status_bits': 1, 'safety_mode': 1,
                 'tool_output_0': False, 'tool_output_1': False} for i in range(3)]

    def capture(self, states, times=(11.2, 11.4), axial_hypothesis_mm=None):
        camera = Mock()
        camera.read.return_value = np.zeros((100, 100, 3), dtype=np.uint8)
        with patch('waste_sorting.tcp_overlay.read_tool_io_state', side_effect=[states[0], states[2]]), \
             patch('waste_sorting.tcp_overlay.read_robot_state', return_value=states[1]), \
             patch('waste_sorting.tcp_overlay.time.monotonic', side_effect=times):
            return capture_overlay(camera, {'transform': np.eye(4)}, [0, 0, .1], (100, 100, 50, 50), 'offline',
                                   axial_hypothesis_mm=axial_hypothesis_mm)

    def test_cad_axial_option_rotates_in_flange_frame_and_preserves_disabled_execution(self):
        states = self.states()
        for state in states:
            # +90 degrees around flange/base Y, including configured TCP rotation.
            state['actual_tcp_pose_base_m_rotvec_rad'] = [.15, 0, 1, 0, np.pi/2, 0]
        _, result = self.capture(states, axial_hypothesis_mm=255.34)
        point = result['points'][2]
        np.testing.assert_allclose(point['candidate_camera_xyz_m'], [.25534, 0, 1], atol=1e-9)
        np.testing.assert_allclose(point['flange_offset_xyz_m'], [0, 0, .25534])
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertFalse(result['automatic_replay_enabled'])
        self.assertFalse(result['axial_hypothesis']['installed_flange_registration_verified'])
        self.assertIsNone(result['axial_hypothesis']['controller_tcp_pose'])
        self.assertIsNone(result['grasp_pose'])
        self.assertIsNone(result['trajectory'])

    def test_invalid_axial_option_rejected_before_any_robot_read(self):
        for value in (0, -1, float('nan'), float('inf'), 1001):
            with self.subTest(value=value), patch('waste_sorting.tcp_overlay.read_tool_io_state') as read:
                with self.assertRaises(ValueError):
                    capture_overlay(Mock(), {}, [], [], axial_hypothesis_mm=value)
                read.assert_not_called()

    def test_cad_axial_option_does_not_bypass_stationary_check(self):
        states = self.states()
        states[2]['actual_qd_rad_s'][0] = .1
        with self.assertRaises(ValueError):
            self.capture(states, axial_hypothesis_mm=255.34)

    def test_projection_uses_rectified_intrinsics_and_rejects_hidden_points(self):
        self.assertEqual(project_point([.1, .2, 1], (100, 100, 50, 50), (100, 100))['pixel'], [60, 70])
        for point in ([0, 0, -1], [float('nan'), 0, 1], [0, 0, 0]):
            self.assertIsNone(project_point(point, (100, 100, 50, 50), (100, 100))['pixel'])
        self.assertEqual(project_point([2, 0, 1], (100, 100, 50, 50), (100, 100))['status'], 'outside_image')

    def test_open_jaws_do_not_turn_195mm_hypothesis_into_verified_tcp(self):
        frame, result = self.capture(self.states())
        self.assertEqual(frame.shape, (100, 100, 3))
        self.assertEqual(len(result['points']), 3)
        self.assertAlmostEqual(result['points'][2]['candidate_camera_xyz_m'][2], 1.195)
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['depth_measured'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertIsNone(result['trajectory'])

    def test_moving_robot_rejects_whole_sample(self):
        states = self.states()
        states[2]['actual_qd_rad_s'][0] = .1
        with self.assertRaises(ValueError):
            self.capture(states)

    def test_camera_frame_outside_bracket_rejected(self):
        with self.assertRaises(ValueError):
            self.capture(self.states(), (11.2, 12.5))
