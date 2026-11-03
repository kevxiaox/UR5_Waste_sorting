import unittest
import numpy as np

from review_tcp_geometry import compare_points


class TcpGeometryTests(unittest.TestCase):
    def test_axial_delta_rotates_with_flange_not_world_z(self):
        flange = np.array([[0., 0., 1., .3], [0., 1., 0., -.1],
                           [-1., 0., 0., .7], [0., 0., 0., 1.]])
        result = compare_points(flange, np.eye(4), [0, 0, .1], [0, 0, .15], .195)
        np.testing.assert_allclose(result['configured_to_hypothesis_base_delta_mm'], [45, 0, 0], atol=1e-10)
        self.assertAlmostEqual(result['configured_to_hypothesis_distance_mm'], 45)
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertIsNone(result['grasp_pose'])
        self.assertIsNone(result['trajectory'])

    def test_camera_translation_and_logo_are_separate_from_grip(self):
        camera = np.eye(4)
        camera[:3, 3] = [1, 2, 3]
        result = compare_points(np.eye(4), camera, [.01, .02, .104], [0, 0, .15], .195)
        np.testing.assert_allclose(result['points']['logo']['candidate_camera_xyz_m'], [-.99, -1.98, -2.896])
        self.assertNotEqual(result['points']['logo'], result['points']['sponge_grip_hypothesis'])

    def test_nonrigid_transform_rejected(self):
        invalid = np.eye(4)
        invalid[0, 0] = 2
        with self.assertRaises(ValueError):
            compare_points(invalid, np.eye(4), [0, 0, .1], [0, 0, .15], .195)
