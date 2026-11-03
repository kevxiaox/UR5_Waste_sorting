import unittest
import numpy as np

from waste_sorting.projection_review import project, review_annotations


def fixture():
    # A flange whose +Z maps to camera +X, translated in front of camera.
    t = np.array([[0., 0., 1., 0.], [1., 0., 0., 0.], [0., 1., 0., 1.], [0., 0., 0., 1.]])
    intrinsics = [100., 100., 100., 100.]
    sample = {'image_resolution': [300, 300], 'candidate_provenance': {'candidate_T_base_from_camera': np.eye(4).tolist()},
              'robot_pose_record': {'T_base_from_flange': t.tolist()},
              'rectified_left_intrinsics_fx_fy_cx_cy': intrinsics,
              'points': [{'name': name, 'flange_offset_xyz_m': [0, 0, offset],
                          'pixel': project(t, [0, 0, offset], intrinsics).tolist()}
                         for name, offset in [('LOGO fit', .1), ('AXIAL', .25)]]}
    return sample


class ProjectionReviewTests(unittest.TestCase):
    def test_distinguishes_axial_line_residual_from_along_line_distance(self):
        sample = fixture()
        result = review_annotations(sample, [110, 100], [[120, 108], [130, 112]])
        self.assertAlmostEqual(result['logo_reprojection_error_px'], 0)
        self.assertAlmostEqual(result['jaw_midpoint_to_axis_line_px'], 10)
        self.assertAlmostEqual(result['hypothesis_to_marked_midpoint_px'][0]['distance_to_marked_midpoint_px'], 10)
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertIsNone(result['replacement_tcp_pose'])
        self.assertIsNone(result['metric_error_mm'])

    def test_aligned_2d_reference_still_does_not_verify_tcp(self):
        result = review_annotations(fixture(), [110, 100], [[120, 100], [130, 100]])
        self.assertAlmostEqual(result['jaw_midpoint_to_axis_line_px'], 0)
        self.assertFalse(result['tcp_verified'])
        self.assertIsNone(result['replacement_tcp_pose'])

    def test_modified_saved_projection_and_bad_clicks_rejected(self):
        sample = fixture()
        sample['points'][0]['pixel'][0] += 1
        with self.assertRaises(ValueError):
            review_annotations(sample, [110, 100], [[120, 100], [130, 100]])
        for bad in ([float('nan'), 10], [-1, 100], [300, 100]):
            with self.assertRaises(ValueError):
                review_annotations(fixture(), bad, [[120, 100], [130, 100]])

    def test_axis_looking_into_camera_reports_degenerate_without_fake_distance(self):
        sample = fixture()
        t = np.eye(4); t[2, 3] = 1
        sample['robot_pose_record']['T_base_from_flange'] = t.tolist()
        for p in sample['points']:
            p['pixel'] = [100, 100]
        result = review_annotations(sample, [100, 100], [[90, 100], [110, 100]])
        self.assertEqual(result['axis_observability'], 'degenerate')
        self.assertIsNone(result['jaw_midpoint_to_axis_line_px'])
