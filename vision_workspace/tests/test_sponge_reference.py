import unittest
import numpy as np

from review_sponge_reference import compare_reference


class SpongeReferenceTests(unittest.TestCase):
    def test_top_to_middle_is_base_vertical_not_flange_axis(self):
        flange = np.array([[0., 0., 1., .305], [0., 1., 0., .1],
                           [-1., 0., 0., .5], [0., 0., 0., 1.]])
        result = compare_reference([.5, .1, .51], flange, [0, 0, .195], 20, True, True, True)
        np.testing.assert_allclose(result['manual_minus_visual_middle_mm'], [0, 0, 0], atol=1e-10)
        self.assertFalse(result['independent_physical_validation_passed'])
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertIsNone(result['grasp_pose'])

    def test_cannot_compare_unconfirmed_placements_or_contact_assumptions(self):
        for confirmations in ((False, True, True), (True, False, True), (True, True, False)):
            with self.assertRaises(ValueError):
                compare_reference([0, 0, .5], np.eye(4), [0, 0, .195], 20, *confirmations)

    def test_discrepancy_remains_reported_without_automatic_pass_threshold(self):
        result = compare_reference([0, 0, .215], np.eye(4), [0, 0, .195], 20, True, True, True)
        self.assertAlmostEqual(result['manual_minus_visual_middle_mm'][2], -10)
        self.assertAlmostEqual(result['discrepancy_norm_mm'], 10)
        self.assertIn('no_pass_fail_accuracy_claim', result['status'])
