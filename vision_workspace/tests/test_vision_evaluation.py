import unittest

from waste_sorting.vision_evaluation import score_predictions, deduplicate_samples
from waste_sorting.zones import new_workspace


class VisionEvaluationTests(unittest.TestCase):
    def setUp(self):
        self.workspace = new_workspace(100, 100)
        self.workspace['zones']['PICK_ZONE']['polygon_px'] = [[10,10],[90,10],[90,90],[10,90]]
        self.item = {'class_name': 'sponge', 'confidence': .65, 'status': 'OK',
                     'bbox_xyxy': [30,30,70,70], 'center_pixel': [50,50]}

    def test_threshold_rejection_is_a_miss_not_wrong_class(self):
        low = score_predictions([self.item], ['sponge'], self.workspace, .45)
        high = score_predictions([self.item], ['sponge'], self.workspace, .8)
        self.assertTrue(low['exact_class_list_match'])
        self.assertEqual(high['missing_expected_classes'], ['sponge'])
        self.assertEqual(high['unexpected_classes'], [])
        self.assertFalse(low['robot_execution_enabled'])

    def test_empty_pick_zone_ignores_real_object_elsewhere(self):
        outside = dict(self.item, bbox_xyxy=[0,0,8,8], center_pixel=[4,4])
        result = score_predictions([outside], [], self.workspace, .45)
        self.assertTrue(result['exact_class_list_match'])
        self.assertEqual(result['accepted_in_pick_zone'], [])

    def test_duplicate_and_wrong_class_outputs_are_not_exact_matches(self):
        duplicate = score_predictions([self.item, self.item], ['sponge'], self.workspace, .45)
        self.assertFalse(duplicate['exact_class_list_match'])
        self.assertEqual(duplicate['extra_same_class_outputs'], 1)
        wrong = score_predictions([dict(self.item, class_name='paper_cup')], ['sponge'], self.workspace, .45)
        self.assertEqual(wrong['unexpected_classes'], ['paper_cup'])
        self.assertEqual(wrong['missing_expected_classes'], ['sponge'])

    def test_invalid_geometry_cannot_score_as_a_hit(self):
        result = score_predictions([dict(self.item, center_pixel=[150,150])], ['sponge'], self.workspace, .45)
        self.assertFalse(result['exact_class_list_match'])

    def test_identical_images_are_not_counted_twice_and_conflicting_labels_rejected(self):
        entries = [{'sample_id': 'a', 'image_sha256': 'same', 'expected_classes_in_pick_zone': ['sponge']},
                   {'sample_id': 'b', 'image_sha256': 'same', 'expected_classes_in_pick_zone': ['sponge']}]
        unique, duplicates = deduplicate_samples(entries)
        self.assertEqual(len(unique), 1)
        self.assertEqual(duplicates, [{'sample_id': 'b', 'duplicate_of': 'a'}])
        entries[1]['expected_classes_in_pick_zone'] = ['paper_cup']
        with self.assertRaises(ValueError):
            deduplicate_samples(entries)
