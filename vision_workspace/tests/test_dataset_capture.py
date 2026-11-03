import json
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from waste_sorting.dataset_capture import DatasetCapture
from waste_sorting.zones import new_workspace
from waste_sorting.vision_evaluation import verified_sample


class DatasetCaptureTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)/'capture'
        self.capture = DatasetCapture(self.folder,new_workspace(20,10),'hash','empty_pick',3)
        self.frame = np.zeros((10,20,3),np.uint8)

    def test_session_requires_fresh_alignment_and_never_infers_ground_truth(self):
        with self.assertRaises(ValueError):
            self.capture.record(self.frame,100,.1)
        self.capture.confirm_alignment()
        sample = self.capture.record(self.frame,100,.1)
        self.assertEqual(sample['status'],'saved')
        self.assertIsNone(sample['expected_classes_in_pick_zone'])
        self.assertEqual(sample['label_review_status'],'unreviewed')
        self.assertFalse(self.capture.data['robot_execution_enabled'])

    def test_duplicate_frame_and_write_failure_are_retained_as_attempts(self):
        self.capture.confirm_alignment()
        self.capture.record(self.frame,100,.1)
        duplicate = self.capture.record(self.frame,100,.2)
        self.assertEqual(duplicate['status'],'failed')
        with patch('waste_sorting.dataset_capture.cv2.imwrite',return_value=False):
            failed = self.capture.record(self.frame,101,.3)
        self.assertEqual(failed['status'],'failed')
        self.capture.finish('completed_with_failed_attempts')
        data = json.loads((self.folder/'session.json').read_text(encoding='utf-8'))
        self.assertEqual(len(data['attempts']),3)
        self.assertEqual(data['saved_images'],1)
        with self.assertRaises(ValueError):
            self.capture.record(self.frame,102,.4)

    def test_abort_preserves_partial_collection_and_original_pixels(self):
        self.capture.confirm_alignment()
        self.capture.record(self.frame,100,.1)
        self.capture.finish('aborted')
        self.assertTrue((self.folder/'rgb_01.png').is_file())
        self.assertEqual(self.capture.data['status'],'aborted')
        self.assertEqual(self.capture.data['saved_images'],1)

    def test_reviewed_capture_can_enter_evaluator_but_scenario_alone_cannot(self):
        workspace = new_workspace(20,10)
        # Nonoverlapping tiny polygons only for this offline fixture.
        for i, zone in enumerate(workspace['zones'].values()):
            zone['polygon_px'] = [[i*5,1],[i*5+3,1],[i*5+3,8],[i*5,8]]
            zone['physical_identity_confirmed'] = True
        capture = DatasetCapture(Path(self.temp.name)/'reviewed',workspace,'hash','empty_pick',1)
        capture.confirm_alignment()
        sample = capture.record(self.frame,100,.1)
        capture.finish('completed')
        session = capture.folder/'session.json'
        entry = {'image_path':sample['image_path'], 'image_sha256':sample['image_sha256'],
                 'source_session_path':str(session),
                 'source_session_sha256':hashlib.sha256(session.read_bytes()).hexdigest(),
                 'label_scope':'class_presence_in_pick_zone_only',
                 'expected_classes_in_pick_zone':[], 'label_source':'scenario_intent'}
        with self.assertRaises(ValueError):
            verified_sample(entry)
        entry['label_source'] = 'visual review of saved image'
        self.assertEqual(verified_sample(entry)[0],Path(sample['image_path']))
