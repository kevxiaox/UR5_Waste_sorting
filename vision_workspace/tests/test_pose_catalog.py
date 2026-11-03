import json
from pathlib import Path
import tempfile
import unittest

from waste_sorting.pose_catalog import build_catalog, digest
from waste_sorting.taught_poses import build_pose_record


class PoseCatalogTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.folder = Path(self.temp.name)
        states = [{
            'robot_ip': 'offline', 'controller_version': '3.15.5.0',
            'controller_timestamp_s': 100+i, 'host_sample_monotonic_s': 10+i,
            'actual_q_rad': [0.1]*6, 'actual_qd_rad_s': [0.0]*6,
            'actual_tcp_pose_base_m_rotvec_rad': [0.1, 0.2, 0.7, 0, 0, 0],
            'active_tcp_offset_flange_m_rotvec_rad': [0, 0, 0.15, 0, 0, 0],
            'runtime_state': 1, 'robot_status_bits': 1, 'safety_mode': 1,
        } for i in range(3)]
        self.record = build_pose_record('PICK_APPROACH', *states)
        self.first = self.write('first/pose.json', self.record)
        self.write('second/pose.json', self.record)

    def write(self, name, data):
        path = self.folder / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(data), encoding='utf-8')
        return path

    def review(self):
        self.write('review.json', {'type': 'operator_posture_review', 'entries': [{
            'source_path': str(self.first), 'source_sha256': digest(self.first),
            'review_status': 'rejected_table_contact_do_not_reuse',
        }]})

    def test_duplicate_labels_keep_rejected_record_without_selecting_target(self):
        self.review()
        catalog = build_catalog(self.folder)
        self.assertEqual(catalog['duplicate_labels']['PICK_APPROACH'], ['first', 'second'])
        self.assertEqual(catalog['records'][0]['review_status'], 'rejected_table_contact_do_not_reuse')
        self.assertFalse(catalog['target_selection_performed'])
        self.assertFalse(catalog['automatic_replay_enabled'])

    def test_modified_source_fails_annotation_hash_and_preserves_rejection(self):
        self.review()
        self.record['note'] = 'changed'
        self.write('first/pose.json', self.record)
        row = build_catalog(self.folder)['records'][0]
        self.assertIn('annotation_source_mismatch', row['evidence_errors'])
        self.assertEqual(row['review_status'], 'rejected_table_contact_do_not_reuse')

    def test_summary_cannot_replace_snapshot_joint_angles(self):
        self.record['actual_q_deg'][0] += 360
        self.write('first/pose.json', self.record)
        row = build_catalog(self.folder)['records'][0]
        self.assertIn('snapshot_summary_mismatch:actual_q_deg', row['evidence_errors'])
        self.assertEqual(row['review_status'], 'evidence_error')

    def test_role_cannot_reference_another_pose_with_identical_hash(self):
        self.write('first/operator_role.json', {
            'source_path': str(self.folder/'second/pose.json'),
            'source_sha256': digest(self.first), 'intended_role': 'candidate_release',
        })
        row = build_catalog(self.folder)['records'][0]
        self.assertIn('annotation_source_mismatch', row['evidence_errors'])

    def test_new_release_label_does_not_relabel_old_records(self):
        old = dict(self.record)
        old.pop('label_scope')
        old.pop('label_purpose')
        self.write('first/pose.json', old)
        new = dict(self.record, label='RIGHT_DROP_RELEASE')
        self.write('second/pose.json', new)
        rows = build_catalog(self.folder)['records']
        self.assertIsNone(rows[0]['label_purpose'])
        self.assertIn('候选释放', rows[1]['label_purpose'])
        self.assertFalse(rows[1]['automatic_replay_enabled'])
