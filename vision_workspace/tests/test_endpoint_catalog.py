import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from tests.test_taught_poses import snapshots
from waste_sorting.taught_poses import build_pose_record
from waste_sorting.endpoint_catalog import build_endpoint_catalog


def write(path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding='utf-8')
    return hashlib.sha256(path.read_bytes()).hexdigest()


class EndpointCatalogTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        self.policy_path = self.root/'policy.json'
        self.policy = {'robot_execution_enabled': False, 'min_confidence': .6,
                       'class_to_category': {'sponge': 'residual'}, 'category_to_zone': {'residual': 'RIGHT_CELL'}}
        policy_sha = write(self.policy_path, self.policy)
        roles = ['PICK_APPROACH', 'PICK_GRASP', 'PICK_LIFT', 'RIGHT_DROP_APPROACH', 'RIGHT_DROP_RELEASE', 'RIGHT_DROP_RETREAT']
        refs, entries = {}, []
        for n, role in enumerate(roles):
            states = snapshots()
            for state in states:
                state['actual_q_rad'][-1] = 3.2
                state['active_tcp_offset_flange_m_rotvec_rad'] = [0.]*6
            record = build_pose_record(role, *states)
            path = self.root/(role+'.json')
            sha = write(path, record)
            refs[role] = {'source_path': str(path), 'source_sha256': sha,
                          'actual_joint_positions_rad_reference_only': record['actual_q_rad'],
                          'recorded_active_tcp_offset_flange_m_rotvec_rad': [0.]*6}
            entries.append({'role': role, 'source_path': str(path), 'source_sha256': sha})
        self.session = self.root/'session.json'
        session_sha = write(self.session, {'endpoints': entries})
        self.index = {'type': 'object_specific_manual_endpoint_index', 'robot_execution_enabled': False,
                      'automatic_replay_enabled': False, 'policy_sha256': policy_sha,
                      'objects': [{'object_class': 'sponge', 'target_zone': 'RIGHT_CELL',
                                   'source_session_path': str(self.session), 'source_session_sha256': session_sha,
                                   'endpoint_references': refs}]}
        self.index_path = self.root/'index.json'

    def build(self, index=None):
        write(self.index_path, self.index if index is None else index)
        return build_endpoint_catalog(self.index_path, self.policy_path)

    def test_joint_revolutions_preserved_and_requests_are_not_trajectories(self):
        result = self.build()
        item = result['objects'][0]
        self.assertEqual(item['waypoints']['pick_grasp']['joint_positions_rad'][-1], 3.2)
        self.assertIsNone(item['offline_planning_requests'][0]['reference_start_joint_positions_rad'])
        self.assertFalse(result['robot_execution_enabled'])
        self.assertIsNone(result['trajectory'])
        self.assertFalse(item['placement_registration_verified'])
        for request in item['offline_planning_requests']:
            self.assertFalse(request['execution_authorized'])
            self.assertFalse(request['collision_checked'])
            self.assertIsNone(request['trajectory'])

    def test_tampered_pose_rejected_even_with_same_index(self):
        path = Path(self.index['objects'][0]['endpoint_references']['PICK_GRASP']['source_path'])
        record = json.loads(path.read_text(encoding='utf-8')); record['actual_q_rad'][0] += .1
        write(path, record)
        with self.assertRaises(ValueError):
            self.build()

    def test_wrong_route_missing_role_duplicate_class_and_execution_flags_rejected(self):
        for kind in ('route', 'role', 'duplicate', 'execution'):
            index = copy.deepcopy(self.index)
            if kind == 'route': index['objects'][0]['target_zone'] = 'LEFT_CELL'
            elif kind == 'role': index['objects'][0]['endpoint_references'].pop('PICK_LIFT')
            elif kind == 'duplicate': index['objects'].append(copy.deepcopy(index['objects'][0]))
            else: index['robot_execution_enabled'] = True
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                self.build(index)

    def test_indexed_nonzero_tcp_and_policy_change_cannot_be_silently_adopted(self):
        index = copy.deepcopy(self.index)
        index['objects'][0]['endpoint_references']['PICK_GRASP']['recorded_active_tcp_offset_flange_m_rotvec_rad'][2] = .188
        with self.assertRaises(ValueError): self.build(index)
        self.policy['min_confidence'] = .8
        write(self.policy_path, self.policy)
        with self.assertRaises(ValueError): self.build()

    def test_prior_protective_stop_is_retained_in_plan_only_candidates(self):
        session = json.loads(self.session.read_text(encoding='utf-8'))
        session['operator_events'] = [{'stage': 'retreat_capture_blocked', 'safety_mode_raw': 3}]
        self.index['objects'][0]['source_session_sha256'] = write(self.session, session)
        result = self.build()
        obj = result['objects'][0]
        self.assertTrue(obj['incident_review_required'])
        self.assertEqual(obj['incident_evidence'][0]['safety_mode_raw'], 3)
        self.assertTrue(all(r['incident_review_required'] for r in obj['offline_planning_requests']))
        self.assertFalse(result['robot_execution_enabled'])
