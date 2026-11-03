"""Audited joint references for offline planning; never constructs a trajectory."""
import hashlib
import json
import math
from pathlib import Path

from .taught_poses import build_pose_record


def read_hashed(path, expected=None):
    path = Path(path)
    raw = path.read_bytes()
    sha = hashlib.sha256(raw).hexdigest()
    if expected is not None and sha != expected:
        raise ValueError('Source hash differs: '+str(path))
    return json.loads(raw), sha


def joint_vector(value):
    if (not isinstance(value, list) or len(value) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in value)):
        raise ValueError('Six finite joint positions in controller order required')
    return list(value)


def build_endpoint_catalog(index_path, policy_path):
    index, index_hash = read_hashed(index_path)
    policy, policy_hash = read_hashed(policy_path)
    if (index.get('type') != 'object_specific_manual_endpoint_index'
            or index.get('robot_execution_enabled') is not False
            or index.get('automatic_replay_enabled') is not False
            or policy.get('robot_execution_enabled') is not False):
        raise ValueError('Expected execution-disabled manual reference index and policy')
    if index.get('policy_sha256') != policy_hash:
        raise ValueError('Index references another policy; explicitly review before rebuilding')
    objects = index.get('objects')
    if not isinstance(objects, list) or not objects:
        raise ValueError('At least one explicit object-specific endpoint set required')
    result, seen = [], set()
    aliases = ('pick_approach', 'pick_grasp', 'pick_lift', 'drop_approach', 'drop_release', 'drop_retreat')
    for item in objects:
        name = item['object_class']
        if name in seen or name not in policy['class_to_category']:
            raise ValueError('Duplicate or unknown object class')
        seen.add(name)
        zone = policy['category_to_zone'][policy['class_to_category'][name]]
        if zone not in ('LEFT_CELL', 'RIGHT_CELL') or item['target_zone'] != zone:
            raise ValueError('Object route mismatch')
        session, _ = read_hashed(item['source_session_path'], item['source_session_sha256'])
        incidents = [event for event in session.get('operator_events', [])
                     if event.get('safety_mode_raw') == 3]
        roles = ['PICK_APPROACH', 'PICK_GRASP', 'PICK_LIFT']
        roles += [zone.replace('_CELL', '_DROP_'+suffix) for suffix in ('APPROACH', 'RELEASE', 'RETREAT')]
        refs = item['endpoint_references']
        if set(refs) != set(roles) or [e['role'] for e in session['endpoints']] != roles:
            raise ValueError('Missing, extra or wrong-order endpoint roles')
        waypoints, requests, previous = {}, [], None
        for alias, role, session_entry in zip(aliases, roles, session['endpoints']):
            ref = refs[role]
            source = Path(ref['source_path']).resolve()
            if (Path(session_entry['source_path']).resolve() != source
                    or session_entry['source_sha256'] != ref['source_sha256']):
                raise ValueError('Session and endpoint index disagree')
            record, sha = read_hashed(source, ref['source_sha256'])
            checked = build_pose_record(record['label'], *record['snapshots'])
            if record['label'] != role:
                raise ValueError('Endpoint label does not match its explicit role')
            for key in ('actual_q_rad', 'configured_tcp_pose_base_m_rotvec_rad',
                        'configured_tcp_offset_flange_m_rotvec_rad'):
                if record[key] != checked[key]:
                    raise ValueError('Pose summary differs from snapshots')
            offset = record['configured_tcp_offset_flange_m_rotvec_rad']
            if (len(offset) != 6 or any(type(v) not in (int, float) or v != 0 for v in offset)
                    or ref['recorded_active_tcp_offset_flange_m_rotvec_rad'] != offset):
                raise ValueError('Only consistently zero-TCP references are supported')
            joints = joint_vector(record['actual_q_rad'])
            if joint_vector(ref['actual_joint_positions_rad_reference_only']) != joints:
                raise ValueError('Indexed joints differ from original snapshot')
            waypoints[alias] = {
                'source_record_id': source.parent.name, 'source_path': str(source),
                'source_sha256': sha, 'joint_positions_rad': joints,
                'recorded_tcp_offset_flange_m_rotvec_rad': offset,
                'status': 'reference_only_not_approved_motion_goal',
            }
            requests.append({
                'stage': alias, 'request_type': 'offline_joint_goal_candidate',
                'reference_start_joint_positions_rad': previous,
                'goal_joint_positions_rad': joints,
                'fresh_actual_start_state_required': True,
                'joint_name_mapping_requires_driver_configuration': True,
                'requires_contact_and_carried_object_model': alias in ('pick_grasp', 'pick_lift', 'drop_approach', 'drop_release'),
                'plan_only': True, 'collision_checked': False, 'execution_authorized': False,
                'incident_review_required': bool(incidents),
                'trajectory': None,
            })
            previous = joints
        result.append({
            'object_class': name, 'target_zone': zone,
            'waypoints': waypoints, 'offline_planning_requests': requests,
            'placement_id': 'manual_fixed_'+name+'_20261026',
            'placement_registration_verified': False,
            'operator_confirmation_of_current_placement_required': True,
            'joint_limits_checked': False, 'robot_execution_enabled': False,
            'incident_review_required': bool(incidents), 'incident_evidence': incidents,
        })
    return {
        'type': 'audited_tcp_zero_joint_reference_catalog', 'schema_version': 1,
        'index_sha256': index_hash, 'policy_sha256': policy_hash,
        'joint_units': 'rad', 'joint_order': 'UR controller actual_q[0:6]; no normalization',
        'objects': result, 'min_confidence': policy['min_confidence'],
        'no_plan_for_unrecorded_classes': True,
        'robot_execution_enabled': False, 'automatic_replay_enabled': False,
        'trajectory': None,
        'limitations': [
            'Joint references are endpoints only, not collision-checked paths or linear Cartesian segments.',
            'A matching visual class does not establish current physical placement at the taught fixture.',
            'Robot calibration, gripper/obstacle geometry, carried-object attachment and driver mapping are required before actual planning/execution.',
            'No interpolation, speed, timing, recovery motion or hardware transport is generated here.',
        ],
    }
