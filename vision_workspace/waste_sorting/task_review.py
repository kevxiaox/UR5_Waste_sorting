"""Review-only task decisions. No planner, transport, or executable coordinates."""
import math
import numpy as np

from .decision import assess_target


def current_stable_window(result, item):
    temporal = result.get('temporal_observation', {})
    samples = temporal.get('samples', [])
    if temporal.get('status') != 'stable_surface_observation_only' or len(samples) != 20:
        return False
    raw = item.get('detector_item')
    if not isinstance(raw, dict):
        return False
    expected = (raw.get('class_name'), item.get('category'), item.get('target_zone'))
    if not all(isinstance(value, str) for value in expected):
        return False
    if samples[-1].get('frame_id') != result['frame_id']:
        return False
    for index, sample in enumerate(samples):
        identity = sample.get('identity', [])
        if (len(identity) != 4 or tuple(identity[:3]) != expected
                or tuple(identity[3]) != tuple(result['image_resolution'])
                or sample.get('frame_id') != samples[0]['frame_id']+index):
            return False
        if (type(sample.get('sensor_timestamp_ns')) is not int
                or type(sample.get('host_time_s')) not in (int, float)
                or not math.isfinite(sample['host_time_s'])):
            return False
        if index and (sample['sensor_timestamp_ns'] <= samples[index-1]['sensor_timestamp_ns']
                      or not 0 < sample['host_time_s']-samples[index-1]['host_time_s'] <= 1):
            return False
    for key, limit in (('camera_axis_std_mm', 5), ('camera_axis_span_mm', 20)):
        values = temporal.get(key, [])
        if len(values) != 3 or any(type(x) not in (int, float) or not math.isfinite(x) or not 0 <= x <= limit for x in values):
            return False
    points = np.asarray([sample.get('camera_xyz_m') for sample in samples], dtype=float)
    if (points.shape != (20, 3) or not np.isfinite(points).all() or np.any(points[:, 2] <= 0)
            or max(points.std(axis=0)*1000) > 5 or max(np.ptp(points,axis=0)*1000) > 20):
        return False
    return samples[-1]['host_time_s']-samples[0]['host_time_s'] <= 10


def build_task_review(result, workspace):
    rows = []
    physical_blockers = ['camera_to_base_requires_independent_validation',
                         'physical_grasp_tcp_unverified', 'surface_point_is_not_validated_grasp_reference',
                         'transport_path_and_drop_geometry_unvalidated', 'execution_disabled']
    for index, item in enumerate(result.get('items', [])):
        raw = item.get('detector_item')
        raw = raw if isinstance(raw, dict) else {}
        detection = {'class_name': raw.get('class_name'), 'confidence': raw.get('confidence'),
                     'bbox': raw.get('bbox_xyxy'), 'center_pixel': raw.get('center_pixel')}
        # Always use global policy, independently of the preview's 0.45 override.
        decision = assess_target(detection, workspace, item.get('camera_xyz_m'))
        blockers = []
        if raw.get('status') != 'OK':
            blockers.append('detector_item_not_ok')
        if decision['status'] != 'waiting_base_calibration':
            blockers.append(decision['status'])
        if item.get('status') != 'candidate_surface_preview_only':
            blockers.append('surface_preview_not_valid')
        if decision.get('category') is not None:
            expected = {'recyclable': 'Recyclable', 'residual': 'General_Waste'}[decision['category']]
            if raw.get('waste_category') != expected or item.get('category') != decision['category'] or item.get('target_zone') != decision['target_zone']:
                blockers.append('class_route_mismatch')
        # Temporal geometry is independent of classification confidence. A stable
        # but low-confidence observation stays blocked, without claiming motion.
        stable = current_stable_window(result, item)
        if not stable:
            blockers.append('waiting_current_single_object_stable_window')
        rows.append({'item_index': index, 'class_name': raw.get('class_name'),
                     'confidence': raw.get('confidence'), 'global_policy_status': decision['status'],
                     'category': decision.get('category'), 'target_zone': decision.get('target_zone'),
                     'vision_review_passed': not blockers, 'current_window_stable': stable,
                     'vision_blockers': blockers, 'physical_blockers': list(physical_blockers),
                     'grasp_pose': None, 'trajectory': None, 'base_xyz_m': None,
                     'execution_status': 'disabled'})
    passed = sum(row['vision_review_passed'] for row in rows)
    status = ('no_detection' if not rows else 'blocked_by_vision_checks' if not passed else
              'blocked_multiple_candidates' if passed > 1 else 'vision_review_only_physical_validation_pending')
    return {'type': 'sorting_task_review', 'frame_id': result.get('frame_id'), 'status': status,
            'items': rows, 'vision_pass_count': passed, 'global_policy_used': True,
            'preview_override_does_not_change_policy': True, 'selected_target': None,
            'robot_execution_enabled': False, 'sorting_execution_enabled': False,
            'grasp_pose': None, 'trajectory': None, 'execution_status': 'disabled',
            'note': 'Review record only, not a task queue or motion plan. Physical blockers remain '
                    'regardless of input flags. No candidate point is converted to an executable target.'}
