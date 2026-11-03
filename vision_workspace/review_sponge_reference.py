"""Offline comparison of a sponge surface and a manually aligned reference.

This is a consistency diagnostic, never an executable grasp target or calibration.
"""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np

from waste_sorting.candidate_preview import load_candidate
from waste_sorting.observations import flange_from_state
from waste_sorting.taught_poses import build_pose_record

ROOT = Path(__file__).resolve().parent


def compare_reference(surface, base_from_flange, offset, thickness_mm,
                      same_placement, flat_top, middle_contact):
    if not all(value is True for value in (same_placement, flat_top, middle_contact)):
        raise ValueError('Requires operator confirmation of unchanged placement, flat top, and middle-height contact')
    surface, offset = np.asarray(surface, dtype=float), np.asarray(offset, dtype=float)
    if (surface.shape != (3,) or offset.shape != (3,)
            or not np.isfinite(surface).all() or not np.isfinite(offset).all()
            or not np.isfinite(thickness_mm) or not 0 < thickness_mm <= 100):
        raise ValueError('Invalid point, offset, or thickness')
    if (base_from_flange.shape != (4, 4) or not np.isfinite(base_from_flange).all()
            or not np.allclose(base_from_flange[3], [0, 0, 0, 1])
            or not np.allclose(base_from_flange[:3, :3].T @ base_from_flange[:3, :3], np.eye(3))
            or not np.isclose(np.linalg.det(base_from_flange[:3, :3]), 1)):
        raise ValueError('Invalid flange transform')
    surface_to_middle = np.array([0., 0., -thickness_mm/2000])
    visual_reference = surface + surface_to_middle
    manual_reference = (base_from_flange @ np.r_[offset, 1])[:3]
    delta = manual_reference - visual_reference
    return {'status': 'consistency_comparison_only_no_pass_fail_accuracy_claim',
            'candidate_visual_surface_base_xyz_m': surface.tolist(),
            'assumed_surface_to_middle_base_delta_m': surface_to_middle.tolist(),
            'modelled_visual_middle_base_xyz_m': visual_reference.tolist(),
            'modelled_manual_grip_reference_base_xyz_m': manual_reference.tolist(),
            'manual_minus_visual_middle_mm': (delta*1000).tolist(),
            'discrepancy_norm_mm': float(np.linalg.norm(delta)*1000),
            'assumed_sponge_thickness_mm': thickness_mm,
            'operator_confirmations': {'same_placement': True, 'flat_top': True, 'middle_height_contact': True},
            'uncertainty_mm': None, 'independent_physical_validation_passed': False,
            'tcp_verified': False, 'robot_execution_enabled': False,
            'grasp_pose': None, 'trajectory': None,
            'note': 'Model reference uses half the coarse sponge thickness along Base -Z, assuming a horizontal top. '
                    'Surface point may differ in XY from actual opposed contact centers. '
                    '195mm grip offset is a hypothesis, not an independently calibrated tool. '
                    'Discrepancy combines extrinsic, depth, detection, TCP, thickness and manual-alignment errors.'}


def review(vision, pose, candidate_offset, thickness_mm, confirmations):
    result = vision['result']
    if (result.get('mode') != 'live_candidate_surface_preview'
            or not vision['workspace_snapshot'].get('camera_pose_revalidated')):
        raise ValueError('Requires a saved session-confirmed live visual sample')
    if len(result['items']) != 1:
        raise ValueError('Requires exactly one sponge')
    item = result['items'][0]
    if (item['detector_item']['class_name'] != 'sponge'
            or item['status'] != 'candidate_surface_preview_only'):
        raise ValueError('Requires a valid sponge surface preview')
    temporal = result.get('temporal_observation', {})
    samples = temporal.get('samples', [])
    if temporal.get('status') != 'stable_surface_observation_only' or len(samples) != 20:
        raise ValueError('Requires a saved 20-sample stable surface window')
    points = np.asarray([s['camera_xyz_m'] for s in samples], dtype=float)
    if points.shape != (20, 3) or not np.isfinite(points).all():
        raise ValueError('Invalid temporal samples')
    if max(points.std(axis=0)*1000) > 5 or max(np.ptp(points, axis=0)*1000) > 20:
        raise ValueError('Saved window exceeds diagnostic limits')
    checked = build_pose_record(pose['label'], *pose['snapshots'])
    if pose['label'] not in ('PICK_GRASP', 'TCP_CHECK_GRASP'):
        raise ValueError('Requires a manually aligned grip reference posture')
    for key in ('actual_q_rad', 'actual_q_deg', 'configured_tcp_pose_base_m_rotvec_rad',
                'configured_tcp_offset_flange_m_rotvec_rad'):
        if pose[key] != checked[key]:
            raise ValueError('Pose summary differs from snapshots')
    if candidate_offset.get('type') != 'sponge_grip_tcp_translation_hypothesis':
        raise ValueError('Requires the explicit sponge-only TCP hypothesis')
    output = compare_reference(item['candidate_base_xyz_m'], flange_from_state(pose['snapshots'][1]),
                               candidate_offset['candidate_translation_flange_xyz_m'], thickness_mm,
                               *confirmations)
    output['visual_sample_frame_id'] = result['frame_id']
    output['visual_pixel'] = item['detector_item']['center_pixel']
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision', type=Path, required=True)
    parser.add_argument('--pose', type=Path, required=True)
    parser.add_argument('--tcp-hypothesis', type=Path,
                        default=ROOT/'data/tcp_checks/sponge_translation_candidate_20261023_130812_000524.json')
    parser.add_argument('--thickness-mm', type=float, required=True)
    parser.add_argument('--same-placement-confirmed', action='store_true')
    parser.add_argument('--flat-top-confirmed', action='store_true')
    parser.add_argument('--middle-contact-confirmed', action='store_true')
    args = parser.parse_args()
    def read(path):
        return json.loads(path.read_text(encoding='utf-8'))
    vision, pose, tcp = read(args.vision), read(args.pose), read(args.tcp_hypothesis)
    workspace_path = ROOT/'config/workspace_zones.json'
    candidate = load_candidate(ROOT/'data/calibration_analysis/candidate_report.json',
                               ROOT/'data/calibration_analysis/training_manifest.json',
                               hashlib.sha256(workspace_path.read_bytes()).hexdigest())
    if (vision['candidate_provenance']['report_sha256'] != candidate['report_sha256']
            or vision['workspace_sha256'] != candidate['workspace_sha256']):
        raise ValueError('Visual sample uses a different candidate report')
    point = candidate['transform'] @ np.r_[vision['result']['items'][0]['camera_xyz_m'], 1]
    if not np.allclose(point[:3], vision['result']['items'][0]['candidate_base_xyz_m'], rtol=0, atol=1e-10):
        raise ValueError('Visual Base point differs from reproduced candidate transform')
    for source in tcp['sources']:
        if hashlib.sha256(Path(source['path']).read_bytes()).hexdigest() != source['sha256']:
            raise ValueError('TCP hypothesis source changed')
    output = review(vision, pose, tcp, args.thickness_mm,
                    (args.same_placement_confirmed, args.flat_top_confirmed, args.middle_contact_confirmed))
    output['sources'] = [{'path': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                         for p in (args.vision, args.pose, args.tcp_hypothesis)]
    folder = ROOT/'data/sponge_reference_checks'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    path = folder/'comparison.json'
    path.write_text(json.dumps(output, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('OFFLINE consistency diagnostic only; no robot connection or executable target.')
    print('Manual minus visual middle mm:', output['manual_minus_visual_middle_mm'])
    print('Saved:', path)


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError) as error:
        raise SystemExit(str(error))
