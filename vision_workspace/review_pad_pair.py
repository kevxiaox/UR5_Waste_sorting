"""Conditional pad midpoint and historical cross-image check. Never sets TCP."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import numpy as np

from waste_sorting.candidate_preview import load_candidate, candidate_provenance
from waste_sorting.point_calibration import load_observations
from waste_sorting.projection_review import sample_geometry, project

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pad-a', type=Path, required=True)
    parser.add_argument('--pad-b', type=Path, required=True,
                        help='Explicit role B source; stored feature label is retained and reported')
    parser.add_argument('--cross-sample', type=Path, required=True)
    parser.add_argument('--cross-annotations', type=Path, required=True)
    args = parser.parse_args()
    paths = [args.pad_a.resolve(), args.pad_b.resolve()]
    if paths[0] == paths[1]:
        raise ValueError('Two different source observations required')
    records = [json.loads(p.read_text(encoding='utf-8')) for p in paths]
    observations, rejected = load_observations(paths)
    if rejected or len(observations) != 2:
        raise ValueError('Source rejected: '+str(rejected))
    if observations[0]['identity'][:3] != observations[1]['identity'][:3] or observations[0]['identity'][4:] != observations[1]['identity'][4:]:
        raise ValueError('Camera/workspace/robot identity mismatch')
    candidate = load_candidate(ROOT/'data/calibration_analysis/candidate_report.json',
                               ROOT/'data/calibration_analysis/training_manifest.json', records[0]['workspace_sha256'])
    first, second = [o['flange_pose'] for o in observations]
    delta_mm = float(1000*np.linalg.norm(first[:3, 3]-second[:3, 3]))
    rotation = first[:3, :3].T@second[:3, :3]
    angle = float(np.degrees(np.arccos(np.clip((np.trace(rotation)-1)/2, -1, 1))))
    if delta_mm > 1 or angle > .1:
        raise ValueError('Poses changed: review capture/feature correspondence before averaging')
    points = [
        (np.linalg.inv(o['flange_pose'])@candidate['transform']@np.r_[o['camera_point'], 1])[:3]
        for o in observations]
    midpoint = np.mean(points, axis=0)
    historical = json.loads(args.cross_sample.read_text(encoding='utf-8'))
    annotations = json.loads(args.cross_annotations.read_text(encoding='utf-8'))
    annotation_source = next(s for s in annotations['sources'] if Path(s['path']).name == 'sample.json')
    if hashlib.sha256(args.cross_sample.read_bytes()).hexdigest() != annotation_source['sha256']:
        raise ValueError('Cross-check annotations refer to another source sample')
    if not np.allclose(historical['candidate_provenance']['candidate_T_base_from_camera'],
                       candidate['transform'], atol=1e-10, rtol=0):
        raise ValueError('Cross-check uses different candidate extrinsics')
    transform, intrinsics = sample_geometry(historical)
    pixel = project(transform, midpoint, intrinsics)
    target = np.array(annotations['arithmetic_2d_jaw_midpoint_pixel'])
    role_metadata = [
        {'role_in_this_comparison': role, 'source_path': str(path),
         'stored_feature_id_unchanged': record['feature_id'], 'selected_pixel': record['pixel']}
        for role, path, record in zip(('A', 'B'), paths, records)]
    inputs = paths+[args.cross_sample.resolve(), args.cross_annotations.resolve()]
    inputs += [p.parent/'rgb.png' for p in paths]
    result = {
        'type': 'conditional_pad_pair_geometry',
        'role_assignment': role_metadata,
        'role_note': 'Second source assigned role B according to operator submission after B capture instruction; raw label/directory retained unchanged. Images visually show different finger tips.',
        'same_opening_assumed_not_sensor_verified': True,
        'between_capture_flange_translation_mm': delta_mm,
        'between_capture_flange_rotation_deg': angle,
        'conditional_selected_surface_flange_mm': [(p*1000).tolist() for p in points],
        'conditional_midpoint_flange_mm': (midpoint*1000).tolist(),
        'selected_3d_point_separation_mm': float(1000*np.linalg.norm(points[1]-points[0])),
        'separation_is_verified_jaw_width': False,
        'candidate_provenance': candidate_provenance(candidate),
        'cross_image': {
            'projected_conditional_midpoint_pixel': pixel.tolist(),
            'annotated_arithmetic_2d_midpoint_pixel': target.tolist(),
            'residual_px': float(np.linalg.norm(pixel-target)),
            'historical_axial_255_34_residual_px': next(p['distance_to_marked_midpoint_px'] for p in annotations['hypothesis_to_marked_midpoint_px'] if '255.34' in p['name']),
            'same_jaw_opening_between_days_confirmed': False,
        },
        'sources': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in inputs],
        'tcp_verified': False, 'camera_to_base_calibrated': False,
        'replacement_tcp_pose': None, 'grasp_pose': None, 'trajectory': None,
        'robot_execution_enabled': False,
        'limitations': [
            'Assumes selected visible surface locations represent intended inner contact middles; no independent surface identity verification.',
            'Depth bias on black/edge pixels and frozen camera-fit residual can bias the midpoint despite small temporal scatter.',
            'Two observations at nearly the same pose do not validate a rigid TCP across orientations or jaw openings.',
            'Historical 2D check uses another image but unverified cross-day opening and an arithmetic 2D midpoint, not measured 3D ground truth.',
            'Numeric midpoint is diagnostic only, not a controller setting or approved grasp reference.',
        ],
    }
    folder = ROOT/'data/gripper_geometry_checks/pair_reviews'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    (folder/'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('Conditional midpoint flange XYZ mm:', result['conditional_midpoint_flange_mm'])
    print('Cross-image residual px:', result['cross_image']['residual_px'])
    print('Diagnostic only; controller TCP unchanged. Saved:', folder/'review.json')


if __name__ == '__main__':
    main()
