"""Offline fixed-logo consistency against the FROZEN fit. No hardware I/O."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from waste_sorting.candidate_preview import load_candidate, candidate_provenance
from waste_sorting.point_calibration import load_observations

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--measurement', type=Path, required=True)
    args = parser.parse_args()
    source = args.measurement.resolve()
    record = json.loads(source.read_text(encoding='utf-8'))
    report_path = ROOT/'data/calibration_analysis/candidate_report.json'
    manifest_path = ROOT/'data/calibration_analysis/training_manifest.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    candidate = load_candidate(report_path, manifest_path, record['workspace_sha256'])
    observations, rejected = load_observations([source])
    if rejected or len(observations) != 1:
        raise ValueError('Invalid observation: '+str(rejected))
    prior, rejected_prior = load_observations(report['training_files']+report['held_out_files'])
    if rejected_prior or any(o['identity'] != observations[0]['identity'] for o in prior):
        raise ValueError('Feature, camera, workspace or robot identity differs from frozen evidence')
    if str(source) in [str(Path(p).resolve()) for p in report['training_files']+report['held_out_files']]:
        raise ValueError('This tool requires a new observation outside original fit/holdout files')
    observation = observations[0]
    measured = candidate['transform']@np.r_[observation['camera_point'], 1]
    predicted = observation['flange_pose']@np.r_[report['candidate_feature_offset_flange_m'], 1]
    residual = (measured[:3]-predicted[:3])*1000
    norm = float(np.linalg.norm(residual))
    nearest = min(prior, key=lambda old: np.linalg.norm(old['flange_pose'][:3, 3]-observation['flange_pose'][:3, 3]))
    relative = nearest['flange_pose'][:3, :3].T@observation['flange_pose'][:3, :3]
    files = [source, source.parent/'rgb.png', report_path, manifest_path]
    result = {
        'type': 'frozen_logo_new_observation_consistency',
        'new_observation_added_to_fit': False, 'frozen_fit_refitted': False,
        'candidate_transformed_measured_logo_base_mm': (measured[:3]*1000).tolist(),
        'frozen_model_predicted_logo_base_mm': (predicted[:3]*1000).tolist(),
        'measured_minus_predicted_base_mm': residual.tolist(), 'residual_norm_mm': norm,
        'original_held_out_rms_mm': report['held_out_rms_mm'],
        'original_held_out_max_mm': report['held_out_max_mm'],
        'residual_range_comparison_only': ('within_original_held_out_range' if norm <= report['held_out_max_mm']
                                          else 'above_original_held_out_range'),
        'nearest_prior_by_flange_translation': {
            'path': nearest['path'],
            'translation_mm': float(1000*np.linalg.norm(nearest['flange_pose'][:3, 3]-observation['flange_pose'][:3, 3])),
            'rotation_deg': float(np.degrees(np.arccos(np.clip((np.trace(relative)-1)/2, -1, 1)))),
        },
        'candidate_provenance': candidate_provenance(candidate),
        'sources': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in files],
        'camera_to_base_calibrated': False, 'tcp_verified': False,
        'independent_physical_validation_passed': False,
        'robot_execution_enabled': False, 'replacement_tcp_pose': None,
        'note': 'One same-feature consistency check against a reproduced frozen fit; no independent absolute-accuracy certification. Selection, depth bias and original fitted feature offset can contribute.',
    }
    target = source.parent/'frozen_logo_review.json'
    if target.exists():
        raise ValueError('Review already exists; preserve the prior review')
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('Frozen model unchanged. New logo residual mm:', round(norm, 4))
    print('Measured minus predicted Base mm:', residual.tolist())
    print('Saved:', target)


if __name__ == '__main__':
    main()
