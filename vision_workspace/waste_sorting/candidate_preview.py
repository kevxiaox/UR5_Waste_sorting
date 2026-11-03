"""Offline candidate surface coordinates only; no robot transport or grasp pose."""
import hashlib
import json
from pathlib import Path

import numpy as np

from .point_calibration import analyze_observations, load_observations
from .zones import zone_at_pixel


def load_candidate(report_path, manifest_path, workspace_hash):
    """Reproduce the frozen fit from raw evidence before displaying coordinates."""
    report_bytes = Path(report_path).read_bytes()
    manifest_bytes = Path(manifest_path).read_bytes()
    report = json.loads(report_bytes)
    manifest = json.loads(manifest_bytes)
    paths = report['training_files']+report['held_out_files']
    if len(set(paths)) != len(paths):
        raise ValueError('Duplicate candidate evidence paths')
    checked = analyze_observations(paths,manifest)
    expected = 'candidate_fit_requires_independent_physical_validation'
    if checked['status'] != expected or report['status'] != expected:
        raise ValueError('Candidate has not passed held-out diagnostic gates')
    observations,rejected = load_observations(paths)
    if rejected or not observations or any(o['identity'][2] != workspace_hash for o in observations):
        raise ValueError('Candidate evidence does not match current workspace fingerprint')
    for flag in ('robot_execution_enabled','camera_to_base_calibrated','independent_physical_validation_passed'):
        if report.get(flag) is not False:
            raise ValueError('Only unvalidated, execution-disabled candidate reports are supported')
    transform = np.asarray(report['candidate_T_base_from_camera'],dtype=float)
    if (transform.shape != (4,4) or not np.isfinite(transform).all() or
            not np.allclose(transform,checked['candidate_T_base_from_camera'],rtol=0,atol=1e-10)):
        raise ValueError('Candidate transform differs from reproduced frozen fit')
    if (report['training_files'] != checked['training_files'] or
            report['held_out_files'] != checked['held_out_files']):
        raise ValueError('Candidate training/held-out split differs from reproduced fit')
    return {'transform':transform, 'workspace_sha256':workspace_hash,
            'report_path':str(Path(report_path).resolve()),
            'report_sha256':hashlib.sha256(report_bytes).hexdigest(),
            'manifest_sha256':hashlib.sha256(manifest_bytes).hexdigest(),
            'training_rms_mm':checked['training_rms_mm'],
            'held_out_rms_mm':checked['held_out_rms_mm'],
            'held_out_max_mm':checked['held_out_max_mm']}


def project_surface(sample, pixel, workspace, candidate):
    result = {'status':'waiting_valid_camera_sample', 'candidate_base_xyz_m':None,
              'candidate_coordinate_frame':'UR_BASE', 'base_xyz_m':None,
              'robot_execution_enabled':False, 'camera_to_base_calibrated':False,
              'independent_physical_validation_passed':False, 'tcp_verified':False,
              'note':'Unvalidated candidate surface point; not a TCP pose or executable grasp target.'}
    if workspace.get('camera_pose_revalidated') is not True:
        return dict(result,status='waiting_camera_revalidation')
    if zone_at_pixel(workspace,pixel) != ['PICK_ZONE']:
        return dict(result,status='rejected_outside_pick_zone')
    if sample.get('status') not in ('valid_camera_sample','camera_measurement_stable'):
        return result
    point = np.asarray(sample.get('camera_xyz_m'),dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0:
        return result
    transform = candidate['transform']
    base = transform[:3,:3]@point+transform[:3,3]
    return dict(result,status='candidate_surface_preview_only',candidate_base_xyz_m=base.tolist())


def candidate_provenance(candidate):
    return {k:v for k,v in candidate.items() if k != 'transform'} | {
        'candidate_T_base_from_camera':candidate['transform'].tolist()}
