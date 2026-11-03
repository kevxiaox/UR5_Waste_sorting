"""Offline EXPERIMENTAL point/pose calibration. Never creates a runtime calibration.

For a fixed tool-body point with UNKNOWN flange offset o:
    R_base_camera @ p_camera_i + t_base_camera = R_base_flange_i @ o + t_base_flange_i
This is our implementation of that geometric constraint, not a reproduction of
the algorithm or accuracy claims in https://arxiv.org/abs/2004.12611.
"""
from pathlib import Path
import json
import hashlib

import cv2
import numpy as np

from .localization import summarize_samples
from .observations import compare_robot_bracket, flange_from_state

MIN_TRAIN = 8
MIN_HOLDOUT = 3


def _skew(p):
    x,y,z = p
    return np.array([[0,-z,y],[z,0,-x],[-y,x,0]],dtype=float)


def _arrays(points, poses):
    points, poses = np.asarray(points,dtype=float), np.asarray(poses,dtype=float)
    n = len(points)
    if points.shape != (n,3) or poses.shape != (n,4,4) or n == 0:
        raise ValueError("Expected Nx3 camera points and Nx4x4 flange poses")
    if not np.isfinite(points).all() or not np.isfinite(poses).all():
        raise ValueError("Non-finite point/pose")
    for pose in poses:
        if (not np.allclose(pose[3],[0,0,0,1],atol=1e-8) or
                not np.allclose(pose[:3,:3].T@pose[:3,:3],np.eye(3),atol=1e-6) or
                not np.isclose(np.linalg.det(pose[:3,:3]),1,atol=1e-6)):
            raise ValueError("Flange poses must be rigid transforms with proper rotations")
    return points, poses


def point_errors(points, poses, transform, offset):
    return (points@transform[:3,:3].T+transform[:3,3] -
            np.einsum('nij,j->ni',poses[:,:3,:3],offset)-poses[:,:3,3])


def _huber_cost(errors, delta=0.005):
    lengths = np.linalg.norm(errors,axis=1)
    return float(np.sum(np.where(lengths<=delta,0.5*lengths**2,delta*(lengths-0.5*delta))))


def fit_point_constraint(points, poses):
    """Conservative initializer + rotation-constrained robust refinement, NumPy only.

    Rejects linearly rank-deficient or poorly conditioned acquisition geometry,
    even if a more specialized algorithm might recover some such configurations.
    """
    points, poses = _arrays(points,poses)
    if len(points) < MIN_TRAIN:
        raise ValueError(f"Need at least {MIN_TRAIN} distinct training poses")
    center = points.mean(axis=0)
    centered = points-center
    linear = np.vstack([np.hstack((np.kron(np.eye(3),p[None,:]),np.eye(3),-a[:3,:3]))
                        for p,a in zip(centered,poses)])
    target = poses[:,:3,3].reshape(-1)
    solution,_,rank,singular = np.linalg.lstsq(linear,target,rcond=None)
    condition = float(singular[0]/singular[-1]) if singular[-1] > 0 else float('inf')
    if rank < 15 or condition > 1e5:
        raise ValueError("Acquisition geometry is degenerate/ill-conditioned; diversify poses and rotation axes")
    raw_rotation = solution[:9].reshape(3,3)
    u,_,vt = np.linalg.svd(raw_rotation)
    rotation = u@np.diag([1,1,np.linalg.det(u@vt)])@vt
    translation_offset = np.vstack([np.hstack((np.eye(3),-a[:3,:3])) for a in poses])
    target = (poses[:,:3,3]-centered@rotation.T).reshape(-1)
    initial,_,_,_ = np.linalg.lstsq(translation_offset,target,rcond=None)
    translation, offset = initial[:3], initial[3:]
    damping = 1e-5
    iterations = 0
    for iterations in range(1,101):
        rotated = centered@rotation.T
        errors = rotated+translation-np.einsum('nij,j->ni',poses[:,:3,:3],offset)-poses[:,:3,3]
        jacobian = np.vstack([np.hstack((-_skew(p),np.eye(3),-a[:3,:3]))
                              for p,a in zip(rotated,poses)])
        lengths = np.linalg.norm(errors,axis=1)
        weights = np.repeat(np.sqrt(np.minimum(1,0.005/np.maximum(lengths,1e-15))),3)
        step,_,_,_ = np.linalg.lstsq(
            np.vstack((jacobian*weights[:,None],np.sqrt(damping)*np.eye(9))),
            np.concatenate((-errors.reshape(-1)*weights,np.zeros(9))),rcond=None)
        if np.linalg.norm(step) < 1e-11:
            break
        old_cost = _huber_cost(errors)
        accepted = False
        for fraction in (1.0,0.5,0.25,0.125,0.0625):
            trial_rotation = cv2.Rodrigues(fraction*step[:3])[0]@rotation
            trial_translation = translation+fraction*step[3:6]
            trial_offset = offset+fraction*step[6:9]
            trial_errors = (centered@trial_rotation.T+trial_translation-
                            np.einsum('nij,j->ni',poses[:,:3,:3],trial_offset)-poses[:,:3,3])
            if _huber_cost(trial_errors) < old_cost:
                rotation,translation,offset = trial_rotation,trial_translation,trial_offset
                damping = max(1e-12,damping/2)
                accepted = True
                break
        if not accepted:
            damping *= 10
            if damping > 1e10:
                raise ValueError("Rotation-constrained refinement failed to converge")
    rotated = centered@rotation.T
    jacobian = np.vstack([np.hstack((-_skew(p),np.eye(3),-a[:3,:3]))
                          for p,a in zip(rotated,poses)])
    singular_j = np.linalg.svd(jacobian,compute_uv=False)
    rank_j = int(np.sum(singular_j > singular_j[0]*1e-5))
    if rank_j != 9:
        raise ValueError("Nine calibration parameters are not sufficiently observable")
    transform = np.eye(4)
    transform[:3,:3] = rotation
    transform[:3,3] = translation-rotation@center
    return transform,offset,{
        "linear_rank":int(rank), "linear_condition":condition,
        "local_jacobian_rank":rank_j,
        "local_jacobian_condition":float(singular_j[0]/singular_j[-1]),
        "refinement_iterations":iterations,
    }


def evaluate_candidate(train_points, train_poses, validation_points, validation_poses):
    result = {"status":"waiting_more_distinct_poses", "robot_execution_enabled":False,
              "camera_to_base_calibrated":False, "independent_physical_validation_passed":False,
              "candidate_T_base_from_camera":None, "candidate_feature_offset_flange_m":None,
              "minimum_training_poses":MIN_TRAIN, "minimum_held_out_poses":MIN_HOLDOUT,
              "provisional_quality_thresholds_mm":{"training_rms":5.0,"held_out_rms":8.0,"held_out_max":15.0},
              "note":"Experimental fit only. Held-out residuals do not establish absolute accuracy or safe robot motion."}
    if len(train_points) < MIN_TRAIN or len(validation_points) < MIN_HOLDOUT:
        return result
    try:
        train_points,train_poses = _arrays(train_points,train_poses)
        validation_points,validation_poses = _arrays(validation_points,validation_poses)
        transform,offset,diagnostics = fit_point_constraint(train_points,train_poses)
        train_mm = np.linalg.norm(point_errors(train_points,train_poses,transform,offset),axis=1)*1000
        held_out_mm = np.linalg.norm(point_errors(validation_points,validation_poses,transform,offset),axis=1)*1000
        training_rms = float(np.sqrt(np.mean(train_mm**2)))
        held_out_rms = float(np.sqrt(np.mean(held_out_mm**2)))
        result.update(diagnostics)
        result.update({"training_rms_mm":training_rms,"held_out_rms_mm":held_out_rms,
                       "held_out_max_mm":float(held_out_mm.max()),
                       "training_errors_mm":train_mm.tolist(),"held_out_errors_mm":held_out_mm.tolist()})
        if training_rms > 5 or held_out_rms > 8 or held_out_mm.max() > 15:
            return dict(result,status="rejected_residuals")
        return dict(result,status="candidate_fit_requires_independent_physical_validation",
                    candidate_T_base_from_camera=transform.tolist(),
                    candidate_feature_offset_flange_m=offset.tolist())
    except (ValueError,np.linalg.LinAlgError) as error:
        return dict(result,status="rejected_geometry_or_data",error=str(error))


def load_observations(paths):
    """Recheck raw bracket/samples rather than trusting the saved pass flags."""
    observations, rejected = [], []
    for path in paths:
        path = Path(path)
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
            if record.get("type") != "candidate_tool_robot_observation":
                raise ValueError("Not a candidate observation")
            workspace = record["workspace_snapshot"]
            source = workspace["source"]
            if (workspace["camera_pose_revalidated"] is not True or source.get("kind") != "zed_live" or
                    source.get("view") != "LEFT" or not source.get("serial_number")):
                raise ValueError("Camera source/revalidation missing")
            if not record.get("feature_id") or not record.get("workspace_sha256"):
                raise ValueError("Feature identity/config fingerprint missing")
            bracket = record["robot_bracket"]
            recomputed = compare_robot_bracket(bracket["before"],bracket["after"],
                bracket["camera_start_host_monotonic_s"],bracket["camera_end_host_monotonic_s"])
            if recomputed["status"] != "robot_bracket_consistent":
                raise ValueError(recomputed["status"])
            summary = summarize_samples(record["samples"])
            if summary["status"] != "camera_measurement_stable" or summary["camera_xyz_m"][2] <= 0:
                raise ValueError("Camera sampling not stable/valid")
            identity = (source["serial_number"],tuple(workspace["image_size"]),record["workspace_sha256"],
                        record["feature_id"],bracket["before"]["robot_ip"],bracket["before"]["controller_version"])
            observations.append({"path":str(path),"camera_point":summary["camera_xyz_m"],
                                 "flange_pose":flange_from_state(bracket["before"]),"identity":identity})
        except (OSError,ValueError,TypeError,KeyError,IndexError) as error:
            rejected.append({"path":str(path),"reason":str(error)})
    return observations,rejected


def distinct_observations(observations):
    """Group essentially repeated poses so held-out tests cannot reuse them."""
    distinct,duplicates = [], []
    for observation in observations:
        pose = observation["flange_pose"]
        is_duplicate = False
        for previous in distinct:
            old = previous["flange_pose"]
            translation = np.linalg.norm(pose[:3,3]-old[:3,3])
            angle = np.arccos(np.clip((np.trace(pose[:3,:3].T@old[:3,:3])-1)/2,-1,1))
            if translation < 0.005 and angle < np.radians(2):
                is_duplicate = True
                break
        (duplicates if is_duplicate else distinct).append(observation)
    return distinct,duplicates


def freeze_training_manifest(paths, destination):
    """Freeze accepted distinct observations before collecting held-out data."""
    paths = [Path(p).resolve() for p in paths]
    observations, rejected = load_observations(paths)
    unique, duplicates = distinct_observations(observations)
    if rejected or duplicates or len(unique) < MIN_TRAIN or len({o['identity'] for o in unique}) != 1:
        raise ValueError("Freeze requires at least eight consistent, valid, distinct observations")
    manifest = {"schema_version":1, "type":"frozen_candidate_training_set",
                "files":[{"path":str(p),"sha256":hashlib.sha256(p.read_bytes()).hexdigest()} for p in paths],
                "robot_execution_enabled":False}
    destination = Path(destination)
    destination.parent.mkdir(parents=True,exist_ok=True)
    with destination.open('x',encoding='utf-8') as stream:
        json.dump(manifest,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    return manifest


def verify_training_manifest(manifest, paths):
    if manifest.get('type') != 'frozen_candidate_training_set' or manifest.get('schema_version') != 1:
        raise ValueError('Invalid training manifest')
    entries = manifest['files']
    frozen = [str(Path(e['path']).resolve()) for e in entries]
    available = {str(Path(p).resolve()) for p in paths}
    if len(frozen) < MIN_TRAIN or len(set(frozen)) != len(frozen):
        raise ValueError('Invalid training file count or duplicate training paths')
    for entry, path in zip(entries,frozen):
        if path not in available or hashlib.sha256(Path(path).read_bytes()).hexdigest() != entry['sha256']:
            raise ValueError('Frozen training file missing or changed: '+path)
    return frozen


def analyze_observations(paths, training_manifest=None):
    paths = [Path(p).resolve() for p in paths]
    frozen = verify_training_manifest(training_manifest,paths) if training_manifest is not None else None
    observations,rejected = load_observations(paths)
    if len({o["identity"] for o in observations}) > 1:
        return {"status":"rejected_mixed_camera_workspace_feature_or_robot", "robot_execution_enabled":False,
                "camera_to_base_calibrated":False, "candidate_T_base_from_camera":None,
                "rejected_records":rejected}
    if frozen is not None:
        by_path = {o['path']:o for o in observations}
        if any(p not in by_path for p in frozen):
            raise ValueError('Frozen training record failed raw-data checks')
        observations = [by_path[p] for p in frozen]+[o for o in observations if o['path'] not in frozen]
    unique,duplicates = distinct_observations(observations)
    split = min(len(unique),len(frozen) if frozen is not None else MIN_TRAIN)
    if frozen is not None and {o['path'] for o in unique[:split]} != set(frozen):
        raise ValueError('Frozen training records are not distinct')
    train,held_out = unique[:split],unique[split:]
    result = evaluate_candidate([o["camera_point"] for o in train],[o["flange_pose"] for o in train],
                                [o["camera_point"] for o in held_out],[o["flange_pose"] for o in held_out])
    result.update({"records_seen":len(paths),"accepted_observations":len(observations),
                   "distinct_robot_poses":len(unique),"repeated_pose_records":len(duplicates),
                   "training_files":[o["path"] for o in train],"held_out_files":[o["path"] for o in held_out],
                   "rejected_records":rejected,
                   "split_note":"Frozen manifest training set; all new distinct poses held out." if frozen is not None else
                   "First eight distinct poses train; all subsequent distinct poses held out. Freeze a manifest before further acquisition."})
    return result
