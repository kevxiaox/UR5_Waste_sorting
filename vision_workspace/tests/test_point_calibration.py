"""Synthetic calibration truth; zero hardware. No synthetic data enters real config."""
import copy
import unittest
import tempfile
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from waste_sorting.point_calibration import (
    analyze_observations, distinct_observations, evaluate_candidate, fit_point_constraint,
    freeze_training_manifest, verify_training_manifest,
)


def synthetic(seed=5, n=16):
    rng = np.random.default_rng(seed)
    transform = np.eye(4)
    transform[:3,:3] = cv2.Rodrigues(np.array([2.8,0.2,-0.4]))[0]
    transform[:3,3] = [0.15,-0.1,1.3]
    offset = np.array([0.03,-0.02,0.15])
    poses,points = [],[]
    for _ in range(n):
        pose = np.eye(4)
        pose[:3,:3] = cv2.Rodrigues(rng.normal(0,0.6,3))[0]
        pose[:3,3] = np.array([0.1,0,0.65])+rng.normal(0,0.1,3)
        point_base = pose[:3,:3]@offset+pose[:3,3]
        points.append(transform[:3,:3].T@(point_base-transform[:3,3]))
        poses.append(pose)
    return np.asarray(points),np.asarray(poses),transform,offset


class PointCalibrationTests(unittest.TestCase):
    def test_training_manifest_preserves_split_and_detects_changed_data(self):
        points,poses,_,_ = synthetic(n=12)
        with tempfile.TemporaryDirectory() as directory:
            paths = [Path(directory)/f'{i}.json' for i in range(12)]
            for p in paths:
                p.write_text('{}',encoding='utf-8')
            observations = [{'path':str(p.resolve()),'camera_point':point,'flange_pose':pose,'identity':('same',)}
                            for p,point,pose in zip(paths,points,poses)]
            destination = Path(directory)/'manifest.json'
            with patch('waste_sorting.point_calibration.load_observations',return_value=(observations[:8],[])):
                manifest = freeze_training_manifest(paths[:8],destination)
                with self.assertRaises(FileExistsError):
                    freeze_training_manifest(paths[:8],destination)
            for count in (11,12):
                with patch('waste_sorting.point_calibration.load_observations',return_value=(observations[:count],[])):
                    report = analyze_observations(list(reversed(paths[:count])),manifest)
                self.assertEqual(report['training_files'],[str(p.resolve()) for p in paths[:8]])
                self.assertEqual(len(report['held_out_files']),count-8)
                self.assertFalse(report['robot_execution_enabled'])
            paths[0].write_text('{"changed":true}',encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'missing or changed'):
                verify_training_manifest(manifest,paths)

    def test_recovers_camera_transform_and_unknown_offset(self):
        points,poses,truth,offset = synthetic()
        estimated,estimated_offset,diagnostics = fit_point_constraint(points[:12],poses[:12])
        np.testing.assert_allclose(estimated,truth,atol=1e-9)
        np.testing.assert_allclose(estimated_offset,offset,atol=1e-9)
        self.assertEqual(diagnostics["local_jacobian_rank"],9)
        report = evaluate_candidate(points[:12],poses[:12],points[12:],poses[12:])
        self.assertEqual(report["status"],"candidate_fit_requires_independent_physical_validation")
        self.assertFalse(report["robot_execution_enabled"])
        self.assertFalse(report["camera_to_base_calibrated"])
        self.assertFalse(report["independent_physical_validation_passed"])

    def test_tolerates_small_noise_without_claiming_absolute_accuracy(self):
        points,poses,truth,_ = synthetic(n=28)
        noisy = points+np.random.default_rng(14).normal(0,0.001,points.shape)
        report = evaluate_candidate(noisy[:22],poses[:22],noisy[22:],poses[22:])
        self.assertEqual(report["status"],"candidate_fit_requires_independent_physical_validation")
        np.testing.assert_allclose(np.asarray(report["candidate_T_base_from_camera"])[:3,3],truth[:3,3],atol=0.01)

    def test_held_out_observations_not_used_to_fit(self):
        points,poses,_,_ = synthetic()
        bad_validation = points[12:]+[0.1,0,0]
        report = evaluate_candidate(points[:12],poses[:12],bad_validation,poses[12:])
        self.assertEqual(report["status"],"rejected_residuals")
        self.assertGreater(report["held_out_rms_mm"],90)
        self.assertIsNone(report["candidate_T_base_from_camera"])

    def test_no_rotations_and_one_rotation_axis_rejected(self):
        points,poses,truth,offset = synthetic()
        for single_axis in (False,True):
            modified = poses.copy()
            modified_points = []
            for i,pose in enumerate(modified):
                pose[:3,:3] = cv2.Rodrigues(np.array([0.0,0.0,i*0.05 if single_axis else 0]))[0]
                modified_points.append(truth[:3,:3].T@(pose[:3,:3]@offset+pose[:3,3]-truth[:3,3]))
            report = evaluate_candidate(np.asarray(modified_points[:12]),modified[:12],
                                        np.asarray(modified_points[12:]),modified[12:])
            self.assertEqual(report["status"],"rejected_geometry_or_data")
            self.assertIsNone(report["candidate_T_base_from_camera"])

    def test_not_enough_data_produces_no_transform(self):
        points,poses,_,_ = synthetic()
        for n,m in ((1,0),(7,3),(8,2)):
            report = evaluate_candidate(points[:n],poses[:n],points[:m],poses[:m])
            self.assertEqual(report["status"],"waiting_more_distinct_poses")
            self.assertIsNone(report["candidate_T_base_from_camera"])

    def test_invalid_pose_and_nonfinite_points_rejected(self):
        points,poses,_,_ = synthetic()
        poses[0,0,0] *= 2
        report = evaluate_candidate(points[:12],poses[:12],points[12:],poses[12:])
        self.assertEqual(report["status"],"rejected_geometry_or_data")
        points,poses,_,_ = synthetic()
        points[0,0] = float('nan')
        report = evaluate_candidate(points[:12],poses[:12],points[12:],poses[12:])
        self.assertEqual(report["status"],"rejected_geometry_or_data")

    def test_duplicate_poses_cannot_become_held_out_validation(self):
        points,poses,_,_ = synthetic(n=1)
        observation = {"camera_point":points[0],"flange_pose":poses[0],"path":"synthetic"}
        unique,duplicates = distinct_observations([copy.deepcopy(observation) for _ in range(15)])
        self.assertEqual(len(unique),1)
        self.assertEqual(len(duplicates),14)
        self.assertEqual(analyze_observations([])["status"],"waiting_more_distinct_poses")


if __name__ == "__main__":
    unittest.main()
