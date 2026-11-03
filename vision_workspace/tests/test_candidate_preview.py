"""Candidate coordinates cannot become executable TCP/grasp targets. No hardware."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from pick_depth_check import save_measurement
from waste_sorting.candidate_preview import load_candidate, project_surface
from waste_sorting.localization import DepthPolicy, sample_surface
from waste_sorting.zones import new_workspace


def fixture():
    workspace = new_workspace(40,30)
    workspace['camera_pose_revalidated'] = True
    workspace['zones']['PICK_ZONE']['polygon_px'] = [[2,2],[36,2],[36,26],[2,26]]
    transform = np.array([[0,-1,0,0.4],[1,0,0,-0.3],[0,0,1,0.2],[0,0,0,1]],float)
    return workspace, {'transform':transform,'workspace_sha256':'test-hash'}


class CandidatePreviewTests(unittest.TestCase):
    def test_known_transform_and_units_remain_surface_only(self):
        workspace,candidate = fixture()
        sample = {'status':'camera_measurement_stable','camera_xyz_m':[0.1,0.2,0.7]}
        result = project_surface(sample,[10,10],workspace,candidate)
        np.testing.assert_allclose(result['candidate_base_xyz_m'],[0.2,-0.2,0.9])
        self.assertIsNone(result['base_xyz_m'])
        for flag in ('robot_execution_enabled','camera_to_base_calibrated','tcp_verified',
                     'independent_physical_validation_passed'):
            self.assertFalse(result[flag])

    def test_revalidation_zone_and_depth_gates(self):
        workspace,candidate = fixture()
        valid = {'status':'valid_camera_sample','camera_xyz_m':[0.1,0.2,0.7]}
        workspace['camera_pose_revalidated'] = False
        self.assertIsNone(project_surface(valid,[10,10],workspace,candidate)['candidate_base_xyz_m'])
        workspace['camera_pose_revalidated'] = True
        self.assertIsNone(project_surface(valid,[0,0],workspace,candidate)['candidate_base_xyz_m'])
        for point in (None,[0,0,0],[0,0,-1],[0,0,float('nan')],[0,1]):
            sample = dict(valid,camera_xyz_m=point)
            self.assertIsNone(project_surface(sample,[10,10],workspace,candidate)['candidate_base_xyz_m'])
        self.assertIsNone(project_surface(dict(valid,status='rejected_unstable_depth'),
                                         [10,10],workspace,candidate)['candidate_base_xyz_m'])

    def test_saved_preview_retains_provenance_and_never_sets_base_target(self):
        workspace,candidate = fixture()
        cloud = np.full((30,40,3),[0.1,0.2,0.7],dtype=np.float32)
        sample = sample_surface(cloud,[10,10],workspace)
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)/'preview'
            summary,_ = save_measurement(folder,np.zeros((30,40,3),np.uint8),workspace,'test-hash',
                                         [10,10],[sample]*20,DepthPolicy(),candidate=candidate)
            saved = json.loads((folder/'measurement.json').read_text(encoding='utf-8'))
            self.assertEqual(saved['type'],'candidate_base_surface_preview')
            self.assertEqual(saved['candidate_provenance']['workspace_sha256'],'test-hash')
            self.assertIsNotNone(summary['candidate_preview']['candidate_base_xyz_m'])
            self.assertIsNone(summary['base_xyz_m'])
            self.assertFalse(summary['robot_execution_enabled'])
            self.assertFalse(saved['calibration_pair'])
            with self.assertRaises(ValueError):
                save_measurement(Path(directory)/'bad',np.zeros((30,40,3),np.uint8),workspace,'test-hash',
                                 [10,10],[sample]*20,DepthPolicy(),candidate=candidate,tool_point=True)

    def test_evidence_checks_reject_modified_matrix_or_other_workspace(self):
        _,candidate = fixture()
        report = {'status':'candidate_fit_requires_independent_physical_validation',
                  'training_files':['train'], 'held_out_files':['held-out'],
                  'candidate_T_base_from_camera':candidate['transform'].tolist(),
                  'training_rms_mm':1.0,'held_out_rms_mm':2.0,'held_out_max_mm':3.0,
                  'robot_execution_enabled':False,'camera_to_base_calibrated':False,
                  'independent_physical_validation_passed':False}
        with tempfile.TemporaryDirectory() as directory:
            rp,mp = Path(directory)/'report.json',Path(directory)/'manifest.json'
            mp.write_text('{}',encoding='utf-8')
            with patch('waste_sorting.candidate_preview.analyze_observations',return_value=report), \
                 patch('waste_sorting.candidate_preview.load_observations',return_value=(
                     [{'identity':('serial',(40,30),'test-hash')}],[])):
                rp.write_text(json.dumps(report),encoding='utf-8')
                loaded = load_candidate(rp,mp,'test-hash')
                self.assertEqual(len(loaded['report_sha256']),64)
                with self.assertRaisesRegex(ValueError,'workspace fingerprint'):
                    load_candidate(rp,mp,'different-hash')
                altered = copy.deepcopy(report)
                altered['candidate_T_base_from_camera'][0][3] += 0.01
                rp.write_text(json.dumps(altered),encoding='utf-8')
                with self.assertRaisesRegex(ValueError,'differs from reproduced'):
                    load_candidate(rp,mp,'test-hash')


if __name__ == '__main__':
    unittest.main()
