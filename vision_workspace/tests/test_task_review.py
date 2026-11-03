import copy
import json
import unittest

from waste_sorting.task_review import build_task_review
from waste_sorting.zones import new_workspace


def fixture():
    workspace = new_workspace(100,100)
    for i, zone in enumerate(workspace['zones'].values()):
        x = i*25
        zone['polygon_px'] = [[x+1,1],[x+23,1],[x+23,40],[x+1,40]]
        zone['physical_identity_confirmed'] = True
    workspace['camera_pose_revalidated'] = True
    raw = {'class_name':'sponge','confidence':.95,'status':'OK','waste_category':'General_Waste',
           'bbox_xyxy':[78,10,92,30],'center_pixel':[85,20]}
    item = {'detector_item':raw,'category':'residual','target_zone':'RIGHT_CELL',
            'camera_xyz_m':[.1,.2,1.0],'status':'candidate_surface_preview_only'}
    samples = [{'frame_id':i,'host_time_s':i*.1,'sensor_timestamp_ns':i*100000000,
                'camera_xyz_m':[.1,.2,1.0],
                'identity':('sponge','residual','RIGHT_CELL',(100,100))} for i in range(1,21)]
    result = {'frame_id':20,'image_resolution':[100,100],'items':[item],
              'temporal_observation':{'status':'stable_surface_observation_only','samples':samples,
                                      'camera_axis_std_mm':[0,0,0],'camera_axis_span_mm':[0,0,0]}}
    return workspace,result


class TaskReviewTests(unittest.TestCase):
    def test_live_and_saved_formats_pass_visual_checks_but_never_enable_execution(self):
        workspace,result = fixture()
        for data in (result,json.loads(json.dumps(result))):
            data.update(tcp_verified=True,camera_to_base_calibrated=True,robot_execution_enabled=True)
            review=build_task_review(data,workspace)
            self.assertEqual(review['vision_pass_count'],1)
            self.assertEqual(review['status'],'vision_review_only_physical_validation_pending')
            self.assertIn('physical_grasp_tcp_unverified',review['items'][0]['physical_blockers'])
            self.assertFalse(review['robot_execution_enabled'])
            self.assertIsNone(review['selected_target'])
            self.assertIsNone(review['trajectory'])

    def test_preview_override_cannot_bypass_global_threshold(self):
        workspace,result=fixture()
        result['preview_min_confidence']=.45
        result['items'][0]['detector_item']['confidence']=.55
        review=build_task_review(result,workspace)
        self.assertEqual(review['vision_pass_count'],0)
        self.assertIn('rejected_low_confidence',review['items'][0]['vision_blockers'])
        self.assertTrue(review['items'][0]['current_window_stable'])
        self.assertNotIn('waiting_current_single_object_stable_window',review['items'][0]['vision_blockers'])
        self.assertFalse(review['robot_execution_enabled'])

    def test_low_confidence_with_stale_window_reports_both_problems(self):
        workspace,result=fixture()
        result['items'][0]['detector_item']['confidence']=.55
        result['frame_id']=21
        review=build_task_review(result,workspace)
        self.assertFalse(review['items'][0]['current_window_stable'])
        self.assertIn('rejected_low_confidence',review['items'][0]['vision_blockers'])
        self.assertIn('waiting_current_single_object_stable_window',review['items'][0]['vision_blockers'])
        self.assertEqual(review['vision_pass_count'],0)

    def test_stale_or_wrong_class_temporal_window_rejected(self):
        for change in ('stale','other_class','duplicate_timestamp','jitter'):
            workspace,result=fixture()
            samples=result['temporal_observation']['samples']
            if change=='stale':
                result['frame_id']=21
            elif change=='other_class':
                samples[0]['identity']=('paper_cup','recyclable','LEFT_CELL',(100,100))
            elif change=='duplicate_timestamp':
                samples[1]['sensor_timestamp_ns']=samples[0]['sensor_timestamp_ns']
            else:
                samples[0]['camera_xyz_m'][2]=1.1
            self.assertEqual(build_task_review(result,workspace)['vision_pass_count'],0)

    def test_missing_alignment_route_conflict_and_invalid_depth_are_blocked(self):
        for change in ('alignment','route','depth'):
            workspace,result=fixture()
            if change=='alignment':
                workspace['camera_pose_revalidated']=False
            elif change=='route':
                result['items'][0]['target_zone']='LEFT_CELL'
            else:
                result['items'][0]['camera_xyz_m']=None
            self.assertEqual(build_task_review(result,workspace)['vision_pass_count'],0)

    def test_multiple_candidates_never_select_one_automatically(self):
        workspace,result=fixture()
        result['items'].append(copy.deepcopy(result['items'][0]))
        review=build_task_review(result,workspace)
        self.assertEqual(review['status'],'blocked_multiple_candidates')
        self.assertIsNone(review['selected_target'])
