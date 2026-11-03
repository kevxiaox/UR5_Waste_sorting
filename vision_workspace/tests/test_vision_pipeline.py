"""Original-pixel JSON contract, matching-frame depth and disabled execution."""
import copy
import json
from types import SimpleNamespace
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

import detector
from waste_sorting.vision_pipeline import VisionPreviewPipeline, parse_detector_json
from waste_sorting.decision import classify_detection
from waste_sorting.zones import new_workspace


def fixture():
    workspace = new_workspace(40,30)
    workspace['camera_pose_revalidated'] = True
    workspace['zones']['PICK_ZONE']['polygon_px'] = [[2,2],[36,2],[36,26],[2,26]]
    frame = np.zeros((30,40,3),np.uint8)
    xyz = np.full((30,40,3),[0.1,0.2,0.7],dtype=np.float32)
    item = {'class_name':'sponge','waste_category':'General_Waste','confidence':0.95,
            'bbox_xyxy':[6,6,14,14],'center_pixel':[10,10],'status':'OK'}
    return workspace,frame,xyz,item


def pipeline_for(items, candidate=True,preview_min_confidence=None):
    def infer(frame):
        h,w = frame.shape[:2]
        return json.dumps({'image_resolution':[w,h],'total_detected':len(items),'items':items}),frame.copy()
    return VisionPreviewPipeline(infer,{'transform':np.eye(4)} if candidate else None,
                                 preview_min_confidence=preview_min_confidence)


class VisionTests(unittest.TestCase):
    def test_preview_threshold_does_not_change_policy_or_bypass_other_gates(self):
        workspace,frame,xyz,item = fixture()
        item['confidence'] = 0.562886
        default,_,_ = pipeline_for([item]).process_frame(frame,xyz,workspace,1)
        preview,_,_ = pipeline_for([item],preview_min_confidence=0.45).process_frame(frame,xyz,workspace,2)
        self.assertEqual(default['items'][0]['status'],'rejected_low_confidence')
        self.assertEqual(preview['items'][0]['status'],'candidate_surface_preview_only')
        self.assertEqual(preview['preview_min_confidence'],0.45)
        self.assertFalse(preview['robot_execution_enabled'])
        detection = {'class_name':'sponge','confidence':0.562886,'bbox':[6,6,14,14],'center_pixel':[10,10]}
        self.assertEqual(classify_detection(detection)['status'],'rejected_low_confidence')
        for modified in (dict(item,status='REJECTED_LOW_CONF'),dict(item,class_name='UNKNOWN')):
            result,_,_ = pipeline_for([modified],preview_min_confidence=0.45).process_frame(frame,xyz,workspace,3)
            self.assertIsNone(result['items'][0]['candidate_base_xyz_m'])
        workspace['camera_pose_revalidated'] = False
        result,_,_ = pipeline_for([item],preview_min_confidence=0.45).process_frame(frame,None,workspace,4)
        self.assertEqual(result['items'][0]['status'],'waiting_camera_revalidation')
        for threshold in (float('nan'),-0.1,1.1,True):
            with self.assertRaises(ValueError):
                pipeline_for([item],preview_min_confidence=threshold)

    def test_all_handover_routes_and_preview_only(self):
        workspace,frame,xyz,item = fixture()
        for name,category,target in [('plastic_bottle','Recyclable','LEFT_CELL'),
                                     ('aluminum_can','Recyclable','LEFT_CELL'),
                                     ('paper_cup','Recyclable','LEFT_CELL'),
                                     ('sponge','General_Waste','RIGHT_CELL')]:
            result,_,_ = pipeline_for([dict(item,class_name=name,waste_category=category)]).process_frame(frame,xyz,workspace,7)
            entry = result['items'][0]
            self.assertEqual(entry['target_zone'],target)
            self.assertEqual(entry['status'],'candidate_surface_preview_only')
            np.testing.assert_allclose(entry['candidate_base_xyz_m'],xyz[10,10])
            self.assertIsNone(entry['base_xyz_m'])
            self.assertIsNone(entry['grasp_pose'])
            self.assertIsNone(result['trajectory'])
            self.assertFalse(result['robot_execution_enabled'])
            self.assertEqual(result['frame_id'],7)

    def test_non_ok_low_conf_unknown_and_category_mismatch_rejected(self):
        workspace,frame,xyz,item = fixture()
        for change,status in [({'status':'REJECTED_LOW_CONF'},'rejected_detector_status'),
                              ({'confidence':0.59},'rejected_low_confidence'),
                              ({'class_name':'banana'},'rejected_unknown_class'),
                              ({'waste_category':'Recyclable'},'rejected_category_mismatch')]:
            result,_,_ = pipeline_for([dict(item,**change)]).process_frame(frame,xyz,workspace,1)
            entry = result['items'][0]
            self.assertEqual(entry['status'],status)
            self.assertIsNone(entry['candidate_base_xyz_m'])

    def test_no_stale_depth_and_invalid_center_is_not_replaced(self):
        workspace,frame,xyz,item = fixture()
        pipeline = pipeline_for([item])
        first,_,_ = pipeline.process_frame(frame,xyz,workspace,1)
        xyz[10,10] = np.nan
        second,_,_ = pipeline.process_frame(frame,xyz,workspace,2)
        self.assertIsNotNone(first['items'][0]['candidate_base_xyz_m'])
        self.assertEqual(second['items'][0]['status'],'rejected_invalid_center_depth')
        self.assertIsNone(second['items'][0]['candidate_base_xyz_m'])

    def test_live_verification_and_pick_gates(self):
        workspace,frame,xyz,item = fixture()
        workspace['camera_pose_revalidated'] = False
        result,_,_ = pipeline_for([item]).process_frame(frame,None,workspace,1)
        self.assertEqual(result['items'][0]['status'],'waiting_camera_revalidation')
        workspace['camera_pose_revalidated'] = True
        outside = dict(item,bbox_xyxy=[0,0,2,2],center_pixel=[1,1])
        result,_,_ = pipeline_for([outside]).process_frame(frame,xyz,workspace,2)
        self.assertEqual(result['items'][0]['status'],'rejected_outside_pick_zone')

    def test_invalid_json_resolution_count_and_nonfinite(self):
        for raw in ('{}','[]','{"image_resolution":[40,30],"total_detected":1,"items":[]}',
                    '{"image_resolution":[40,30],"total_detected":0,"items":[],"bad":NaN}',
                    '{"image_resolution":[80,60],"total_detected":0,"items":[]}'):
            with self.assertRaises(ValueError):
                parse_detector_json(raw,[40,30])

    def test_bad_geometry_empty_detections_and_missing_transform(self):
        workspace,frame,xyz,item = fixture()
        for change in ({'bbox_xyxy':[0,0,100,100]}, {'center_pixel':[10.5,10]},
                       {'center_pixel':[True,10]}, {'bbox_xyxy':[14,6,6,14]}):
            result,_,_ = pipeline_for([dict(item,**change)]).process_frame(frame,xyz,workspace,1)
            self.assertEqual(result['items'][0]['status'],'rejected_detector_geometry')
        result,_,_ = pipeline_for([]).process_frame(frame,xyz,workspace,2)
        self.assertEqual(result['items'],[])
        result,_,_ = pipeline_for([item],candidate=False).process_frame(frame,xyz,workspace,3)
        self.assertEqual(result['items'][0]['status'],'waiting_candidate_transform')

    def test_detector_maps_square_coordinates_to_original_not_display(self):
        coordinates = MagicMock()
        coordinates.detach.return_value.cpu.return_value.tolist.return_value = [128,64,256,256]
        box = SimpleNamespace(conf=[0.92],cls=[0],xyxy=[coordinates])
        model = MagicMock()
        model.names = {0:'sponge'}
        model.predict.return_value = [SimpleNamespace(boxes=[box])]
        frame = np.zeros((720,1280,3),np.uint8)
        with patch.object(detector,'_load_model',return_value=model):
            raw,view = detector.detect_waste(frame)
        entry = json.loads(raw)['items'][0]
        self.assertEqual(entry['bbox_xyxy'],[320,90,640,360])
        self.assertEqual(entry['center_pixel'],[480,225])
        self.assertEqual(view.shape,frame.shape)
        self.assertFalse(frame.any())
        self.assertEqual(model.predict.call_args.kwargs['source'].shape,(512,512,3))


if __name__ == '__main__':
    unittest.main()
