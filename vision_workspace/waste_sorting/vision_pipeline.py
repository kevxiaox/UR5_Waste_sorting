"""Frame-local preview stages. Own one instance per vision worker; no actuators."""
import json
import math

import numpy as np

from .candidate_preview import project_surface
from .decision import POLICY_PATH, classify_detection, finite_vector
from .localization import sample_surface
from .gripper import ToolOutputGripperPreview


def parse_detector_json(raw, image_size):
    def invalid_constant(value):
        raise ValueError('Nonfinite detector JSON: '+value)
    data = json.loads(raw,parse_constant=invalid_constant)
    if not isinstance(data,dict) or data.get('image_resolution') != list(image_size):
        raise ValueError('Detector resolution differs from original camera frame')
    items = data.get('items')
    if (not isinstance(items,list) or type(data.get('total_detected')) is not int or
            data['total_detected'] != len(items)):
        raise ValueError('Invalid detector item count')
    return items


def adapt_item(item, image_size):
    if not isinstance(item,dict):
        return None,'rejected_detector_item'
    if item.get('status') != 'OK':
        return None,'rejected_detector_status'
    bbox,center = item.get('bbox_xyxy'),item.get('center_pixel')
    if (not finite_vector(bbox,4) or not isinstance(center,list) or len(center) != 2 or
            any(type(v) is not int for v in center)):
        return None,'rejected_detector_geometry'
    w,h = image_size
    x1,y1,x2,y2 = bbox
    if not (0 <= x1 < x2 < w and 0 <= y1 < y2 < h and x1 <= center[0] <= x2 and y1 <= center[1] <= y2):
        return None,'rejected_detector_geometry'
    return {'class_name':item.get('class_name'),'confidence':item.get('confidence'),
            'bbox':bbox,'center_pixel':center},None


class VisionPreviewPipeline:
    """Acquire -> infer -> filter -> sample matching XYZ -> candidate -> preview.

    An inference callable can be injected for tests or another vision worker.
    Never share a camera between workers or apply results to a newer XYZ frame.
    """
    def __init__(self,infer,candidate=None,preview_min_confidence=None):
        self.infer = infer
        self.candidate = candidate
        self.gripper = ToolOutputGripperPreview()
        self.preview_min_confidence = (json.loads(POLICY_PATH.read_text(encoding='utf-8'))['min_confidence']
                                       if preview_min_confidence is None else preview_min_confidence)
        if (type(self.preview_min_confidence) not in (float,int) or
                not math.isfinite(self.preview_min_confidence) or not 0 <= self.preview_min_confidence <= 1):
            raise ValueError('Preview confidence must be finite and between 0 and 1')

    def process_frame(self,frame,xyz,workspace,frame_id):
        h,w = frame.shape[:2]
        if workspace['image_size'] != [w,h]:
            raise ValueError('Workspace and frame sizes differ')
        raw,view = self.infer(frame)
        if not isinstance(view,np.ndarray) or view.shape != frame.shape:
            raise ValueError('Detector changed visualization resolution')
        items = parse_detector_json(raw,[w,h])
        output = []
        for index,item in enumerate(items):
            entry = {'item_index':index,'detector_item':item,'status':None,'category':None,
                     'target_zone':None,'camera_xyz_m':None,'candidate_base_xyz_m':None,
                     'base_xyz_m':None,'grasp_pose':None,'trajectory':None,
                     'execution_status':'disabled','robot_execution_enabled':False}
            detection,error = adapt_item(item,[w,h])
            if error:
                output.append(dict(entry,status=error))
                continue
            route = classify_detection(detection,preview_min_confidence=self.preview_min_confidence)
            entry.update(route)
            if route['status'] != 'ready_for_localization':
                output.append(entry)
                continue
            # Caller-provided waste_category never chooses the physical bin.
            expected = {'recyclable':'Recyclable','residual':'General_Waste'}[route['category']]
            if item.get('waste_category') != expected:
                output.append(dict(entry,status='rejected_category_mismatch'))
                continue
            sample = sample_surface(xyz,detection['center_pixel'],workspace)
            entry['depth_sample'] = sample
            entry['status'] = sample['status']
            if sample['status'] == 'valid_camera_sample':
                entry['camera_xyz_m'] = sample['camera_xyz_m']
                if self.candidate is None:
                    entry['status'] = 'waiting_candidate_transform'
                else:
                    preview = project_surface(sample,detection['center_pixel'],workspace,self.candidate)
                    entry['candidate_preview'] = preview
                    entry['candidate_base_xyz_m'] = preview['candidate_base_xyz_m']
                    entry['status'] = preview['status']
            output.append(entry)
        return {'frame_id':frame_id,'image_resolution':[w,h], 'items':output,
                'gripper_control_preview':{'open':self.gripper.preview('open'),
                                           'close':self.gripper.preview('close')},
                'preview_min_confidence':self.preview_min_confidence,
                'confidence_scope':'preview_only_global_sorting_policy_unchanged',
                'robot_execution_enabled':False,'camera_to_base_calibrated':False,'tcp_verified':False,
                'independent_physical_validation_passed':False,
                'depth_scope':'single_matching_frame_not_temporal_stability_check',
                'note':'Box center samples a visible surface, not necessarily object top or grasp center.',
                'task_outline':['detect','filter','sample_matching_depth','candidate_base_preview',
                                'future_validate_grasp','future_plan_approach','future_grasp','future_place'],
                'trajectory':None,'execution_status':'disabled'},view,raw
