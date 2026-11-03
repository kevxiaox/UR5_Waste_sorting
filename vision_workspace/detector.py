"""Local adapter for the teammate's detect_waste(BGR) -> (JSON, BGR) contract.

Preserves the supplied 512x512 stretch and CPU inference. Original handover files
remain in Vision_Module_Final.zip/; that path is a directory on this computer.
"""
import hashlib
import json
import os
from pathlib import Path
import threading

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT/'Vision_Module_Final.zip'/'best.pt'
CATEGORY_MAP = {'plastic_bottle':'Recyclable','aluminum_can':'Recyclable',
                'paper_cup':'Recyclable','sponge':'General_Waste'}
_model = None
_lock = threading.Lock()


def _load_model():
    global _model
    if _model is None:
        if not MODEL_PATH.is_file():
            raise FileNotFoundError(MODEL_PATH)
        os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'data/ultralytics_config'))
        from ultralytics import YOLO
        _model = YOLO(str(MODEL_PATH))
    return _model


def detector_info():
    with _lock:
        model = _load_model()
        return {'weights_path':str(MODEL_PATH),
                'weights_sha256':hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
                'names':model.names,'device':'cpu','preprocessing':'stretch_512x512_then_original_pixel_mapping'}


def detect_waste(image_bgr, conf_threshold=0.45):
    if (not isinstance(image_bgr,np.ndarray) or image_bgr.dtype != np.uint8 or
            image_bgr.ndim != 3 or image_bgr.shape[2] != 3 or min(image_bgr.shape[:2]) < 2):
        raise ValueError('detect_waste requires a nonempty uint8 BGR image')
    if type(conf_threshold) not in (int,float) or not 0 <= conf_threshold <= 1:
        raise ValueError('Invalid confidence threshold')
    h,w = image_bgr.shape[:2]
    view = image_bgr.copy()
    resized = cv2.resize(image_bgr,(512,512))
    with _lock:
        model = _load_model()
        result = model.predict(source=resized,device='cpu',verbose=False)[0]
        boxes = [(float(b.conf[0]),str(model.names[int(b.cls[0])]),
                  b.xyxy[0].detach().cpu().tolist()) for b in result.boxes]
    items = []
    for confidence,name,box in boxes:
        x1,y1,x2,y2 = [int(v*s) for v,s in zip(box,(w/512,h/512,w/512,h/512))]
        x1,x2 = [max(0,min(w-1,x)) for x in (x1,x2)]
        y1,y2 = [max(0,min(h-1,y)) for y in (y1,y2)]
        center = [(x1+x2)//2,(y1+y2)//2]
        status = ('REJECTED_UNKNOWN_CLASS' if name not in CATEGORY_MAP else
                  'REJECTED_LOW_CONF' if confidence < conf_threshold else 'OK')
        items.append({'class_name':name,'waste_category':CATEGORY_MAP.get(name,'UNKNOWN'),
                      'confidence':confidence,'bbox_xyxy':[x1,y1,x2,y2],
                      'center_pixel':center,'status':status})
        color = (0,200,0) if status == 'OK' else (0,165,255)
        cv2.rectangle(view,(x1,y1),(x2,y2),color,2)
        cv2.circle(view,tuple(center),5,(0,0,255),-1)
        cv2.putText(view,f'{name} {confidence:.2f}',(x1,max(20,y1-7)),
                    cv2.FONT_HERSHEY_SIMPLEX,0.55,color,2)
    return json.dumps({'image_resolution':[w,h],'total_detected':len(items),'items':items},
                      ensure_ascii=False,allow_nan=False),view
