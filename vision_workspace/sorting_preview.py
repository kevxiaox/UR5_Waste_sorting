"""ZED + teammate YOLO + candidate Base coordinates. No robot connection."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from detector import detect_waste, detector_info
from waste_sorting.camera import ZedSource
from waste_sorting.candidate_preview import load_candidate, candidate_provenance
from waste_sorting.vision_pipeline import VisionPreviewPipeline
from waste_sorting.temporal_preview import TemporalSurfaceObserver
from waste_sorting.task_review import build_task_review
from waste_sorting.zones import overlay, timestamp, validate_workspace

ROOT = Path(__file__).resolve().parent
WINDOW = 'YOLO sorting preview - CANDIDATE ONLY - ROBOT DISABLED'


def draw_preview(view,result,workspace):
    view = overlay(view,workspace)
    for item in result['items']:
        detector_item = item['detector_item']
        if not isinstance(detector_item,dict):
            continue
        point = detector_item.get('center_pixel')
        if not isinstance(point,list) or len(point) != 2 or any(type(v) is not int for v in point):
            continue
        x,y = point
        lines = [f"{item['target_zone'] or '-'} | {item['status']}"]
        if item['candidate_base_xyz_m'] is not None:
            lines.append('Candidate Base mm: '+str(np.round(np.asarray(item['candidate_base_xyz_m'])*1000,1).tolist()))
        for index,line in enumerate(lines):
            cv2.putText(view,line,(max(5,min(x,view.shape[1]-650)),max(20,min(y+25+index*22,view.shape[0]-8))),
                        cv2.FONT_HERSHEY_SIMPLEX,0.48,(0,255,255),1,cv2.LINE_AA)
    cv2.putText(view,'PREVIEW ONLY / UNVALIDATED EXTRINSICS / TCP UNVERIFIED / ROBOT DISABLED',
                (8,22),cv2.FONT_HERSHEY_SIMPLEX,0.53,(0,0,255),2)
    if 'task_review' in result:
        cv2.putText(view,'Task: '+result['task_review']['status'],(8,45),
                    cv2.FONT_HERSHEY_SIMPLEX,0.50,(0,255,255),1,cv2.LINE_AA)
    return view


def save_preview(output,frame,view,result,raw,workspace,config_hash,info,candidate):
    folder = output/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    for name,image in (('rgb.png',frame),('marked.png',view)):
        if not cv2.imwrite(str(folder/name),image):
            raise IOError('Failed to save '+name)
    payload = {'type':'sorting_preview','timestamp_utc':timestamp(),
               'workspace_snapshot':workspace,'workspace_sha256':config_hash,'detector':info,
               'candidate_provenance':candidate_provenance(candidate) if candidate is not None else None,
               'result':result}
    (folder/'preview.json').write_text(json.dumps(payload,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    (folder/'detector.json').write_text(raw+'\n',encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print('Saved:',folder/'preview.json')
    return folder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,help='Offline image inference, saves and exits; no live verification/depth')
    parser.add_argument('--config',type=Path,default=ROOT/'config/workspace_zones.json')
    parser.add_argument('--candidate-report',type=Path,default=ROOT/'data/calibration_analysis/candidate_report.json')
    parser.add_argument('--training-manifest',type=Path,default=ROOT/'data/calibration_analysis/training_manifest.json')
    parser.add_argument('--output',type=Path,default=ROOT/'data/sorting_previews')
    parser.add_argument('--scale',type=float,default=0.85)
    parser.add_argument('--preview-min-confidence',type=float,default=None,
                        help='Override confidence for this disabled-execution preview only; default uses current sorting policy')
    parser.add_argument('--observe-stability', action='store_true',
                        help='Observe 20 distinct processed ZED frames of one candidate surface; never enables execution')
    args = parser.parse_args()
    if not 0.2 <= args.scale <= 2:
        parser.error('--scale must be between 0.2 and 2')
    if args.preview_min_confidence is not None and not 0 <= args.preview_min_confidence <= 1:
        parser.error('--preview-min-confidence must be between 0 and 1')
    if args.image and args.observe_stability:
        parser.error('--observe-stability requires live matching RGB/XYZ frames')
    config_bytes = args.config.read_bytes()
    workspace = json.loads(config_bytes)
    validate_workspace(workspace,require_complete=True)
    workspace['camera_pose_revalidated'] = False
    config_hash = hashlib.sha256(config_bytes).hexdigest()
    info = detector_info()
    pipeline = VisionPreviewPipeline(detect_waste,preview_min_confidence=args.preview_min_confidence)
    print('Detector:',json.dumps(info,ensure_ascii=False))
    print(f'No robot or gripper connection. PREVIEW confidence threshold: {pipeline.preview_min_confidence:.2f}. Global sorting policy unchanged.')
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise ValueError('Cannot read image: '+str(args.image))
        result,view,raw = pipeline.process_frame(frame,None,workspace,frame_id=0)
        result['mode'] = 'offline_image_no_depth_or_live_revalidation'
        result['input_image'] = str(args.image.resolve())
        result['task_review'] = build_task_review(result,workspace)
        save_preview(args.output,frame,draw_preview(view,result,workspace),result,raw,
                     workspace,config_hash,info,None)
        return
    candidate = load_candidate(args.candidate_report,args.training_manifest,config_hash)
    pipeline = VisionPreviewPipeline(detect_waste,candidate,preview_min_confidence=args.preview_min_confidence)
    observer = TemporalSurfaceObserver() if args.observe_stability else None
    # Preload YOLO before opening ZED; one owner acquires matching RGB+XYZ.
    with ZedSource() as camera:
        source = workspace['source']
        if source.get('kind') != 'zed_live' or source.get('view') != 'LEFT' or source.get('serial_number') != camera.serial:
            raise ValueError('Workspace does not match this ZED camera')
        cv2.namedWindow(WINDOW,cv2.WINDOW_AUTOSIZE)
        print('Inspect polygons > V confirm alignment. S saves current frame/result. +/- zoom. Q quits.')
        print('Box centers are visible surface samples, not grasp centers. Keep scene stationary for checks.')
        try:
            frame_id = 0
            while True:
                frame,xyz = camera.read_with_xyz()
                capture_time = time.monotonic()
                sensor_timestamp_ns = int(camera.camera.get_timestamp(camera.sl.TIME_REFERENCE.IMAGE).get_nanoseconds()) if observer else None
                frame_id += 1
                start = time.monotonic()
                result,view,raw = pipeline.process_frame(frame,xyz,workspace,frame_id)
                result['capture_host_monotonic_s'] = capture_time
                result['processing_seconds'] = time.monotonic()-start
                result['mode'] = 'live_candidate_surface_preview'
                if observer:
                    result['temporal_observation'] = observer.observe(result,capture_time,sensor_timestamp_ns)
                    result['depth_scope'] = 'single_matching_frame_with_separate_temporal_diagnostic'
                result['task_review'] = build_task_review(result,workspace)
                view = draw_preview(view,result,workspace)
                h,w = frame.shape[:2]
                size = (round(w*args.scale),round(h*args.scale))
                footer = np.full((110 if observer else 85,size[0],3),20,np.uint8)
                count = sum(i['status'] == 'candidate_surface_preview_only' for i in result['items'])
                lines = [f"Alignment: {'VERIFIED' if workspace['camera_pose_revalidated'] else 'UNVERIFIED'} | V confirm | S save | +/- zoom | Q quit",
                         f"Frame {frame_id} | detected {len(result['items'])} | candidate surfaces {count} | processing {result['processing_seconds']:.2f}s",
                         f'PREVIEW confidence >= {pipeline.preview_min_confidence:.2f} | SINGLE RGB/XYZ FRAME | ROBOT DISABLED']
                if observer:
                    temporal = result['temporal_observation']
                    lines.append(f"Temporal {temporal['valid_processed_frames']}/20 | {temporal['status']}")
                for i,line in enumerate(lines):
                    cv2.putText(footer,line,(8,21+i*25),cv2.FONT_HERSHEY_SIMPLEX,0.47,(255,255,255),1)
                cv2.imshow(WINDOW,np.vstack((cv2.resize(view,size),footer)))
                key = cv2.waitKey(1)&0xFF
                if key in (27,ord('q')) or cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord('v'):
                    workspace['camera_pose_revalidated'] = True
                    if observer:
                        observer.reset()
                    print('Camera/table alignment confirmed for this live session only')
                elif key == ord('s'):
                    save_preview(args.output,frame,view,result,raw,workspace,config_hash,info,candidate)
                elif key in (ord('+'),ord('='),ord('-')):
                    args.scale = min(2,max(0.2,args.scale+(-0.1 if key == ord('-') else 0.1)))
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except (ValueError,KeyError,RuntimeError,OSError,cv2.error) as error:
        raise SystemExit(str(error))
