"""Capture scheduled raw ZED RGB samples without inference or robot connection."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from waste_sorting.camera import ZedSource
from waste_sorting.dataset_capture import DatasetCapture, SCENARIOS
from waste_sorting.zones import overlay, validate_workspace

ROOT = Path(__file__).resolve().parent
WINDOW = 'Evaluation RGB capture - NO MODEL / NO ROBOT'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', choices=SCENARIOS)
    parser.add_argument('--count', type=int, default=5)
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()
    if not 1 <= args.count <= 20 or not np.isfinite(args.interval) or not 1 <= args.interval <= 10:
        parser.error('count must be 1..20; interval must be 1..10 seconds')
    raw = (ROOT/'config/workspace_zones.json').read_bytes()
    workspace = json.loads(raw)
    validate_workspace(workspace, require_complete=True)
    workspace['camera_pose_revalidated'] = False
    folder = ROOT/'data/vision_evaluation/captures'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    session = DatasetCapture(folder, workspace, hashlib.sha256(raw).hexdigest(), args.scenario, args.count)
    session.data.update(interval_minimum_s=args.interval, countdown_s=3, camera_serial=None)
    status = 'aborted'
    print('No model or robot connection. Inspect zones > V confirm > B starts after 3 seconds. Q exits.')
    print('All scheduled attempts retained; no confidence-based selection. Labels require later review.')
    print('Session:', folder/'session.json')
    try:
        with ZedSource() as camera:
            source = workspace['source']
            if (source.get('kind') != 'zed_live' or source.get('view') != 'LEFT'
                    or source.get('serial_number') != camera.serial):
                raise ValueError('Workspace camera identity mismatch')
            session.data['camera_serial'] = camera.serial
            session.flush()
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
            next_due = None
            while True:
                frame = camera.read()
                now = time.monotonic()
                sensor_ts = int(camera.camera.get_timestamp(camera.sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
                if next_due is not None and now >= next_due:
                    attempt = session.record(frame, sensor_ts, now)
                    print(f"Attempt {attempt['attempt']}/{args.count}: {attempt['status']}", attempt.get('error', ''), flush=True)
                    if len(session.data['attempts']) == args.count:
                        status = 'completed' if all(a['status'] == 'saved' for a in session.data['attempts']) else 'completed_with_failed_attempts'
                        break
                    # No burst backfilling after delays; keep actual timestamps.
                    next_due = time.monotonic()+args.interval
                view = overlay(frame.copy(), session.workspace)
                if next_due is None:
                    message = 'Inspect zones > V confirm > B begin | Q exit'
                else:
                    message = f'Keep hands clear | next in {max(0,next_due-now):.1f}s | Q abort'
                for i, text in enumerate((args.scenario+' / NO MODEL / NO ROBOT', message,
                                          f"Attempts {len(session.data['attempts'])}/{args.count} / labels UNREVIEWED")):
                    cv2.putText(view,text,(8,24+i*25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,255),2)
                cv2.imshow(WINDOW,cv2.resize(view,None,fx=.85,fy=.85))
                key = cv2.waitKey(20)&255
                if key in (27,ord('q')) or cv2.getWindowProperty(WINDOW,cv2.WND_PROP_VISIBLE)<1:
                    break
                if key == ord('v') and next_due is None:
                    session.confirm_alignment()
                    print('Alignment confirmed for this session only.')
                elif key == ord('b') and next_due is None:
                    if not session.workspace['camera_pose_revalidated']:
                        print('Inspect zones and press V first.')
                        continue
                    session.data['status'] = 'capturing'
                    session.flush()
                    next_due = time.monotonic()+3
                    print('Starting in 3 seconds; keep scene unchanged and hands clear.')
    except BaseException as error:
        status = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'error'
        session.data['error'] = str(error)
        raise
    finally:
        session.finish(status)
        cv2.destroyAllWindows()
        print('Saved session:', folder/'session.json')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, cv2.error) as error:
        raise SystemExit(str(error))
