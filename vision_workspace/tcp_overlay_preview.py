"""ZED + output-only robot snapshots. Frozen 2D hypotheses; NEVER actuates."""
from datetime import datetime
import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from waste_sorting.camera import ZedSource
from waste_sorting.candidate_preview import load_candidate, candidate_provenance
from waste_sorting.point_calibration import analyze_observations
from waste_sorting.tcp_overlay import capture_overlay, axial_hypothesis
from waste_sorting.zones import overlay, validate_workspace

ROOT = Path(__file__).resolve().parent
WINDOW = 'READ ONLY TCP hypothesis check - NO MOTION'
COLORS = [(0, 255, 255), (255, 180, 0), (255, 0, 255)]


def draw(frame, workspace, result, axial_hypothesis_mm=None):
    view = overlay(frame.copy(), workspace)
    lines = ['READ ONLY / UNVALIDATED EXTRINSICS AND TCP',
             ('195mm is a SPONGE-GRIP HYPOTHESIS, not current open-jaw center'
              if axial_hypothesis_mm is None else
              f'{axial_hypothesis_mm:g}mm AXIAL ASSUMPTION / CAD mount not registered / preview only'),
             'FROZEN SAMPLE: S save / R return to live / Q exit' if result else
             'LIVE: inspect zones > V confirm > M stationary sample / Q exit']
    if result:
        for i, point in enumerate(result['points']):
            color = COLORS[i]
            lines.append(point['name']+' | '+point['status'])
            if point['pixel'] is not None:
                x, y = map(round, point['pixel'])
                cv2.drawMarker(view, (x, y), color, cv2.MARKER_CROSS, 22, 2)
                cv2.putText(view, str(i+1), (x+10, y-10), cv2.FONT_HERSHEY_SIMPLEX, .7, color, 2)
    for i, line in enumerate(lines):
        y = 22+24*i
        cv2.rectangle(view, (0, y-19), (view.shape[1], y+4), (20, 20, 20), -1)
        color = COLORS[i-3] if 3 <= i < 6 else (255, 255, 255)
        cv2.putText(view, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1, cv2.LINE_AA)
    return view


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--axial-hypothesis-mm', type=float, default=None,
                        help='Display-only assumed flange Z offset in mm; does not set robot TCP')
    args = parser.parse_args()
    hypothesis = axial_hypothesis(args.axial_hypothesis_mm)
    config = ROOT/'config/workspace_zones.json'
    raw = config.read_bytes()
    workspace = json.loads(raw)
    validate_workspace(workspace, require_complete=True)
    workspace['camera_pose_revalidated'] = False
    report_path = ROOT/'data/calibration_analysis/candidate_report.json'
    manifest_path = ROOT/'data/calibration_analysis/training_manifest.json'
    candidate = load_candidate(report_path, manifest_path, hashlib.sha256(raw).hexdigest())
    report = json.loads(report_path.read_text(encoding='utf-8'))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    checked = analyze_observations(report['training_files']+report['held_out_files'], manifest)
    logo_offset = checked['candidate_feature_offset_flange_m']
    if not np.allclose(logo_offset, report['candidate_feature_offset_flange_m'], rtol=0, atol=1e-10):
        raise ValueError('Logo offset differs from reproduced fit')
    print('No motion or I/O writes. M reads output-only RTDE; keep robot program STOPPED and robot stationary.')
    print('V confirms image alignment only. M freezes a new bracketed image. S saves; R resumes live; Q exits.')
    print(hypothesis['name']+'; assumed X=Y=0. No flange registration or depth measurement.')
    with ZedSource() as camera:
        source = workspace['source']
        if (source.get('kind') != 'zed_live' or source.get('view') != 'LEFT'
                or source.get('serial_number') != camera.serial):
            raise ValueError('Workspace camera identity mismatch')
        calibration = camera.camera.get_camera_information().camera_configuration.calibration_parameters.left_cam
        intrinsics = (calibration.fx, calibration.fy, calibration.cx, calibration.cy)
        result = None
        frame = camera.read()
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        try:
            while True:
                if result is None:
                    frame = camera.read()
                if [frame.shape[1], frame.shape[0]] != workspace['image_size']:
                    raise ValueError('Workspace and image resolution differ')
                view = draw(frame, workspace, result, args.axial_hypothesis_mm)
                cv2.imshow(WINDOW, cv2.resize(view, None, fx=.85, fy=.85))
                key = cv2.waitKey(30) & 255
                if key in (27, ord('q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                if key == ord('v') and result is None:
                    workspace['camera_pose_revalidated'] = True
                    print('Alignment confirmed for this session only; not external calibration or TCP verification.')
                elif key == ord('r'):
                    result = None
                elif key == ord('m') and result is None:
                    if not workspace['camera_pose_revalidated']:
                        print('Inspect zone alignment and press V first.')
                        continue
                    try:
                        frame, result = capture_overlay(camera, candidate, logo_offset, intrinsics,
                                                        axial_hypothesis_mm=args.axial_hypothesis_mm)
                        result.update(camera_serial=camera.serial, image_resolution=[frame.shape[1], frame.shape[0]],
                                      workspace_snapshot=json.loads(json.dumps(workspace)),
                                      candidate_provenance=candidate_provenance(candidate))
                        print('FROZEN: do not interpret this as current state after moving. S saves; R resumes.')
                        for point in result['points']:
                            print(point['name'], point['status'], point['pixel'])
                    except (OSError, ValueError, RuntimeError, KeyError) as error:
                        result = None
                        print('Sample rejected; no overlay retained:', error)
                elif key == ord('s') and result is not None:
                    folder = ROOT/'data/tcp_overlay_checks'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
                    folder.mkdir(parents=True, exist_ok=False)
                    for name, picture in (('rgb.png', frame), ('marked.png', view)):
                        if not cv2.imwrite(str(folder/name), picture):
                            raise OSError('Could not save '+name)
                    (folder/'sample.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
                    print('Saved:', folder/'sample.json')
        finally:
            cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, RuntimeError, KeyError, cv2.error) as error:
        raise SystemExit(str(error))
