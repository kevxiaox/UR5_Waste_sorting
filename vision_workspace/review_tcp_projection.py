"""Annotate a SAVED image to diagnose projection mismatch. No hardware imports."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from waste_sorting.projection_review import review_annotations, sample_geometry, project

ROOT = Path(__file__).resolve().parent
WINDOW = 'OFFLINE image review - NO ROBOT CONNECTION'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sample', type=Path, required=True, help='Explicit saved sample.json')
    parser.add_argument('--logo-pixel', type=float, nargs=2)
    parser.add_argument('--jaw-pixels', type=float, nargs=4)
    args = parser.parse_args()
    raw = args.sample.read_bytes()
    sample = json.loads(raw)
    rgb_path = args.sample.parent/'rgb.png'
    frame = cv2.imread(str(rgb_path))
    if frame is None or [frame.shape[1], frame.shape[0]] != sample['image_resolution']:
        raise ValueError('Saved original RGB missing or resolution differs')
    transform, intrinsics = sample_geometry(sample)
    colors = [(0, 255, 255), (255, 180, 0), (255, 0, 255)]
    points = []
    if (args.logo_pixel is None) != (args.jaw_pixels is None):
        raise ValueError('Supply both --logo-pixel and --jaw-pixels or neither')
    if args.logo_pixel is not None:
        points = [args.logo_pixel, args.jaw_pixels[:2], args.jaw_pixels[2:]]
    else:
        # Crop and enlarge from the original, preserving exact click mapping.
        projected = np.array([p['pixel'] for p in sample['points'] if p['pixel'] is not None])
        low = np.maximum(0, np.floor(projected.min(axis=0)-160)).astype(int)
        high = np.minimum(sample['image_resolution'], np.ceil(projected.max(axis=0)+160)).astype(int)
        x0, y0 = low
        x1, y1 = high
        scale = min(2., 1050/(x1-x0), 620/(y1-y0))
        width, height = round((x1-x0)*scale), round((y1-y0)*scale)
        sx, sy = width/(x1-x0), height/(y1-y0)

        def mouse(event, x, y, flags, context):
            if event == cv2.EVENT_LBUTTONDOWN and len(points) < 3 and 0 <= x < width and 0 <= y < height:
                points.append([float(x/sx+x0), float(y/sy+y0)])
            elif event == cv2.EVENT_RBUTTONDOWN and points:
                points.pop()

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, mouse)
        print('OFFLINE: uses the saved image, not current robot state. No motion or I/O.')
        print('Click: 1 blue LOGO center; 2 jaw A inner contact middle; 3 jaw B inner contact middle.')
        print('Do not guess hidden contact locations. Q quits; right-click undoes; Enter saves all three.')
        labels = ['Click blue LOGO center', 'Click jaw A inner contact-face middle',
                  'Click jaw B inner contact-face middle', 'Enter/S save | right-click undo | Q quit']
        try:
            while True:
                view = frame.copy()
                for i, point in enumerate(sample['points']):
                    if point['pixel'] is not None:
                        cv2.drawMarker(view, tuple(map(round, point['pixel'])), colors[i], cv2.MARKER_CROSS, 18, 2)
                for i, pixel in enumerate(points):
                    pt = tuple(map(round, pixel))
                    cv2.circle(view, pt, 4, (0, 0, 255), 2)
                    cv2.putText(view, 'P'+str(i+1), (pt[0]+6, pt[1]+12), cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 0, 255), 1)
                zoom = cv2.resize(view[y0:y1, x0:x1], (width, height))
                footer = np.full((90, width, 3), 20, np.uint8)
                for i, text in enumerate(['SAVED IMAGE ONLY - not live', labels[len(points)],
                                          'If a contact middle is hidden, Q quits without a result.']):
                    cv2.putText(footer, text, (8, 22+i*27), cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1)
                cv2.imshow(WINDOW, np.vstack((zoom, footer)))
                key = cv2.waitKey(30)&255
                if key in (27, ord('q')) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    print('Cancelled; no annotation saved.')
                    return
                if key in (10, 13, ord('s')) and len(points) == 3:
                    break
        finally:
            cv2.destroyAllWindows()

    result = review_annotations(sample, points[0], points[1:])
    result['sources'] = [{'path': str(p.resolve()), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                         for p in (args.sample, rgb_path)]
    output = ROOT/'data/tcp_projection_reviews'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    (output/'review.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print('Logo reprojection error px:', result['logo_reprojection_error_px'])
    print('2D jaw midpoint to projected flange-axis line px:', result['jaw_midpoint_to_axis_line_px'])
    print('Diagnostic only: no physical pass/fail, millimetre correction or TCP setting.')
    print('Saved:', output/'review.json')


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, cv2.error) as error:
        raise SystemExit(str(error))
