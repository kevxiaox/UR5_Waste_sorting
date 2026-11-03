"""Draw source CAD edges with the selected mounting reference highlighted."""
import json
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tmp/cad_tools'))
import gmsh


def main():
    folder = ROOT/'data/cad_review/rg6_20261023'
    ref = json.loads((folder/'cad_nominal_tcp.json').read_text(encoding='utf-8'))
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    try:
        import hashlib
        if hashlib.sha256(Path(ref['source_path']).read_bytes()).hexdigest() != ref['source_sha256']:
            raise ValueError('Source CAD hash mismatch')
        gmsh.model.occ.importShapes(ref['source_path'])
        gmsh.model.occ.synchronize()
        selected = set()
        for face in ref['reference_faces']:
            selected.update(abs(t) for _, t in gmsh.model.getBoundary([(2, face['face_tag'])]))
        curves = []
        for _, tag in gmsh.model.getEntities(1):
            lo, hi = gmsh.model.getParametrizationBounds(1, tag)
            n = 2 if gmsh.model.getType(1, tag) == 'Line' else 32
            p = np.asarray(gmsh.model.getValue(1, tag, np.linspace(lo[0], hi[0], n))).reshape(-1, 3)
            curves.append((tag in selected, p))
        canvas = np.full((760, 1100, 3), 250, np.uint8)
        # Exact CAD orthographic projections; no flange axes inferred.
        for axes, y0, title in [((0, 1), 65, 'CAD XY: rear interface (red), selected pad faces (red)'),
                                ((0, 2), 410, 'CAD XZ: inspect the wrist-to-gripper attachment here')]:
            all_points = np.vstack([p[:, axes] for _, p in curves])
            low, high = all_points.min(axis=0), all_points.max(axis=0)
            scale = min(960/(high[0]-low[0]), 240/(high[1]-low[1]))
            mid = (high+low)/2
            def pixel(points):
                xy = np.atleast_2d(points)[:, axes]
                return np.column_stack(((xy[:, 0]-mid[0])*scale+550,
                                        -(xy[:, 1]-mid[1])*scale+y0+150)).round().astype(np.int32)
            for highlight, points in sorted(curves, key=lambda pair: pair[0]):
                cv2.polylines(canvas, [pixel(points)], False,
                              (30, 40, 220) if highlight else (185, 180, 170),
                              2 if highlight else 1, cv2.LINE_AA)
            origin = pixel(ref['datum_origin_cad_mm'])[0]
            target = pixel(ref['nominal_point_cad_mm'])[0]
            for pt, label, color in [(origin, 'A: mounting reference', (30, 40, 220)),
                                      (target, 'B: CAD nominal point', (0, 140, 0))]:
                cv2.drawMarker(canvas, tuple(pt), color, cv2.MARKER_CROSS, 18, 2)
                offset = np.array([20, -105 if label.startswith('A') else 95])
                text_pos = pt+offset
                text_pos[0] = min(text_pos[0], 820)
                cv2.arrowedLine(canvas, tuple(text_pos+[0, 8]), tuple(pt), color, 2, tipLength=.08)
                cv2.putText(canvas, label, tuple(text_pos), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
            cv2.putText(canvas, title, (25, y0), cv2.FONT_HERSHEY_SIMPLEX, .62, (30, 30, 30), 1, cv2.LINE_AA)
        cv2.putText(canvas, 'RG6 supplied STEP: interface inspection reference', (25, 30), cv2.FONT_HERSHEY_SIMPLEX, .8, (20, 20, 20), 2)
        cv2.putText(canvas, 'A-to-B: 230.538 mm axial in selected datum; shown opening: 103.168 mm', (25, 720), cv2.FONT_HERSHEY_SIMPLEX, .6, (20, 20, 20), 1)
        cv2.putText(canvas, 'CAD geometry only. UR flange registration pending. No robot command.', (25, 746), cv2.FONT_HERSHEY_SIMPLEX, .55, (30, 40, 180), 1)
        output = folder/'mount_reference.png'
        if not cv2.imwrite(str(output), canvas):
            raise OSError('Could not save reference image')
        print('Saved:', output)
    finally:
        gmsh.finalize()


if __name__ == '__main__':
    main()
