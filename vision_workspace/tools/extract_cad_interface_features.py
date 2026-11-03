"""Offline circular and planar features for paired CAD interface registration."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tmp/cad_tools'))
import gmsh


def main():
    jobs = [
        ('rg6', Path(r'C:\Users\KEVXI\Downloads\RG6_v2.STEP'),
         ROOT/'data/cad_review/rg6_20261023'),
        ('quick_changer', Path(r'C:\Users\KEVXI\Downloads\Quick Changer Robot side_v2_with_cable_holder.STEP'),
         ROOT/'data/cad_review/quick_changer_20261023'),
    ]
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    try:
        for name, source, output in jobs:
            gmsh.clear()
            gmsh.model.occ.importShapes(str(source))
            gmsh.model.occ.synchronize()
            circles, planes = [], []
            for _, tag in gmsh.model.getEntities(1):
                if gmsh.model.getType(1, tag) != 'Circle':
                    continue
                bounds = gmsh.model.getBoundingBox(1, tag)
                if name == 'rg6' and bounds[0] > 75:
                    continue
                lo, hi = gmsh.model.getParametrizationBounds(1, tag)
                uv = np.linspace(lo[0], hi[0], 12)
                points = np.asarray(gmsh.model.getValue(1, tag, uv)).reshape(-1, 3)
                mean = points.mean(axis=0)
                local = points-mean
                _, singular, vt = np.linalg.svd(local, full_matrices=False)
                if singular[1] < 1e-8:
                    continue
                basis = vt[:2].T
                xy = local@basis
                ab = np.column_stack((2*xy, np.ones(len(xy))))
                fit = np.linalg.lstsq(ab, (xy*xy).sum(axis=1), rcond=None)[0]
                center = mean + basis@fit[:2]
                radii = np.linalg.norm(points-center, axis=1)
                if np.ptp(radii) > 1e-5:
                    raise ValueError('Noncircular curve returned as Circle')
                circles.append({'edge': tag, 'center_mm': center.tolist(),
                                'radius_mm': float(radii.mean()),
                                'normal': vt[2].tolist()})
            for _, tag in gmsh.model.getEntities(2):
                if gmsh.model.getType(2, tag) != 'Plane':
                    continue
                bounds = gmsh.model.getBoundingBox(2, tag)
                if name == 'rg6' and bounds[0] > 75:
                    continue
                planes.append({'face': tag, 'area_mm2': gmsh.model.occ.getMass(2, tag),
                               'center_mm': list(gmsh.model.occ.getCenterOfMass(2, tag)),
                               'bounds_mm': list(bounds)})
            result = {'source_path': str(source),
                      'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
                      'circles': circles, 'planes': planes,
                      'scope': 'CAD geometry only; no robot connection or transform inferred'}
            target = output/'interface_features.json'
            target.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
            print(name, 'circle/plane counts:', len(circles), len(planes), flush=True)
    finally:
        gmsh.finalize()


if __name__ == '__main__':
    main()
