"""Derive an explicitly defined CAD nominal point; never write robot settings."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tmp/cad_tools'))
import gmsh

SOURCE = Path(r'C:\Users\KEVXI\Downloads\RG6_v2.STEP')
SHA256 = '40c123b5ba556445deeccc31cca708884edce824ff862461e3be51c6b6383206'
OUTPUT = ROOT / 'data/cad_review/rg6_20261023/cad_nominal_tcp.json'


def main():
    digest = hashlib.sha256(SOURCE.read_bytes()).hexdigest()
    if digest != SHA256:
        raise ValueError('STEP changed: face tags and datum selection must be reviewed again')
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    try:
        gmsh.model.occ.importShapes(str(SOURCE))
        gmsh.model.occ.synchronize()
        faces = []
        for solid, face in [(11, 338), (26, 566), (112, 4218)]:
            boundary = {abs(t) for d, t in gmsh.model.getBoundary([(3, solid)])}
            if face not in boundary or gmsh.model.getType(2, face) != 'Plane':
                raise ValueError('Reference face topology changed')
            faces.append({
                'solid_tag': solid, 'solid_name': gmsh.model.getEntityName(3, solid),
                'face_tag': face,
                'area_mm2': gmsh.model.occ.getMass(2, face),
                'area_centroid_cad_mm': list(gmsh.model.occ.getCenterOfMass(2, face)),
                'bounds_cad_mm': list(gmsh.model.getBoundingBox(2, face)),
            })
        # Circular trim on selected rear annular face: circumcircle center is
        # the mounting axis; area centroid of a perforated plate is NOT its axis.
        edge = 10117
        if edge not in {abs(t) for d, t in gmsh.model.getBoundary([(2, 4218)])}:
            raise ValueError('Selected mounting circle is not on reference face')
        lo, hi = gmsh.model.getParametrizationBounds(1, edge)
        params = np.linspace(float(lo[0]), float(hi[0]), 4, endpoint=False)
        circle = np.asarray(gmsh.model.getValue(1, edge, params)).reshape(-1, 3)
        origin = circle.mean(axis=0)
        radii = np.linalg.norm(circle-origin, axis=1)
        np.testing.assert_allclose(radii, 35.3, atol=1e-6)
        np.testing.assert_allclose(origin[0], faces[2]['area_centroid_cad_mm'][0], atol=1e-6)
        left, right = [np.array(f['area_centroid_cad_mm']) for f in faces[:2]]
        point = (left + right) / 2
        # Right-handed CONSTRUCTED mounting datum, not registered UR axes:
        # x_D = +y_CAD, y_D = +z_CAD, z_D = +x_CAD.
        rotation = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
        np.testing.assert_allclose(np.linalg.det(rotation), 1.)
        datum_from_point = rotation.T @ (point-origin)
        cad_from_datum = np.eye(4)
        cad_from_datum[:3, :3] = rotation
        cad_from_datum[:3, 3] = origin
        np.testing.assert_allclose(
            (cad_from_datum @ np.r_[datum_from_point, 1])[:3], point, atol=1e-9)
        result = {
            'kind': 'cad_nominal_reference_not_controller_tcp',
            'basis_decision': 'User requested CAD-derived TCP as nominal reference',
            'source_path': str(SOURCE), 'source_sha256': digest, 'length_unit': 'mm',
            'point_definition': 'Midpoint of area centroids of paired flat pad faces 338 and 566',
            'reference_faces': faces, 'mount_axis_circle_edge': edge,
            'mount_axis_circle_radius_mm': float(radii.mean()),
            'selected_datum_definition': 'Axis center on rear annular plane face 4218 of part 101164',
            'selected_datum_is_verified_ur_mating_plane': False,
            'datum_origin_cad_mm': origin.tolist(),
            'datum_axes_in_cad': {'x': [0, 1, 0], 'y': [0, 0, 1], 'z': [1, 0, 0]},
            'cad_from_selected_datum_mm': cad_from_datum.tolist(),
            'nominal_point_cad_mm': point.tolist(),
            'nominal_point_selected_datum_mm': datum_from_point.tolist(),
            'paired_plane_gap_mm': abs(float(left[1]-right[1])),
            'opening_state': 'As supplied in STEP; not closed or sponge holding state',
            'datum_sensitivity': {
                'alternative': 'If mating plane is rear tabs at CAD X=51.8297556732 mm',
                'alternative_point_selected_datum_mm': (datum_from_point+[0, 0, 2]).tolist(),
                'axial_difference_mm': 2.,
            },
            'rotation_convention': 'Point only; no installed UR tool orientation inferred',
            'robot_flange_from_cad': None, 'controller_tcp_pose': None,
            'tcp_verified': False, 'robot_execution_enabled': False,
            'notes': [
                'Keep CAD geometry as nominal basis per user instruction; no further ruler verification requested.',
                'Selected datum is reproducible but actual robot adapter stack and flange axis registration remain unknown.',
                'Applies to supplied standard pad geometry and opening only; no unmodeled extensions included.',
                'No robot connection, motion, I/O write or controller setting change performed.',
            ],
        }
        OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
        print('Nominal point in SELECTED CAD datum (mm):', datum_from_point.round(6).tolist())
        print('Paired plane gap (mm):', round(result['paired_plane_gap_mm'], 6))
        print('Saved:', OUTPUT)
    finally:
        gmsh.finalize()


if __name__ == '__main__':
    main()
