"""Offline interface hypotheses with exact OCC intersection checks, no robot API."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tmp/cad_tools'))
import gmsh


def main():
    rg_dir = ROOT/'data/cad_review/rg6_20261023'
    qc_dir = ROOT/'data/cad_review/quick_changer_20261023'
    rg = json.loads((rg_dir/'interface_features.json').read_text())
    qc = json.loads((qc_dir/'interface_features.json').read_text())
    nominal = json.loads((rg_dir/'cad_nominal_tcp.json').read_text())
    for source in (rg, qc):
        if hashlib.sha256(Path(source['source_path']).read_bytes()).hexdigest() != source['source_sha256']:
            raise ValueError('Source hash changed')
    rfaces = {f['face']: f for f in rg['planes']}
    qfaces = {f['face']: f for f in qc['planes']}
    plane_x = rfaces[3785]['center_mm'][0]
    np.testing.assert_allclose(plane_x, rfaces[3797]['center_mm'][0], atol=1e-7)
    plane_y = qfaces[54]['center_mm'][1]
    np.testing.assert_allclose(plane_y, qfaces[55]['center_mm'][1], atol=1e-7)
    axis_yz = np.array(nominal['datum_origin_cad_mm'])[1:]
    origin = np.r_[plane_x, axis_yz]
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal', 0)
    candidates = []
    try:
        for angle in (0, 180):
            gmsh.clear()
            gmsh.model.occ.importShapes(rg['source_path'])
            gmsh.model.occ.synchronize()
            rg_solids = gmsh.model.getEntities(3)
            imported = gmsh.model.occ.importShapes(qc['source_path'])
            gmsh.model.occ.synchronize()
            qc_solids = [dt for dt in imported if dt[0] == 3]
            # QC tool-facing direction -Y points towards RG6 +X.
            rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
            if angle:
                rotation = np.diag([1., -1., -1.]) @ rotation
            translation = origin - rotation @ [0., plane_y, 0.]
            transform = np.eye(4)
            transform[:3, :3] = rotation
            transform[:3, 3] = translation
            gmsh.model.occ.affineTransform(qc_solids, transform.ravel().tolist())
            gmsh.model.occ.synchronize()
            intersections = []
            # Compare all solids whose exact bounding boxes intersect. The source
            # files remain unchanged; boolean results exist in memory only.
            for qdim, qtag in qc_solids:
                qb = np.array(gmsh.model.getBoundingBox(3, qtag))
                for rdim, rtag in rg_solids:
                    rb = np.array(gmsh.model.getBoundingBox(3, rtag))
                    if np.any(np.minimum(qb[3:], rb[3:])-np.maximum(qb[:3], rb[:3]) <= 1e-5):
                        continue
                    result, _ = gmsh.model.occ.intersect(
                        [(3, qtag)], [(3, rtag)], removeObject=False, removeTool=False)
                    volume = sum(gmsh.model.occ.getMass(dim, tag) for dim, tag in result if dim == 3)
                    if volume > .001:
                        intersections.append({'quick_changer_solid': qtag,
                                              'rg6_solid': rtag, 'overlap_mm3': volume})
                    if result:
                        gmsh.model.occ.remove(result, recursive=True)
            # Frame D follows previous axes, but its origin is this candidate's
            # robot mounting plane center QC(0,0,0), not the old RG6 ring datum.
            basis = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
            point = np.array(nominal['nominal_point_cad_mm'])
            candidate = {
                'hypothesis': 'rear paired RG6 pads mate with QC tool-facing pads',
                'rotation_about_mount_axis_deg': angle,
                'T_rg6_cad_from_qc_cad_mm': transform.tolist(),
                'candidate_robot_mount_origin_rg6_cad_mm': translation.tolist(),
                'candidate_point_datum_D_mm': (basis.T@(point-translation)).tolist(),
                'overlap_mm3': sum(item['overlap_mm3'] for item in intersections),
                'solid_intersections': intersections,
                'status': 'unvalidated_assembly_hypothesis',
            }
            candidates.append(candidate)
            print('Angle:', angle, 'overlap mm3:', round(candidate['overlap_mm3'], 6),
                  'point in candidate D mm:', candidate['candidate_point_datum_D_mm'], flush=True)
    finally:
        gmsh.finalize()
    report = {
        'kind': 'offline_mount_candidate_screening',
        'source_sha256': {'rg6': rg['source_sha256'], 'quick_changer': qc['source_sha256']},
        'rg6_candidate_contact_faces': [3785, 3797],
        'quick_changer_candidate_contact_faces': [54, 55],
        'rg6_candidate_contact_plane_cad_x_mm': plane_x,
        'qc_candidate_contact_plane_cad_y_mm': plane_y,
        'candidates': candidates,
        'interpretation': 'Positive intersection rejects ideal rigid assembly at these CAD states. Zero intersection alone does not prove mating, revision compatibility or installed mounting orientation.',
        'robot_flange_registration_known': False, 'controller_tcp_pose': None,
        'tcp_verified': False, 'robot_execution_enabled': False,
    }
    target = qc_dir/'assembly_candidates.json'
    target.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('Saved:', target)


if __name__ == '__main__':
    main()
