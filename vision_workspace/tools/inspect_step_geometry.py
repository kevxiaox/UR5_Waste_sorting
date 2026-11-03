"""Offline STEP geometry rendering. CAD axes are NOT robot flange axes."""
import hashlib
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tmp/cad_tools'))
import gmsh
import cv2
import numpy as np

SOURCES = [
    Path(r'C:\Users\KEVXI\Downloads\2F85-CLOSED_20181108-Sep-06-2024-02-25-56-3542-PM.step'),
    Path(r'C:\Users\KEVXI\Downloads\2F85_Opened_20190924-Sep-06-2024-02-25-46-4707-PM.step'),
]


def render(vertices, segments, title, output):
    canvas = np.full((1000, 1200, 3), 245, np.uint8)
    center = (vertices.min(axis=0)+vertices.max(axis=0))/2
    vertices = vertices-center
    views = [('CAD XY', np.eye(3)),
             ('CAD XZ', np.array([[1,0,0],[0,0,1],[0,-1,0]])),
             ('CAD YZ', np.array([[0,1,0],[0,0,1],[1,0,0]])),
             ('ISOMETRIC', np.array([[.707,-.707,0],[.408,.408,.816],[-.577,-.577,.577]]))]
    for index, (name, rotation) in enumerate(views):
        transformed = vertices@rotation.T
        scale = min(500/np.ptp(transformed[:,0]), 360/np.ptp(transformed[:,1]))
        x0, y0 = (index % 2)*600, (index//2)*480+40
        pixel = np.column_stack((transformed[:,0]*scale+x0+300,
                                 -transformed[:,1]*scale+y0+230)).round().astype(np.int32)
        edges = transformed[segments]
        order = np.argsort(edges[:,:,2].mean(axis=1))
        for i in order:
            a,b = pixel[segments[i]]
            cv2.line(canvas,tuple(a),tuple(b),(100,85,65),1,cv2.LINE_AA)
        cv2.putText(canvas,name,(x0+20,y0+30),cv2.FONT_HERSHEY_SIMPLEX,.7,(20,20,20),2)
    cv2.putText(canvas,title,(15,25),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,0),1)
    cv2.putText(canvas,'CAD origin/axes only. No flange registration or TCP validation.',(15,980),cv2.FONT_HERSHEY_SIMPLEX,.6,(0,0,180),1)
    if not cv2.imwrite(str(output),canvas):
        raise OSError('Failed to save rendered views')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,action='append')
    parser.add_argument('--output',type=Path,default=ROOT/'data/cad_review/2f85_20261023')
    parser.add_argument('--label',default='2F-85')
    args=parser.parse_args()
    output = args.output
    output.mkdir(parents=True,exist_ok=True)
    summaries = []
    gmsh.initialize()
    gmsh.option.setNumber('General.Terminal',0)
    try:
        for index, source in enumerate(args.source or SOURCES):
            gmsh.clear()
            gmsh.model.add('inspection')
            gmsh.model.occ.importShapes(str(source))
            gmsh.model.occ.synchronize()
            bounds = list(gmsh.model.getBoundingBox(-1,-1))
            print('Imported:',source.name,'CAD bounds:',bounds,flush=True)
            # Evaluate CAD edges directly: no mesh healing or source geometry modification.
            vertex_parts, segment_parts, count = [], [], 0
            for _, tag in gmsh.model.getEntities(1):
                low, high = gmsh.model.getParametrizationBounds(1, tag)
                samples = 2 if gmsh.model.getType(1,tag)=='Line' else 24
                parameters = np.linspace(float(low[0]),float(high[0]),samples)
                points = np.asarray(gmsh.model.getValue(1,tag,parameters)).reshape(-1,3)
                vertex_parts.append(points)
                segment_parts.append(np.column_stack((np.arange(count,count+samples-1),
                                                      np.arange(count+1,count+samples))))
                count += samples
            vertices = np.vstack(vertex_parts)
            segments = np.vstack(segment_parts)
            state = source.stem if args.source else ('closed' if index==0 else 'opened')
            picture = output/(state+'_views.png')
            render(vertices,segments,args.label+' '+state+' (supplied STEP edges)',picture)
            summaries.append({'source_path':str(source),'source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
                              'file_length_units':'millimetres per STEP SI_UNIT metadata',
                              'state':state,'cad_bounds_xyz_minmax':bounds,
                              'cad_axis_extent_mm':(np.array(bounds[3:])-bounds[:3]).tolist(),
                              'solid_count':len(gmsh.model.getEntities(3)),
                              'solids':[{'tag':tag,'name':gmsh.model.getEntityName(3,tag),
                                         'cad_bounds_xyz_minmax':list(gmsh.model.getBoundingBox(3,tag)),
                                         'cad_center_of_mass_xyz':list(gmsh.model.occ.getCenterOfMass(3,tag))}
                                        for _,tag in gmsh.model.getEntities(3)],
                              'display_edge_segment_count':len(segments),'render_path':str(picture),
                              'display_method':'sampled original CAD edges, no source healing',
                              'robot_flange_registration_known':False,'tcp_computed':False})
            print('Rendered:',picture,flush=True)
    finally:
        gmsh.finalize()
    (output/'geometry_inventory.json').write_text(json.dumps({'models':summaries,
        'note':'Geometric extents are CAD-axis bounds of supplied assemblies, not installed flange-to-grasp offsets. '
               'Physical gripper match and missing/custom attachment geometry require confirmation.',
        'robot_execution_enabled':False},ensure_ascii=False,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    main()
