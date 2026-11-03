"""Work cell: where objects are picked, where each category is dropped, and how to grasp.

This is the meeting point with the other bricks:
  * vision/category -> `make_request(...)` builds a PickRequest (class, object xy, yaw);
  * the sorting policy (configs/sorting_policy.json) maps class -> category -> drop zone;
  * `task_poses(...)` turns a request into the poses the pick-and-place machine will visit.
All poses are UR base frame [x, y, z, rx, ry, rz]; grasps are top-down (object weight is
negligible, so no payload or slow-transport handling).
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .contracts import validate_joints, validate_pose
from .geometry import top_down_rotvec

CELL_PATH = Path(__file__).resolve().parents[1] / 'configs' / 'sim_cell' / 'cell_config.json'


@dataclass(frozen=True)
class PickRequest:
    """One object to sort. object_base_xyz_m: object centre in the UR base frame (z is
    informative: the grasp height comes from the pick-zone surface + per-class height)."""
    task_id: str
    class_name: str
    target_zone: str
    object_base_xyz_m: tuple
    object_yaw_rad: float = 0.0
    source: str = ''


@dataclass(frozen=True)
class TaskPoses:
    pre_grasp: list
    grasp: list
    drop_approach: list
    release: list
    held_object_length_m: float


def load_cell(path=CELL_PATH):
    cell = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_cell(cell)
    return cell


def _finite(v, positive=False):
    return type(v) in (int, float) and math.isfinite(v) and (v > 0 if positive else True)


def _grasp_ok(g):
    return (isinstance(g, dict) and _finite(g.get('grasp_height_m')) and g['grasp_height_m'] >= 0
            and _finite(g.get('held_object_length_m')) and g['held_object_length_m'] >= 0)


def validate_cell(cell):
    if cell.get('schema_version') != 1 or cell.get('frame') != 'UR_BASE':
        raise ValueError('Unsupported cell schema or frame')
    observe = cell.get('observe')
    if not isinstance(observe, dict):
        raise ValueError('observe section required (joints may be null)')
    if observe.get('joints') is not None:
        validate_joints(observe['joints'])
    zone = cell.get('pick_zone', {})
    poly = zone.get('polygon_xy_m')
    if (not isinstance(poly, list) or len(poly) < 3
            or not all(isinstance(p, list) and len(p) == 2 and all(_finite(v) for v in p) for p in poly)):
        raise ValueError('pick_zone.polygon_xy_m needs at least 3 [x, y] points')
    if not _finite(zone.get('surface_z_m')):
        raise ValueError('pick_zone.surface_z_m must be finite')
    if not _finite(cell.get('approach_height_m'), positive=True) or not _finite(cell.get('tool_yaw_deg')):
        raise ValueError('approach_height_m must be positive and tool_yaw_deg finite')
    grasp = cell.get('grasp', {})
    if not _grasp_ok(grasp.get('default')) or not isinstance(grasp.get('classes'), dict):
        raise ValueError('grasp.default and grasp.classes required')
    if not all(_grasp_ok(g) for g in grasp['classes'].values()):
        raise ValueError('Each grasp class needs grasp_height_m and held_object_length_m >= 0')
    drops = cell.get('drop_zones')
    if not isinstance(drops, dict) or not drops:
        raise ValueError('drop_zones required')
    for name, d in drops.items():
        if not isinstance(d, dict) or not _finite(d.get('approach_height_m'), positive=True):
            raise ValueError(f'{name}: approach_height_m must be positive')
        validate_pose(d.get('release_pose'))
    if not _finite(cell.get('gripper', {}).get('settle_s')) or cell['gripper']['settle_s'] < 0:
        raise ValueError('gripper.settle_s must be >= 0')


def unmeasured_entries(cell):
    """Names of cell entries not yet taught on the robot."""
    out = [k for k in ('observe', 'pick_zone') if cell[k].get('measured') is not True]
    out += [f'drop_zones.{k}' for k, d in cell['drop_zones'].items() if d.get('measured') is not True]
    return out


def point_in_polygon(x, y, poly):
    """Even-odd rule; points on an edge count as outside (conservative)."""
    inside = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside


def make_request(task_id, class_name, object_xyz, policy, *, yaw_rad=0.0, source=''):
    """Category brick -> request: the drop zone comes from the sorting policy."""
    category = policy['class_to_category'].get(class_name)
    if category is None:
        raise ValueError(f'Unknown class {class_name!r} in the sorting policy')
    xyz = tuple(float(v) for v in object_xyz)
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz) or not _finite(yaw_rad):
        raise ValueError('object_xyz must be 3 finite numbers and yaw finite')
    return PickRequest(task_id, class_name, policy['category_to_zone'][category], xyz, float(yaw_rad), source)


def task_poses(cell, request):
    """Poses for one pick-and-place. Raises ValueError if the object/zone is not allowed."""
    x, y = request.object_base_xyz_m[:2]
    zone = cell['pick_zone']
    if not point_in_polygon(x, y, zone['polygon_xy_m']):
        raise ValueError(f'Object at ({x:.3f}, {y:.3f}) is outside the pick zone')
    drop = cell['drop_zones'].get(request.target_zone)
    if drop is None:
        raise ValueError(f'No drop zone configured for {request.target_zone!r}')
    grasp_cfg = cell['grasp']['classes'].get(request.class_name, cell['grasp']['default'])
    rotvec = top_down_rotvec(math.radians(cell['tool_yaw_deg']) + request.object_yaw_rad)
    grasp_z = zone['surface_z_m'] + grasp_cfg['grasp_height_m']
    grasp = [x, y, grasp_z] + rotvec
    pre_grasp = [x, y, grasp_z + cell['approach_height_m']] + rotvec
    release = list(drop['release_pose'])
    drop_approach = release[:2] + [release[2] + drop['approach_height_m']] + release[3:]
    return TaskPoses(pre_grasp, grasp, drop_approach, release, grasp_cfg['held_object_length_m'])
