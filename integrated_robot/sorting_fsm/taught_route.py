"""Pick-and-place from the team's taught joint references (option A), ready for measured positions (B).

Data: data/planning_inputs_four_objects_tcp_zero/ (2026-10-26, 24 manual references, 6 per class:
pick_approach, pick_grasp, pick_lift, drop_approach, drop_release, drop_retreat; controller TCP = 0
while recording; one fixed object placement per class). `load_taught_catalog` re-checks the package
exactly like its own verify_inputs.py (hash of every evidence file and of the catalog, per-class
files equal to the catalog, joints equal to the source snapshots, zero TCP) and that its routes
match our sorting policy. Any difference refuses the whole catalog.

Option A, fixed placement (max_pick_offset_m = 0): the object is put by hand on the taught pick
spot and every move ends exactly on a taught posture (joint goal = taught joints, executed as
moveJ, like a pendant program replaying its waypoints). The transfers are planned freely
(direct or RRT-Connect); the short descend/lift/release/retreat moves must be one direct
joint segment whose TCP and tool tip stay within max_path_deviation_m of a straight line
(measured on the 24 references: <= 8 mm). Why not moveL: several references are close to the
elbow singularity (bottle drop: elbow 4 deg, arm almost straight) where a Cartesian line makes the
joint speeds explode; moveJ has no singularity and ends on the taught joints whatever our
model's ~2 mm calibration error.

Option B, measured position (max_pick_offset_m > 0, needs vision + a validated camera -> base
transform): the pick approach/grasp/lift are the taught poses translated horizontally by
(object xy - taught grasp xy), same orientation and heights, reached with the taught arm
configuration (IK nearest to the taught joints) and moveL for descend/lift. The planner refuses
a moveL that passes near a singularity (then teach that class away from it). Drop unchanged.

Grasp overrides (cell taught_route.grasp_overrides, sorting_fsm/grasp_override.py): a class's pick
approach/grasp/lift can be replaced by a grasp taught by hand with tools/teach_grasp.py (e.g. a
tilted sponge grasp); its drop part is unchanged and the team's package is never edited.

Route aliases (cell taught_route.route_aliases, e.g. {"aluminum_can": "plastic_bottle"}): a class
uses the WHOLE route (pick and drop) of another taught class with the same target zone, e.g. when
both objects are put on the same spot. None of the aliased class's own references are used, so its
incident review does not apply; the target class must not need one itself.

Classes without taught references (toothbrush) have no route: never another class's plan.
"""
from dataclasses import dataclass, replace
import hashlib
import json
import math
from pathlib import Path

from .cell import TaskPoses, point_in_polygon
from .contracts import Action, validate_joints, validate_pose
from .grasp_override import apply_grasp_overrides
from .picking import picking_steps
from .steps import Step, default_pick_strategy
from .ur5_kinematics import matmul, matrix_to_pose, pose_to_matrix

WAYPOINTS = ('pick_approach', 'pick_grasp', 'pick_lift', 'drop_approach', 'drop_release', 'drop_retreat')


@dataclass(frozen=True)
class TaughtSet:
    object_class: str
    target_zone: str
    joints: dict          # waypoint -> 6 joint angles (rad), controller order, unwrapped
    flange_poses: dict    # waypoint -> controller-reported pose with TCP = 0 (flange), UR base
    incident_review_required: bool
    incident_evidence: list
    source_sha256: dict   # waypoint -> evidence file hash


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def load_taught_catalog(folder, policy):
    """{class: TaughtSet} from a verified planning-input package. Raises ValueError."""
    root = Path(folder).resolve()
    mappings = json.loads((root / 'portable_sources.json').read_text(encoding='utf-8'))
    records, hashes = {}, {}
    for item in mappings['sources']:
        source = (root / item['relative_path']).resolve()
        if not source.is_relative_to(root):
            raise ValueError('Evidence path outside the package')
        raw = source.read_bytes()
        if _sha(raw) != item['sha256']:
            raise ValueError(f"Changed evidence file: {item['relative_path']}")
        records[item['original_source_path']] = json.loads(raw)
        hashes[item['original_source_path']] = item['sha256']
    catalog_raw = (root / 'catalog.json').read_bytes()
    if _sha(catalog_raw) != mappings['catalog_sha256']:
        raise ValueError('Changed catalog.json')
    catalog = json.loads(catalog_raw)
    if catalog.get('type') != 'audited_tcp_zero_joint_reference_catalog' or catalog.get('trajectory') is not None:
        raise ValueError('Unexpected catalog type')
    sets = {}
    for obj in catalog['objects']:
        name = obj['object_class']
        exported = json.loads((root / f'{name}_requests.json').read_text(encoding='utf-8'))
        if exported != obj:
            raise ValueError(f'{name}_requests.json differs from the catalog')
        category = policy['class_to_category'].get(name)
        if category is None or policy['category_to_zone'][category] != obj['target_zone']:
            raise ValueError(f'{name}: catalog route {obj["target_zone"]} differs from the sorting policy')
        if name in sets or set(obj['waypoints']) != set(WAYPOINTS):
            raise ValueError(f'{name}: duplicate class or wrong waypoint set')
        joints, poses, shas = {}, {}, {}
        for waypoint in WAYPOINTS:
            ref = obj['waypoints'][waypoint]
            record = records.get(ref['source_path'])
            if record is None or hashes[ref['source_path']] != ref['source_sha256']:
                raise ValueError(f'{name}.{waypoint}: evidence missing or hash differs')
            if record['actual_q_rad'] != ref['joint_positions_rad']:
                raise ValueError(f'{name}.{waypoint}: joints differ from the source snapshot')
            if (record['configured_tcp_offset_flange_m_rotvec_rad'] != [0] * 6
                    or ref['recorded_tcp_offset_flange_m_rotvec_rad'] != [0] * 6):
                raise ValueError(f'{name}.{waypoint}: only zero-TCP references are supported')
            joints[waypoint] = validate_joints(ref['joint_positions_rad'])
            poses[waypoint] = validate_pose(record['configured_tcp_pose_base_m_rotvec_rad'])
            shas[waypoint] = ref['source_sha256']
        sets[name] = TaughtSet(name, obj['target_zone'], joints, poses,
                               bool(obj.get('incident_review_required')),
                               list(obj.get('incident_evidence', [])), shas)
    return sets


def _with_tcp(flange_pose, tcp_pose):
    return matrix_to_pose(matmul(pose_to_matrix(flange_pose), pose_to_matrix(tcp_pose)))


def _shift(pose, dx, dy):
    return [pose[0] + dx, pose[1] + dy] + list(pose[2:])


def apply_route_aliases(taught, aliases):
    """Copy of the taught sets where each aliased class takes the whole route of its target class."""
    out = dict(taught)
    for class_name, target in (aliases or {}).items():
        for name in (class_name, target):
            if name not in taught:
                raise ValueError(f'route alias {class_name!r} -> {target!r}: no taught references for {name!r}')
        if target in (aliases or {}):
            raise ValueError(f'route alias {class_name!r} -> {target!r}: no alias chain allowed')
        source = taught[target]
        if source.target_zone != taught[class_name].target_zone:
            raise ValueError(f'route alias {class_name!r} -> {target!r}: different target zone '
                             f'({taught[class_name].target_zone} vs {source.target_zone})')
        if source.incident_review_required:
            raise ValueError(f'route alias {class_name!r} -> {target!r}: {target} needs an incident review')
        out[class_name] = replace(source, object_class=class_name)
    return out


class TaughtRoute:
    """route for PickPlaceMachine: request -> Steps through the taught references.

    tcp_pose: the controller TCP (motion_config tcp_pose); targets are the taught flange poses
    composed with it, so the references stay valid whatever TCP is set on the pendant.
    """
    def __init__(self, taught, cell, tcp_pose, *, pick_strategy=default_pick_strategy,
                 max_pick_offset_m=0.0, max_path_deviation_m=0.01, incident_reviewed=()):
        if type(max_pick_offset_m) not in (int, float) or not 0 <= max_pick_offset_m <= 0.2:
            raise ValueError('max_pick_offset_m must be in [0, 0.2] m')
        if type(max_path_deviation_m) not in (int, float) or not 0 < max_path_deviation_m <= 0.05:
            raise ValueError('max_path_deviation_m must be in (0, 0.05] m')
        self.taught, self.cell, self.tcp_pose = taught, cell, validate_pose(tcp_pose)
        self.pick_strategy, self.max_offset = pick_strategy, float(max_pick_offset_m)
        self.max_deviation = float(max_path_deviation_m)
        self.incident_reviewed = frozenset(incident_reviewed)

    @classmethod
    def from_cell(cls, cell, policy, tcp_pose, *, root, pick_strategy=default_pick_strategy):
        cfg = cell['taught_route']
        taught = load_taught_catalog(Path(root) / cfg['catalog_dir'], policy)
        taught = apply_grasp_overrides(taught, cfg.get('grasp_overrides', {}), root=root)
        taught = apply_route_aliases(taught, cfg.get('route_aliases', {}))
        return cls(taught, cell, tcp_pose, pick_strategy=pick_strategy,
                   max_pick_offset_m=cfg['max_pick_offset_m'],
                   max_path_deviation_m=cfg['max_path_deviation_m'],
                   incident_reviewed=cfg['incident_reviewed'])

    def classes(self):
        return sorted(self.taught)

    def taught_set(self, class_name):
        taught = self.taught.get(class_name)
        if taught is None:
            raise ValueError(f'No taught references for {class_name!r}; never reusing another class plan')
        if taught.incident_review_required and class_name not in self.incident_reviewed:
            raise ValueError(f'{class_name}: a protective stop happened while teaching; review its cause on '
                             'site, then add the class to taught_route.incident_reviewed')
        return taught

    def grasp_position(self, class_name):
        """TCP position at the taught grasp: where a fixed-placement object is (option A)."""
        return _with_tcp(self.taught_set(class_name).flange_poses['pick_grasp'], self.tcp_pose)[:3]

    def pick_offset(self, request):
        taught = self.taught_set(request.class_name)
        if request.target_zone != taught.target_zone:
            raise ValueError(f'Request zone {request.target_zone} differs from the taught {taught.target_zone}')
        gx, gy, _ = self.grasp_position(request.class_name)
        dx, dy = request.object_base_xyz_m[0] - gx, request.object_base_xyz_m[1] - gy
        offset = math.hypot(dx, dy)
        if offset > self.max_offset + 1e-6:
            raise ValueError(f'Object is {offset * 1000:.0f} mm from the taught pick spot; allowed '
                             f'{self.max_offset * 1000:.0f} mm (fixed placement unless measured positions are enabled)')
        if offset > 1e-6 and not point_in_polygon(*request.object_base_xyz_m[:2],
                                                  self.cell['pick_zone']['polygon_xy_m']):
            raise ValueError('Measured object position is outside the pick zone')
        return (0.0, 0.0) if offset <= 1e-6 else (dx, dy)

    def poses(self, request):
        """TaskPoses (for pick strategies) of this request, with the TCP applied."""
        taught = self.taught_set(request.class_name)
        dx, dy = self.pick_offset(request)
        pose = {w: _with_tcp(taught.flange_poses[w], self.tcp_pose) for w in WAYPOINTS}
        grasp_cfg = self.cell['grasp']['classes'].get(request.class_name, self.cell['grasp']['default'])
        return TaskPoses(_shift(pose['pick_approach'], dx, dy), _shift(pose['pick_grasp'], dx, dy),
                         pose['drop_approach'], pose['drop_release'], grasp_cfg['held_object_length_m'])

    def steps(self, request):
        taught = self.taught_set(request.class_name)
        dx, dy = self.pick_offset(request)
        fixed = (dx, dy) == (0.0, 0.0)
        q = taught.joints
        pose = {w: _with_tcp(taught.flange_poses[w], self.tcp_pose) for w in WAYPOINTS}
        poses = self.poses(request)
        grip = self.pick_strategy(request, poses)
        held, straight = poses.held_object_length_m, self.max_deviation
        if fixed:
            reach = Step('TRANSFER_TO_PICK', 'joint_q', q['pick_approach'], phase='reaching')
            pick = picking_steps(q['pick_grasp'], q['pick_lift'], kind='joint_q', grip_steps=grip,
                                 held_object_m=held, max_path_deviation_m=straight)
        else:
            reach = Step('TRANSFER_TO_PICK', 'joint_pose', poses.pre_grasp,
                         reference_joints=q['pick_approach'], phase='reaching')
            pick = picking_steps(poses.grasp, _shift(pose['pick_lift'], dx, dy), kind='linear', grip_steps=grip)
        steps = [reach, *pick,
                 Step('TRANSFER_TO_DROP', 'joint_q', q['drop_approach'], held_object_m=held, phase='transport'),
                 Step('DESCEND_TO_RELEASE', 'joint_q', q['drop_release'], held_object_m=held,
                      max_path_deviation_m=straight, phase='placing'),
                 Step('RELEASE', 'gripper', action=Action.OPEN, phase='placing'),
                 Step('RETREAT', 'joint_q', q['drop_retreat'], max_path_deviation_m=straight, phase='placing')]
        if self.cell['observe'].get('joints') is not None:
            steps.append(Step('RETURN_TO_OBSERVE', 'joint_q', list(self.cell['observe']['joints']),
                              phase='return'))
        return steps
