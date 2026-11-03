"""Grasp overrides: a pick taught by hand for one class, replacing that class's taught pick.

Why: the team's references grasp every object with the tool almost horizontal. For flat objects
(sponge) a tilted grasp (~30-40 deg below horizontal) reaches over the object; it stays in the same
arm configuration up to ~40 deg (beyond, the arm would have to flip; vertical is out of reach).

Only the GRASP posture is taught (arm placed in freedrive, tools/teach_grasp.py). The approach and
the lift are derived from it, because a hand-made straight retreat is not possible:
  * pick_approach: the grasp pose moved back by back_m along the tool axis (the descent is then a
    straight move along the tilted tool axis);
  * pick_lift: the grasp pose moved up by lift_m (vertical), same orientation; if that is out of
    reach in the same arm configuration (tilted grasps: the flange is already high, the arm would
    have to stretch), the grasp pose moved back by lift_m along the tool axis instead (the way the
    gripper came in), which must still raise the object by MIN_LIFT_RISE_M.
Both are solved with the closed-form IK, nearest to the taught grasp joints (same configuration),
and must pass the same checks as the team's references: joint limits, collisions, and a tool path
within max_deviation_m of a straight line (the moves are executed as moveJ).

File (data/grasp_overrides/<class>.json), enabled per class in cell_config
taught_route.grasp_overrides = {"sponge": "data/grasp_overrides/sponge.json"}. The drop part of the
class (drop_approach, drop_release, drop_retreat) is unchanged. The team's package is never edited.
"""
from dataclasses import replace
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path

from .contracts import validate_joints, validate_pose
from .geometry import rotvec_to_matrix, tool_z_axis
from .ur5_kinematics import forward

PICK_WAYPOINTS = ('pick_approach', 'pick_grasp', 'pick_lift')
ZERO_TCP = [0.0] * 6
MODEL_TOLERANCE_M = 0.005        # our model vs the controller (measured ~2 mm on this robot)
MODEL_TOLERANCE_RAD = 0.03
FILE_TOLERANCE_M = 0.001         # stored joints vs stored poses (same model, rounding only)
SAME_CONFIGURATION_RAD = 1.0     # derived postures must stay close to the taught grasp joints
MIN_LIFT_RISE_M = 0.03           # a lift along the tool axis must raise the object at least this much


def tool_tilt_deg(flange_pose):
    """Angle of the tool axis below the horizontal plane (deg; 90 = pointing straight down)."""
    axis = tool_z_axis(flange_pose[3:])
    return math.degrees(math.asin(max(-1.0, min(1.0, -axis[2]))))


def _orientation_error(pose_a, pose_b):
    ra, rb = rotvec_to_matrix(pose_a[3:]), rotvec_to_matrix(pose_b[3:])
    trace = sum(ra[k][i] * rb[k][i] for i in range(3) for k in range(3))
    return math.acos(max(-1.0, min(1.0, (trace - 1) / 2)))


def _pose_error(pose_a, pose_b):
    return math.dist(pose_a[:3], pose_b[:3]), _orientation_error(pose_a, pose_b)


def derive_pick(planner, grasp_joints, controller_pose, *, back_m, lift_m, max_deviation_m, held_object_m):
    """(result, problems) for a grasp posture read from the robot with TCP = 0 on the pendant.

    result: {'joints': {waypoint: q}, 'flange_poses': {waypoint: pose}, 'deviation_m': {...},
    'tool_tilt_deg': float} (None when the grasp itself is unusable). problems: [] when every
    check passed; otherwise human-readable reasons, nothing should be saved."""
    if not 0.02 <= back_m <= 0.20 or not 0.02 <= lift_m <= 0.20:
        raise ValueError('back_m and lift_m must be within [0.02, 0.20] m')
    q = validate_joints(grasp_joints)
    controller_pose = validate_pose(controller_pose)
    problems = []
    grasp = forward(q, ZERO_TCP)
    position_error, orientation_error = _pose_error(grasp, controller_pose)
    if position_error > MODEL_TOLERANCE_M or orientation_error > MODEL_TOLERANCE_RAD:
        problems.append(f'controller pose differs from our model by {position_error * 1000:.0f} mm / '
                        f'{orientation_error:.3f} rad: set the pendant TCP to 0 (Installation > TCP)')
    state = planner.state_problem(q)
    if state is not None:
        problems.append(f'grasp posture refused by the model: {state}')
        return None, problems
    axis = tool_z_axis(grasp[3:])
    back = lambda d: [p - d * a for p, a in zip(grasp[:3], axis)] + grasp[3:]   # noqa: E731
    joints, poses = {'pick_grasp': q}, {'pick_grasp': grasp}

    def solve(target):
        """(joints, None) in the grasp's arm configuration, or (None, reason)."""
        candidates, rejected = planner.goal_candidates(target, q)
        if not candidates:
            return None, '; '.join(sorted(set(rejected))) or 'out of reach'
        jump = max(abs(a - b) for a, b in zip(candidates[0], q))
        if jump > SAME_CONFIGURATION_RAD:
            return None, f'only reachable by changing arm configuration (joint jump {math.degrees(jump):.0f} deg)'
        return candidates[0], None

    approach, reason = solve(back(back_m))
    if approach is None:
        problems.append(f'pick_approach: {reason}; tilt the gripper less')
    else:
        joints['pick_approach'], poses['pick_approach'] = approach, forward(approach, ZERO_TCP)
    lift, reason = solve([grasp[0], grasp[1], grasp[2] + lift_m] + grasp[3:])
    lift_mode = 'vertical'
    if lift is None:
        lift_mode = 'tool_axis'
        if -axis[2] * lift_m < MIN_LIFT_RISE_M:
            problems.append(f'pick_lift: vertical lift impossible ({reason}) and the tool is too '
                            'horizontal to lift along its axis')
        else:
            lift, reason_axis = solve(back(lift_m))
            if lift is None:
                problems.append(f'pick_lift: vertical ({reason}) and along the tool axis ({reason_axis}) impossible')
    if lift is not None:
        joints['pick_lift'], poses['pick_lift'] = lift, forward(lift, ZERO_TCP)
    deviation = {}
    if 'pick_approach' in joints:
        deviation['descend'] = planner.joint_path_deviation(joints['pick_approach'], q)
        segment = planner.segment_problem(joints['pick_approach'], q)
        if segment is not None:
            problems.append(f'descent to the grasp collides: {segment}')
    if 'pick_lift' in joints:
        deviation['lift'] = planner.joint_path_deviation(q, joints['pick_lift'])
        with planner._mode(held_object_m=held_object_m):
            segment = planner.segment_problem(q, joints['pick_lift'])
        if segment is not None:
            problems.append(f'lift with the object collides: {segment}')
    for move, value in deviation.items():
        if value > max_deviation_m:
            problems.append(f'{move}: tool path {value * 1000:.0f} mm from a straight line '
                            f'(max {max_deviation_m * 1000:.0f} mm)')
    result = {'joints': joints, 'flange_poses': poses, 'deviation_m': deviation,
              'tool_tilt_deg': tool_tilt_deg(grasp), 'lift_mode': lift_mode}
    return result, problems


def override_record(class_name, result, grasp_joints, controller_pose, *, back_m, lift_m, note=''):
    """JSON-ready record of a derived pick (result of derive_pick with no problems)."""
    if set(result['joints']) != set(PICK_WAYPOINTS):
        raise ValueError('incomplete pick: every waypoint must be derived before saving')
    return {
        'schema_version': 1,
        'type': 'grasp_override',
        'object_class': class_name,
        'taught_at_utc': datetime.now(timezone.utc).isoformat(),
        'tcp_pose': ZERO_TCP,
        'back_m': back_m,
        'lift_m': lift_m,
        'tool_tilt_deg': round(result['tool_tilt_deg'], 2),
        'lift_mode': result['lift_mode'],
        'deviation_m': {k: round(v, 5) for k, v in result['deviation_m'].items()},
        'controller_grasp': {'joints': list(grasp_joints), 'tcp_pose': list(controller_pose)},
        'waypoints': {name: {'joints': list(result['joints'][name]),
                             'flange_pose': list(result['flange_poses'][name]),
                             'source': 'taught' if name == 'pick_grasp' else 'derived'}
                      for name in PICK_WAYPOINTS},
        'note': note,
    }


def load_grasp_override(path, class_name):
    """{'joints', 'flange_poses', 'sha256'} of a saved override, re-checked. Raises ValueError."""
    raw = Path(path).read_bytes()
    data = json.loads(raw)
    if data.get('schema_version') != 1 or data.get('type') != 'grasp_override':
        raise ValueError(f'{path}: not a grasp override file')
    if data.get('object_class') != class_name:
        raise ValueError(f'{path}: object class {data.get("object_class")!r}, expected {class_name!r}')
    if data.get('tcp_pose') != ZERO_TCP:
        raise ValueError(f'{path}: only overrides recorded with TCP = 0 are supported')
    waypoints = data.get('waypoints', {})
    if set(waypoints) != set(PICK_WAYPOINTS):
        raise ValueError(f'{path}: waypoints must be {PICK_WAYPOINTS}')
    joints, poses = {}, {}
    for name in PICK_WAYPOINTS:
        q = validate_joints(waypoints[name]['joints'])
        pose = validate_pose(waypoints[name]['flange_pose'])
        position_error, orientation_error = _pose_error(forward(q, ZERO_TCP), pose)
        if position_error > FILE_TOLERANCE_M or orientation_error > 0.005:
            raise ValueError(f'{path}: {name} joints and pose do not match (file edited?)')
        joints[name], poses[name] = q, pose
    controller = data.get('controller_grasp', {})
    if controller.get('joints') != waypoints['pick_grasp']['joints']:
        raise ValueError(f'{path}: pick_grasp joints differ from the recorded controller reading')
    position_error, orientation_error = _pose_error(poses['pick_grasp'], validate_pose(controller.get('tcp_pose')))
    if position_error > MODEL_TOLERANCE_M or orientation_error > MODEL_TOLERANCE_RAD:
        raise ValueError(f'{path}: pick_grasp differs from the controller pose recorded with it')
    return {'joints': joints, 'flange_poses': poses, 'sha256': hashlib.sha256(raw).hexdigest()}


def apply_grasp_overrides(taught, overrides, *, root):
    """Copy of the taught sets with the pick waypoints of the overridden classes replaced."""
    out = dict(taught)
    for class_name, relative in (overrides or {}).items():
        if class_name not in taught:
            raise ValueError(f'grasp override for {class_name!r}: no taught references for this class')
        loaded = load_grasp_override(Path(root) / relative, class_name)
        base = taught[class_name]
        out[class_name] = replace(
            base,
            joints={**base.joints, **loaded['joints']},
            flange_poses={**base.flange_poses, **loaded['flange_poses']},
            source_sha256={**base.source_sha256, **{w: loaded['sha256'] for w in PICK_WAYPOINTS}})
    return out
