"""Starter integration contract. Every accepted plan here is simulation-only."""
from dataclasses import dataclass
from enum import Enum
import math
from typing import Protocol


class Action(str, Enum):
    MOVE = 'move_to_waypoint'
    MOVE_POSE = 'move_linear_to_pose'
    MOVE_JOINTS = 'move_joint_space'
    OPEN = 'open_gripper'
    CLOSE = 'close_gripper'


@dataclass(frozen=True)
class CommandEvent:
    command_id: str
    task_id: str
    status: str
    reason: str = ''
    simulation_only: bool = True
    physical_grasp_verified: bool = False
    physical_release_verified: bool = False


class Executor(Protocol):
    simulation_only: bool

    def submit(self, task_id: str, action: Action, waypoint: str | None = None,
               *, pose: list[float] | None = None, joints: list[float] | None = None) -> str: ...
    def poll(self, command_id: str) -> CommandEvent: ...
    def cancel(self, command_id: str) -> CommandEvent: ...
    def current_tcp_pose(self) -> list[float]: ...
    def current_joint_positions(self) -> list[float]: ...


def validate_pose(pose) -> list[float]:
    """UR convention [x, y, z, rx, ry, rz]: metres and rotation vector in radians."""
    if (not isinstance(pose, (list, tuple)) or len(pose) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)):
        raise ValueError('Pose must be 6 finite numbers [x, y, z, rx, ry, rz]')
    return [float(v) for v in pose]


def validate_joints(joints) -> list[float]:
    """Six UR joint angles in radians (base, shoulder, elbow, wrist 1, 2, 3)."""
    if (not isinstance(joints, (list, tuple)) or len(joints) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in joints)):
        raise ValueError('Joints must be 6 finite numbers in radians')
    return [float(v) for v in joints]


def validate_proposal(proposal: dict, policy: dict) -> dict:
    """Policy checks on a visual proposal; returns its category and route."""
    if proposal.get('schema_version') != 1:
        raise ValueError('Unsupported schema')
    if proposal.get('simulation_only') is not True:
        raise ValueError('Only simulation proposals and plans are supported')
    for data in (proposal, policy):
        if data.get('robot_execution_enabled') is not False:
            raise ValueError('Real execution must remain disabled')
    task_id = proposal.get('task_id')
    if not isinstance(task_id, str) or not task_id.strip():
        raise ValueError('Missing or mismatched task ID')
    name = proposal.get('class_name')
    category = policy['class_to_category'].get(name)
    if category is None:
        raise ValueError('Unknown class')
    route = policy['category_to_zone'][category]
    if proposal.get('target_zone') != route:
        raise ValueError('Route mismatch')
    confidence = proposal.get('confidence')
    if (type(confidence) not in (int, float) or not math.isfinite(confidence)
            or not policy['min_confidence'] <= confidence <= 1):
        raise ValueError('Invalid or below-policy confidence')
    if proposal.get('vision_review_passed') is not True or proposal.get('stable_window_frames') != 20:
        raise ValueError('Stable visual observation required')
    if proposal.get('physical_ready') is not False:
        raise ValueError('Do not promote a saved visual observation to physical readiness')
    return {'class_name': name, 'category': category, 'target_zone': route}


def validate_simulation_task(proposal: dict, plan: dict, policy: dict) -> None:
    """Validate mock fixtures, NOT physical readiness or authorization."""
    if plan.get('schema_version') != 1:
        raise ValueError('Unsupported schema')
    if plan.get('simulation_only') is not True:
        raise ValueError('Only simulation proposals and plans are supported')
    if plan.get('robot_execution_enabled') is not False:
        raise ValueError('Real execution must remain disabled')
    route = validate_proposal(proposal, policy)['target_zone']
    if proposal['task_id'] != plan.get('task_id'):
        raise ValueError('Missing or mismatched task ID')
    if plan.get('target_zone') != route:
        raise ValueError('Route mismatch')
    waypoints = plan.get('waypoints')
    required = ('pick_approach', 'pick_grasp', 'pick_lift', 'drop_approach', 'drop_release', 'drop_retreat')
    if not isinstance(waypoints, dict):
        raise ValueError('Missing waypoint map')
    if any(not isinstance(waypoints.get(key), str) or not waypoints[key].startswith('SIM_ONLY_')
           for key in required):
        raise ValueError('Only explicit SIM_ONLY symbolic waypoints are accepted')


def validate_timeout(seconds: float) -> None:
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or seconds <= 0:
        raise ValueError('Timeout must be a positive finite number')
