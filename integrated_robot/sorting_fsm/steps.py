"""Stage description shared by the routes, the picking function and the pick-and-place machine."""
from dataclasses import dataclass

from .contracts import Action

STEP_KINDS = ('joint_pose', 'joint_q', 'linear', 'gripper')
# Phases of one task, in order: reaching = move to the pre-grasp pose (at a safe distance from the
# object); picking = the automatic grasp (picking.picking_steps); transport; placing; return.
PHASES = ('reaching', 'picking', 'transport', 'placing', 'return')


@dataclass(frozen=True)
class Step:
    name: str
    kind: str                    # one of STEP_KINDS
    target: list | None = None   # TCP pose (joint_pose, linear) or joints (joint_q)
    action: Action | None = None # OPEN / CLOSE for gripper steps
    held_object_m: float = 0.0   # object hanging below the TCP during a joint move
    # joint_pose steps only: choose the IK solution nearest to these joints (arm configuration).
    reference_joints: list | None = None
    # joint_q steps only: the move must be one direct joint segment whose TCP and tool tip stay
    # within this distance of their straight line (approach/retreat replayed as moveJ).
    max_path_deviation_m: float | None = None
    phase: str = ''              # one of PHASES ('' = not assigned)


def default_pick_strategy(request, poses):
    """Grip part of the picking: close the gripper at the grasp pose. A picking variant can
    return other gripper / linear steps (e.g. close, small extra move)."""
    return [Step('CLOSE_GRIPPER', 'gripper', action=Action.CLOSE)]
