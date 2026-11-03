"""Automatic picking: started once the reaching phase has put the gripper at the pre-grasp pose,
a safe distance from the object. Nothing is done by hand:

  OPEN_GRIPPER -> DESCEND_TO_GRASP -> <grip steps, default CLOSE_GRIPPER> -> LIFT

* the gripper is driven by the program (tool digital output 0: ON closes, OFF opens, see
  configs/gripper_io.json); each gripper step waits gripper.settle_s before the next one;
* the approach and lift are short, near-straight moves, planned and collision-checked like every
  other stage (with the held object during the lift);
* there is no grasp sensor with the digital-output control: "picked" means the close command
  completed, not that the object is held. The operator watches the first runs.

The machine can stop once before the picking (`confirm_phases=('picking',)`): the robot waits at
the pre-grasp pose, the operator checks the distance and starts the whole picking with one Enter.
"""
from dataclasses import replace

from .contracts import Action
from .steps import Step


def picking_steps(grasp_target, lift_target, *, kind, grip_steps=None, held_object_m=0.0,
                  max_path_deviation_m=None):
    """Stages of the picking phase.

    kind: 'linear' (targets are TCP poses, moveL) or 'joint_q' (targets are taught joints, moveJ
    whose TCP stays within max_path_deviation_m of a straight line).
    grip_steps: gripper / linear Steps done at the grasp pose (default: close the gripper).
    """
    if kind not in ('linear', 'joint_q'):
        raise ValueError("picking moves are 'linear' or 'joint_q'")
    grip = [Step('CLOSE_GRIPPER', 'gripper', action=Action.CLOSE)] if grip_steps is None else list(grip_steps)
    if not grip or not all(isinstance(s, Step) and s.kind in ('gripper', 'linear') for s in grip):
        raise ValueError('grip steps must be a non-empty list of gripper or linear Steps')
    if not any(s.kind == 'gripper' and s.action is Action.CLOSE for s in grip):
        raise ValueError('the grip steps must close the gripper')
    straight = max_path_deviation_m if kind == 'joint_q' else None
    held = held_object_m if kind == 'joint_q' else 0.0
    return [Step('OPEN_GRIPPER', 'gripper', action=Action.OPEN, phase='picking'),
            Step('DESCEND_TO_GRASP', kind, grasp_target, max_path_deviation_m=straight, phase='picking'),
            *(replace(s, phase='picking') for s in grip),
            Step('LIFT', kind, lift_target, held_object_m=held, max_path_deviation_m=straight, phase='picking')]
