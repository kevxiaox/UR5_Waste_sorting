import json
from pathlib import Path
import tempfile
import unittest

from sorting_fsm.cell import load_cell, make_request
from sorting_fsm.contracts import Action
from sorting_fsm.mock_executor import MockExecutor
from sorting_fsm.motion_planner import MotionPlanner, load_motion_config
from sorting_fsm.pick_place_machine import TERMINAL_STATES, PickPlaceMachine
from sorting_fsm.picking import picking_steps
from sorting_fsm.steps import Step
from sorting_fsm.taught_route import TaughtRoute, load_taught_catalog

ROOT = Path(__file__).resolve().parents[1]
POLICY = json.loads((ROOT / 'configs' / 'sorting_policy.json').read_text(encoding='utf-8'))
CELL = load_cell(ROOT / 'configs' / 'real_cell' / 'cell_config.json')
MOTION = load_motion_config(ROOT / 'configs' / 'real_cell' / 'motion_config.json')
TAUGHT = load_taught_catalog(ROOT / 'data' / 'planning_inputs_four_objects_tcp_zero', POLICY)
ROUTE = TaughtRoute(TAUGHT, CELL, MOTION['tcp_pose'])


class PickingStepsTests(unittest.TestCase):
    def test_automatic_sequence(self):
        steps = picking_steps([0.1] * 6, [0.2] * 6, kind='joint_q', held_object_m=0.05, max_path_deviation_m=0.01)
        self.assertEqual([s.name for s in steps], ['OPEN_GRIPPER', 'DESCEND_TO_GRASP', 'CLOSE_GRIPPER', 'LIFT'])
        self.assertEqual({s.phase for s in steps}, {'picking'})
        self.assertEqual([s.action for s in steps if s.kind == 'gripper'], [Action.OPEN, Action.CLOSE])
        self.assertEqual(steps[3].held_object_m, 0.05)
        self.assertEqual(steps[1].max_path_deviation_m, 0.01)

    def test_grip_must_close_and_kind_checked(self):
        with self.assertRaisesRegex(ValueError, 'close'):
            picking_steps([0.1] * 6, [0.2] * 6, kind='joint_q',
                          grip_steps=[Step('OPEN_AGAIN', 'gripper', action=Action.OPEN)])
        with self.assertRaises(ValueError):
            picking_steps([0.1] * 6, [0.2] * 6, kind='joint_pose')

    def test_route_phases_in_order(self):
        steps = ROUTE.steps(make_request('t1', 'sponge', ROUTE.grasp_position('sponge'), POLICY))
        self.assertEqual([(s.name, s.phase) for s in steps], [
            ('TRANSFER_TO_PICK', 'reaching'), ('OPEN_GRIPPER', 'picking'), ('DESCEND_TO_GRASP', 'picking'),
            ('CLOSE_GRIPPER', 'picking'), ('LIFT', 'picking'), ('TRANSFER_TO_DROP', 'transport'),
            ('DESCEND_TO_RELEASE', 'placing'), ('RELEASE', 'placing'), ('RETREAT', 'placing')]
            + ([('RETURN_TO_OBSERVE', 'return')] if CELL['observe']['joints'] else []))


class ConfirmBeforePickingTests(unittest.TestCase):
    def make(self, **kwargs):
        now = [0.0]
        executor = MockExecutor(clock=lambda: now[0], delay_s=0.2, tcp_pose=MOTION['tcp_pose'],
                                start_joints=TAUGHT['sponge'].joints['drop_retreat'])
        machine = PickPlaceMachine(executor, cell=CELL, clock=lambda: now[0], planner=MotionPlanner(MOTION),
                                   route=ROUTE, journal_path=str(Path(tempfile.mkdtemp()) / 'j.jsonl'), **kwargs)
        return machine, executor, now

    def run_until_wait_or_end(self, machine, now):
        while machine.state not in TERMINAL_STATES + ('WAIT_CONFIRM',):
            machine.tick()
            now[0] += 0.05

    def test_waits_once_at_pre_grasp_then_picks_automatically(self):
        machine, executor, now = self.make(confirm_phases=('picking',))
        request = make_request('t1', 'sponge', ROUTE.grasp_position('sponge'), POLICY)
        machine.submit(request)
        machine.approve('t1')
        self.run_until_wait_or_end(machine, now)
        self.assertEqual(machine.state, 'WAIT_CONFIRM')
        self.assertEqual(machine.pending_step().name, 'OPEN_GRIPPER')
        # Reaching done: the robot waits at the taught pre-grasp posture, gripper untouched.
        at = executor.current_joint_positions()
        self.assertLess(max(abs(a - b) for a, b in zip(at, TAUGHT['sponge'].joints['pick_approach'])), 1e-9)
        self.assertFalse(any(c['action'] in (Action.OPEN, Action.CLOSE) for c in executor.commands))
        machine.confirm('t1')
        self.run_until_wait_or_end(machine, now)
        self.assertEqual(machine.state, 'COMPLETED')   # no other confirmation needed
        gripper = [c['action'] for c in executor.commands if c['action'] in (Action.OPEN, Action.CLOSE)]
        self.assertEqual(gripper, [Action.OPEN, Action.CLOSE, Action.OPEN])

    def test_cancel_at_pre_grasp_sends_nothing_more(self):
        machine, executor, now = self.make(confirm_phases=('picking',))
        machine.submit(make_request('t1', 'sponge', ROUTE.grasp_position('sponge'), POLICY))
        machine.approve('t1')
        self.run_until_wait_or_end(machine, now)
        sent = len(executor.commands)
        machine.cancel('t1')
        self.assertEqual(machine.state, 'CANCELLED')
        self.assertEqual(len(executor.commands), sent)

    def test_unknown_phase_rejected(self):
        with self.assertRaises(ValueError):
            self.make(confirm_phases=('grasping',))


if __name__ == '__main__':
    unittest.main()
