import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from sorting_fsm.adapters.ur5_rtde_executor import UR5RTDEExecutor
from sorting_fsm.cell import load_cell, make_request, point_in_polygon, task_poses, unmeasured_entries
from sorting_fsm.contracts import Action
from sorting_fsm.mock_executor import MockExecutor
from sorting_fsm.motion_planner import MotionPlanner, MotionPlanningError, load_motion_config
from sorting_fsm.pick_place_machine import TERMINAL_STATES, PickPlaceMachine, Step
from sorting_fsm.ur5_kinematics import forward

ROOT = Path(__file__).resolve().parents[1]
POLICY = json.loads((ROOT / 'configs' / 'sorting_policy.json').read_text(encoding='utf-8'))
GRIPPER = json.loads((ROOT / 'configs' / 'gripper_io.json').read_text(encoding='utf-8'))
MOTION = load_motion_config()
PLANNER = MotionPlanner(MOTION)
SPONGE_XYZ, BOTTLE_XYZ = (-0.50, -0.20, 0.0), (-0.45, -0.30, 0.0)


class CellTests(unittest.TestCase):
    def setUp(self):
        self.cell = load_cell()

    def test_policy_decides_the_drop_zone(self):
        self.assertEqual(make_request('a', 'sponge', SPONGE_XYZ, POLICY).target_zone, 'RIGHT_CELL')
        self.assertEqual(make_request('b', 'plastic_bottle', BOTTLE_XYZ, POLICY).target_zone, 'LEFT_CELL')
        with self.assertRaises(ValueError):
            make_request('c', 'banana', SPONGE_XYZ, POLICY)

    def test_poses_are_top_down_above_the_surface(self):
        poses = task_poses(self.cell, make_request('a', 'sponge', SPONGE_XYZ, POLICY))
        self.assertAlmostEqual(poses.grasp[2], self.cell['pick_zone']['surface_z_m'] + 0.015)
        self.assertAlmostEqual(poses.pre_grasp[2] - poses.grasp[2], self.cell['approach_height_m'])
        self.assertEqual(poses.release, self.cell['drop_zones']['RIGHT_CELL']['release_pose'])

    def test_object_outside_pick_zone_is_refused(self):
        with self.assertRaisesRegex(ValueError, 'outside the pick zone'):
            task_poses(self.cell, make_request('a', 'sponge', (0.3, 0.3, 0.0), POLICY))

    def test_point_in_polygon(self):
        square = [[0, 0], [1, 0], [1, 1], [0, 1]]
        self.assertTrue(point_in_polygon(0.5, 0.5, square))
        self.assertFalse(point_in_polygon(1.5, 0.5, square))

    def test_placeholder_cell_is_reported_unmeasured(self):
        self.assertIn('pick_zone', unmeasured_entries(self.cell))


class LinearPlanTests(unittest.TestCase):
    def test_vertical_descent_to_near_table(self):
        q = [0.5, -1.2, 1.5, -1.9, -1.57, 0.0]
        target = forward(q, PLANNER.tcp_pose)
        target[2] = 0.01
        plan = PLANNER.plan_linear(q, target)
        self.assertLess(math.dist(forward(plan.end_joints, PLANNER.tcp_pose)[:3], target[:3]), 1e-6)

    def test_linear_refusals(self):
        q = [0.5, -1.2, 1.5, -1.9, -1.57, 0.0]
        pose = forward(q, PLANNER.tcp_pose)
        below = pose[:2] + [-0.01] + pose[3:]
        far = [pose[0] + 0.5] + pose[1:]
        rotated = pose[:3] + [0.0, 0.0, 0.0]
        for target, pattern in ((below, 'table'), (far, 'max_linear'), (rotated, 'orientation')):
            with self.assertRaisesRegex(MotionPlanningError, pattern):
                PLANNER.plan_linear(q, target)


def make_machine(executor=None, *, clock=None, journal=None, **kwargs):
    cell = load_cell()
    now = [0.0]
    clock = clock or (lambda: now[0])
    executor = executor or MockExecutor(clock=clock, delay_s=0.1, start_joints=cell['observe']['joints'],
                                        tcp_pose=MOTION['tcp_pose'], **kwargs.pop('mock', {}))
    journal = journal or str(Path(tempfile.mkdtemp()) / 'journal.jsonl')
    machine = PickPlaceMachine(executor, cell=cell, clock=clock, journal_path=journal, planner=PLANNER, **kwargs)
    return machine, executor, now, journal


def run(machine, now, limit=10000):
    for _ in range(limit):
        if machine.state in TERMINAL_STATES:
            return machine.state
        now[0] += 0.05
        machine.tick()
    return machine.state


def records(journal):
    return [json.loads(line) for line in Path(journal).read_text(encoding='utf-8').splitlines()]


class PickPlaceMachineTests(unittest.TestCase):
    def test_both_routes_complete_with_the_expected_stage_order(self):
        for cls, xyz, zone in (('sponge', SPONGE_XYZ, 'RIGHT_CELL'), ('plastic_bottle', BOTTLE_XYZ, 'LEFT_CELL')):
            machine, ex, now, journal = make_machine()
            request = make_request('t1', cls, xyz, POLICY)
            machine.submit(request)
            self.assertEqual(ex.commands, [])          # submit sends nothing
            machine.approve('t1')
            self.assertEqual(run(machine, now), 'COMPLETED')
            stages = [r['to_state'] for r in records(journal) if r['event'] == 'stage_started']
            self.assertEqual(stages, ['TRANSFER_TO_PICK', 'OPEN_GRIPPER', 'DESCEND_TO_GRASP', 'CLOSE_GRIPPER',
                                      'LIFT', 'TRANSFER_TO_DROP', 'DESCEND_TO_RELEASE', 'RELEASE',
                                      'RETREAT', 'RETURN_TO_OBSERVE'])
            self.assertEqual(machine.result.target_zone, zone)
            self.assertFalse(machine.result.physical_grasp_verified)
            release = load_cell()['drop_zones'][zone]['release_pose']
            poses = [c['pose'] for c in ex.commands if c['action'] == Action.MOVE_POSE.value]
            self.assertIn(release, poses)

    def test_infeasible_sequence_sends_nothing(self):
        machine, ex, now, journal = make_machine()
        cell = machine.cell
        cell['drop_zones']['RIGHT_CELL']['release_pose'] = [1.5, 0.0, 0.2, math.pi, 0.0, 0.0]
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        self.assertEqual(machine.approve('t1'), 'FAILED')
        self.assertEqual(ex.commands, [])
        self.assertIn('TRANSFER_TO_DROP infeasible', records(journal)[-1]['reason'])

    def test_no_command_without_approval(self):
        machine, ex, now, _ = make_machine()
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        for _ in range(50):
            now[0] += 0.1
            machine.tick()
        self.assertEqual((machine.state, ex.commands), ('WAIT_APPROVAL', []))

    def test_failure_latches_without_gripper_or_recovery(self):
        machine, ex, now, _ = make_machine(mock={'script': {4: 'failure'}})  # the CLOSE command
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        self.assertEqual(run(machine, now), 'FAILED')
        self.assertEqual(len(ex.commands), 4)
        for _ in range(20):
            machine.tick()
        self.assertEqual(len(ex.commands), 4)

    def test_timeout_then_late_success_does_not_advance(self):
        machine, ex, now, journal = make_machine(mock={'script': {1: 'never_complete'}})
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        self.assertEqual(run(machine, now), 'TIMED_OUT')
        self.assertEqual(len(ex.commands), 1)
        self.assertIn('executor_cancel_ack', [r['event'] for r in records(journal)])

    def test_cancel_mid_motion(self):
        machine, ex, now, journal = make_machine()
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        machine.cancel('t1')
        now[0] += 10
        machine.tick()
        self.assertEqual((machine.state, len(ex.commands)), ('CANCELLED', 1))
        events = [r['event'] for r in records(journal)]
        self.assertLess(events.index('logical_cancel'), events.index('executor_cancel_ack'))

    def test_step_mode_waits_for_each_confirmation(self):
        machine, ex, now, _ = make_machine(step_mode=True)
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        self.assertEqual((machine.state, ex.commands), ('WAIT_CONFIRM', []))
        self.assertEqual(machine.pending_step().name, 'TRANSFER_TO_PICK')
        confirmations = 0
        while machine.state not in TERMINAL_STATES:
            if machine.state == 'WAIT_CONFIRM':
                machine.confirm('t1')
                confirmations += 1
            now[0] += 0.05
            machine.tick()
        self.assertEqual((machine.state, confirmations), ('COMPLETED', 10))

    def test_robot_moved_unexpectedly_stops_the_task(self):
        machine, ex, now, journal = make_machine()
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        while machine.state != 'OPEN_GRIPPER':
            now[0] += 0.05
            machine.tick()
        ex._joints[0] += 0.2   # someone pushed the arm
        self.assertEqual(run(machine, now), 'FAILED')
        self.assertIn('away from the expected start', records(journal)[-1]['reason'])

    def test_custom_pick_strategy(self):
        strategy = lambda request, poses: [Step('CLOSE_GRIPPER', 'gripper', action=Action.CLOSE),  # noqa: E731
                                          Step('WIGGLE_UP', 'linear', poses.grasp[:2] + [poses.grasp[2] + 0.01] + poses.grasp[3:])]
        machine, ex, now, journal = make_machine(pick_strategy=strategy)
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        self.assertEqual(run(machine, now), 'COMPLETED')
        self.assertIn('WIGGLE_UP', [r['to_state'] for r in records(journal)])

    def test_single_active_task_and_duplicates(self):
        machine, ex, now, journal = make_machine()
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        with self.assertRaisesRegex(ValueError, 'Busy'):
            machine.submit(make_request('t2', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        run(machine, now)
        machine.acknowledge('t1')
        with self.assertRaisesRegex(ValueError, 'not retryable'):
            machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY), retry=True)
        restarted, _, _, _ = make_machine(journal=journal)
        with self.assertRaisesRegex(ValueError, 'Duplicate'):
            restarted.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))

    def test_real_executor_requires_explicit_opt_in(self):
        class Real:
            simulation_only = False
        with self.assertRaises(ValueError):
            make_machine(Real())


# ---------------------------------------------------------------- fake ur_rtde for the adapter
class FakeRobot:
    def __init__(self, q):
        self.q, self.qd = list(q), [0.0] * 6
        self.tcp_offset = list(MOTION['tcp_pose'])
        self.protective = self.emergency = False
        self.program_running, self.mode = True, 7
        self.calls, self.target = [], None
        self.tcp_error = 0.0

    def arrive(self):
        self.q, self.qd = list(self.target), [0.0] * 6

    def tcp(self):
        pose = forward(self.q, MOTION['tcp_pose'])
        pose[0] += self.tcp_error
        return pose


def fake_interfaces(robot):
    class Receive:
        def __init__(self, ip): pass
        def getActualQ(self): return list(robot.q)
        def getActualQd(self): return list(robot.qd)
        def getActualTCPPose(self): return robot.tcp()
        def isProtectiveStopped(self): return robot.protective
        def isEmergencyStopped(self): return robot.emergency
        def getRobotMode(self): return robot.mode
        def disconnect(self): robot.calls.append('receive.disconnect')

    class Control:
        def __init__(self, ip): robot.calls.append('control.connect')
        def getTCPOffset(self): return list(robot.tcp_offset)
        def isProgramRunning(self): return robot.program_running
        def isJointsWithinSafetyLimits(self, q): return True
        def isPoseWithinSafetyLimits(self, p): return True
        def moveJ(self, q, v, a, asynchronous):
            robot.calls.append(('moveJ', v, a, asynchronous))
            robot.target, robot.qd = list(q), [0.3] * 6
            return True
        def moveL(self, pose, v, a, asynchronous):
            robot.calls.append(('moveL', v, a, asynchronous))
            robot.target, robot.qd = PLANNER.plan_linear(robot.q, pose).end_joints, [0.1] * 6
            return True
        def stopJ(self, a): robot.calls.append('stopJ')
        def stopL(self, a): robot.calls.append('stopL')
        def stopScript(self): robot.calls.append('stopScript')
        def disconnect(self): robot.calls.append('control.disconnect')

    class IO:
        def __init__(self, ip): pass
        def setToolDigitalOut(self, channel, level):
            robot.calls.append(('tool_out', channel, level))
            return True

    return Receive, Control, IO


class RTDEExecutorTests(unittest.TestCase):
    START = [0.5, -1.6, 1.4, -1.4, -1.5708, 0.0]

    def make(self, robot=None, now=None):
        robot = robot or FakeRobot(self.START)
        now = now or [0.0]
        ex = UR5RTDEExecutor('10.0.0.1', motion_config=MOTION, gripper_config=GRIPPER, settle_s=0.5,
                             clock=lambda: now[0], interfaces=fake_interfaces(robot))
        return ex, robot, now

    def test_refuses_before_taking_control_on_calibration_mismatch(self):
        robot = FakeRobot(self.START)
        robot.tcp_error = 0.02
        with self.assertRaisesRegex(RuntimeError, 'mismatch'):
            self.make(robot)
        self.assertNotIn('control.connect', robot.calls)

    def test_refuses_on_tcp_offset_mismatch_or_bad_state(self):
        robot = FakeRobot(self.START)
        robot.tcp_offset[2] = 0.25534
        with self.assertRaisesRegex(RuntimeError, 'Controller TCP'):
            self.make(robot)
        for attribute, value, pattern in (('protective', True, 'protective'), ('emergency', True, 'emergency'),
                                          ('mode', 5, 'RUNNING')):
            robot = FakeRobot(self.START)
            setattr(robot, attribute, value)
            with self.assertRaisesRegex(RuntimeError, pattern):
                self.make(robot)

    def test_joint_move_completes_only_at_rest_on_target(self):
        ex, robot, _ = self.make()
        goal = [0.4, -1.5, 1.4, -1.5, -1.5708, 0.0]
        cid = ex.submit('t', Action.MOVE_JOINTS, joints=goal)
        self.assertEqual(robot.calls[-1], ('moveJ', MOTION['max_joint_speed_rad_s'],
                                           MOTION['max_joint_acceleration_rad_s2'], True))
        self.assertEqual(ex.poll(cid).status, 'running')
        robot.qd = [0.0] * 6                # stopped but not at target: still not completed
        self.assertEqual(ex.poll(cid).status, 'running')
        robot.arrive()
        event = ex.poll(cid)
        self.assertEqual((event.status, event.simulation_only), ('completed', False))

    def test_protective_stop_fails_the_command(self):
        ex, robot, _ = self.make()
        cid = ex.submit('t', Action.MOVE_JOINTS, joints=[0.4, -1.5, 1.4, -1.5, -1.5708, 0.0])
        robot.protective = True
        self.assertEqual(ex.poll(cid).status, 'failed')
        with self.assertRaisesRegex(RuntimeError, 'not ready'):
            ex.submit('t', Action.OPEN)

    def test_gripper_uses_configured_output_and_settle_time(self):
        ex, robot, now = self.make()
        cid = ex.submit('t', Action.CLOSE)
        self.assertEqual(robot.calls[-1], ('tool_out', GRIPPER['tool_output_channel'], GRIPPER['close_level']))
        self.assertEqual(ex.poll(cid).status, 'running')
        now[0] = 0.5
        event = ex.poll(cid)
        self.assertEqual(event.status, 'completed')
        self.assertFalse(event.physical_grasp_verified)

    def test_cancel_requests_controlled_stop(self):
        ex, robot, _ = self.make()
        cid = ex.submit('t', Action.MOVE_JOINTS, joints=[0.4, -1.5, 1.4, -1.5, -1.5708, 0.0])
        self.assertEqual(ex.cancel(cid).status, 'cancelled')
        self.assertIn('stopJ', robot.calls)

    def test_symbolic_waypoints_rejected(self):
        ex, _, _ = self.make()
        with self.assertRaises(ValueError):
            ex.submit('t', Action.MOVE, 'SIM_ONLY_PICK')

    def test_full_task_through_real_adapter_with_fake_robot(self):
        robot = FakeRobot(load_cell()['observe']['joints'])
        now = [0.0]
        ex, _, _ = self.make(robot, now)
        machine, _, _, _ = make_machine(ex, clock=lambda: now[0], require_simulation=False)
        machine.submit(make_request('t1', 'sponge', SPONGE_XYZ, POLICY))
        machine.approve('t1')
        for _ in range(200):
            if machine.state in TERMINAL_STATES:
                break
            now[0] += 1.0
            if robot.target is not None:
                robot.arrive()
            machine.tick()
        self.assertEqual(machine.state, 'COMPLETED')


if __name__ == '__main__':
    unittest.main()
