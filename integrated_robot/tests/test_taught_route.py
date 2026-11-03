import copy
import json
import math
from pathlib import Path
import shutil
import tempfile
import unittest

from sorting_fsm.cell import load_cell, make_request
from sorting_fsm.collision import link_segments
from sorting_fsm.contracts import Action
from sorting_fsm.geometry import rotation_between, slerp_rotvec
from sorting_fsm.mock_executor import MockExecutor
from sorting_fsm.motion_planner import MotionPlanner, MotionPlanningError, load_motion_config
from sorting_fsm.pick_place_machine import TERMINAL_STATES, PickPlaceMachine, Step
from sorting_fsm.taught_route import WAYPOINTS, TaughtRoute, apply_route_aliases, load_taught_catalog
from sorting_fsm.ur5_kinematics import forward

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'data' / 'planning_inputs_four_objects_tcp_zero'
POLICY = json.loads((ROOT / 'configs' / 'sorting_policy.json').read_text(encoding='utf-8'))
CELL = load_cell(ROOT / 'configs' / 'real_cell' / 'cell_config.json')
MOTION = load_motion_config(ROOT / 'configs' / 'real_cell' / 'motion_config.json')
TAUGHT = load_taught_catalog(PACKAGE, POLICY)
OBSERVE = CELL['observe']['joints']   # taught observe pose: every cycle ends there


def end_joints(name):
    return OBSERVE or TAUGHT[name].joints['drop_retreat']
PLANNER = MotionPlanner(MOTION)


def route(**kwargs):
    kwargs.setdefault('incident_reviewed', ('aluminum_can',))
    return TaughtRoute(TAUGHT, CELL, MOTION['tcp_pose'], **kwargs)


def run(machine_route, class_name, xyz=None, *, cell=CELL, planner=PLANNER):
    now = [0.0]
    start = TAUGHT['sponge'].joints['drop_retreat']
    executor = MockExecutor(clock=lambda: now[0], delay_s=0.2, start_joints=start, tcp_pose=MOTION['tcp_pose'])
    journal = Path(tempfile.mkdtemp()) / 'journal.jsonl'
    machine = PickPlaceMachine(executor, cell=cell, clock=lambda: now[0], journal_path=str(journal),
                               planner=planner, route=machine_route)
    xyz = machine_route.grasp_position(class_name) if xyz is None else xyz
    request = make_request('t-' + class_name, class_name, xyz, POLICY)
    machine.submit(request)
    machine.approve(request.task_id)
    while machine.state not in TERMINAL_STATES:
        machine.tick()
        now[0] += 0.05
    return machine, executor, [json.loads(line) for line in journal.read_text(encoding='utf-8').splitlines()]


class CatalogTests(unittest.TestCase):
    def test_loads_four_verified_classes(self):
        self.assertEqual(sorted(TAUGHT), ['aluminum_can', 'paper_cup', 'plastic_bottle', 'sponge'])
        sponge = json.loads((PACKAGE / 'sponge_requests.json').read_text(encoding='utf-8'))
        for w in WAYPOINTS:
            self.assertEqual(TAUGHT['sponge'].joints[w], sponge['waypoints'][w]['joint_positions_rad'])
        self.assertEqual(TAUGHT['sponge'].target_zone, 'RIGHT_CELL')
        self.assertTrue(TAUGHT['aluminum_can'].incident_review_required)
        self.assertFalse(TAUGHT['sponge'].incident_review_required)

    def test_changed_evidence_byte_refuses_everything(self):
        copy_dir = Path(tempfile.mkdtemp()) / 'pkg'
        shutil.copytree(PACKAGE, copy_dir)
        pose = next((copy_dir / 'evidence' / 'poses').glob('*/pose.json'))
        pose.write_bytes(pose.read_bytes().replace(b'"label"', b'"label" ', 1))
        with self.assertRaisesRegex(ValueError, 'Changed evidence'):
            load_taught_catalog(copy_dir, POLICY)

    def test_policy_route_mismatch_refused(self):
        policy = copy.deepcopy(POLICY)
        policy['class_to_category']['sponge'] = 'recyclable'
        with self.assertRaisesRegex(ValueError, 'differs from the sorting policy'):
            load_taught_catalog(PACKAGE, policy)

    def test_our_model_reproduces_the_controller_poses(self):
        # Kinematic check on real data: FK(taught joints) vs pose reported by the controller.
        for taught in TAUGHT.values():
            for w in WAYPOINTS:
                predicted = forward(taught.joints[w], [0.0] * 6)
                self.assertLess(math.dist(predicted[:3], taught.flange_poses[w][:3]), 0.003)

    def test_all_taught_postures_valid_in_real_cell_model(self):
        for taught in TAUGHT.values():
            for w in WAYPOINTS:
                self.assertIsNone(PLANNER.state_problem(taught.joints[w]), (taught.object_class, w))


class RouteTests(unittest.TestCase):
    def test_fixed_placement_steps_are_the_taught_postures(self):
        r = route()
        request = make_request('t1', 'plastic_bottle', r.grasp_position('plastic_bottle'), POLICY)
        steps = r.steps(request)
        q = TAUGHT['plastic_bottle'].joints
        self.assertEqual([s.name for s in steps],
                         ['TRANSFER_TO_PICK', 'OPEN_GRIPPER', 'DESCEND_TO_GRASP', 'CLOSE_GRIPPER', 'LIFT',
                          'TRANSFER_TO_DROP', 'DESCEND_TO_RELEASE', 'RELEASE', 'RETREAT']
                         + (['RETURN_TO_OBSERVE'] if OBSERVE else []))
        targets = [s.target for s in steps if s.kind == 'joint_q']
        self.assertEqual(targets, [q[w] for w in WAYPOINTS] + ([OBSERVE] if OBSERVE else []))
        for s in steps:
            if s.name in ('DESCEND_TO_GRASP', 'LIFT', 'DESCEND_TO_RELEASE', 'RETREAT'):
                self.assertEqual(s.max_path_deviation_m, 0.01)
        self.assertGreater(steps[4].held_object_m, 0)
        self.assertEqual(steps[8].held_object_m, 0)

    def test_grasp_position_applies_the_controller_tcp(self):
        tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
        with_tcp = TaughtRoute(TAUGHT, CELL, tcp).grasp_position('sponge')
        flange = TAUGHT['sponge'].flange_poses['pick_grasp']
        self.assertAlmostEqual(math.dist(with_tcp, flange[:3]), 0.15, places=9)

    def test_untaught_class_refused_never_another_plan(self):
        r = route()
        request = make_request('t1', 'toothbrush', (0.21, 0.03, 0.6), POLICY)
        with self.assertRaisesRegex(ValueError, 'No taught references'):
            r.steps(request)

    def test_incident_class_needs_review(self):
        r = route(incident_reviewed=())
        request = make_request('t1', 'aluminum_can', (0.21, 0.02, 0.6), POLICY)
        with self.assertRaisesRegex(ValueError, 'protective stop'):
            r.steps(request)

    def test_offset_refused_in_fixed_placement(self):
        r = route()
        x, y, z = r.grasp_position('sponge')
        with self.assertRaisesRegex(ValueError, 'taught pick spot'):
            r.steps(make_request('t1', 'sponge', (x + 0.01, y, z), POLICY))

    def test_measured_position_translates_the_pick(self):
        r = route(max_pick_offset_m=0.05)
        x, y, z = r.grasp_position('sponge')
        steps = r.steps(make_request('t1', 'sponge', (x + 0.02, y - 0.01, z), POLICY))
        by_name = {s.name: s for s in steps}
        self.assertEqual(by_name['TRANSFER_TO_PICK'].kind, 'joint_pose')
        self.assertEqual(by_name['TRANSFER_TO_PICK'].reference_joints, TAUGHT['sponge'].joints['pick_approach'])
        grasp = by_name['DESCEND_TO_GRASP']
        self.assertEqual(grasp.kind, 'linear')
        taught = TAUGHT['sponge'].flange_poses['pick_grasp']
        self.assertAlmostEqual(grasp.target[0], taught[0] + 0.02)
        self.assertAlmostEqual(grasp.target[1], taught[1] - 0.01)
        for got, want in zip(grasp.target[2:], taught[2:]):
            self.assertAlmostEqual(got, want, places=12)
        self.assertEqual(by_name['TRANSFER_TO_DROP'].target, TAUGHT['sponge'].joints['drop_approach'])

    def test_measured_position_outside_pick_zone_refused(self):
        r = route(max_pick_offset_m=0.2)
        x, y, z = r.grasp_position('sponge')
        with self.assertRaisesRegex(ValueError, 'outside the pick zone'):
            r.steps(make_request('t1', 'sponge', (x + 0.1, y, z), POLICY))

    def test_pick_strategy_hook_kept(self):
        def strategy(request, poses):
            return [Step('CLOSE_GRIPPER', 'gripper', action=Action.CLOSE),
                    Step('SETTLE_CHECK', 'gripper', action=Action.CLOSE)]
        r = route(pick_strategy=strategy)
        names = [s.name for s in r.steps(make_request('t1', 'sponge', r.grasp_position('sponge'), POLICY))]
        self.assertIn('SETTLE_CHECK', names)


class SimulationTests(unittest.TestCase):
    def test_every_taught_class_completes_and_ends_on_taught_joints(self):
        for name in TAUGHT:
            machine, executor, _ = run(route(), name)
            self.assertEqual(machine.state, 'COMPLETED', name)
            final = executor.current_joint_positions()
            self.assertLess(max(abs(a - b) for a, b in zip(final, end_joints(name))), 1e-9)

    def test_untaught_class_rejected_without_command(self):
        now = [0.0]
        executor = MockExecutor(clock=lambda: now[0], start_joints=TAUGHT['sponge'].joints['drop_retreat'],
                                tcp_pose=MOTION['tcp_pose'])
        machine = PickPlaceMachine(executor, cell=CELL, clock=lambda: now[0], planner=PLANNER, route=route(),
                                   journal_path=str(Path(tempfile.mkdtemp()) / 'j.jsonl'))
        with self.assertRaises(ValueError):
            machine.submit(make_request('t1', 'toothbrush', (0.21, 0.03, 0.6), POLICY))
        self.assertEqual(machine.state, 'IDLE')
        self.assertEqual(executor.commands, [])

    def test_too_curved_approach_rejected_before_any_command(self):
        tight = route(max_path_deviation_m=0.0001)
        machine, executor, journal = run(tight, 'paper_cup')
        self.assertEqual(machine.state, 'FAILED')
        self.assertIn('deviates', journal[-1]['reason'])
        self.assertEqual(executor.commands, [])

    def test_route_and_machine_pick_strategy_are_exclusive(self):
        with self.assertRaises(ValueError):
            PickPlaceMachine(MockExecutor(), cell=CELL, clock=lambda: 0.0, route=route(),
                             pick_strategy=lambda r, p: [], journal_path=str(Path(tempfile.mkdtemp()) / 'j.jsonl'))


class LinearRotationTests(unittest.TestCase):
    def test_slerp_endpoints_and_midpoint(self):
        a, b = [0.0, 0.0, 0.0], [0.0, 0.0, 0.2]
        self.assertLess(rotation_between(slerp_rotvec(a, b, 0.0), a), 1e-12)
        self.assertLess(rotation_between(slerp_rotvec(a, b, 1.0), b), 1e-12)
        self.assertAlmostEqual(rotation_between(slerp_rotvec(a, b, 0.5), a), 0.1, places=9)

    def test_small_rotation_allowed_only_when_configured(self):
        q = TAUGHT['sponge'].joints['pick_approach']
        start = forward(q, MOTION['tcp_pose'])
        target = [start[0], start[1], start[2] - 0.03] + slerp_rotvec(start[3:], [-1.0, 1.6, -1.4], 0.5)
        self.assertGreater(rotation_between(start[3:], target[3:]), 0.01)
        plan = PLANNER.plan_linear(q, target)
        self.assertLess(math.dist(forward(plan.end_joints, MOTION['tcp_pose'])[:3], target[:3]), 1e-6)
        strict = copy.deepcopy(MOTION)
        del strict['planner']['max_linear_rotation_rad']
        with self.assertRaisesRegex(MotionPlanningError, 'orientation'):
            MotionPlanner(strict).plan_linear(q, target)

    def test_tool_capsule_follows_the_flange(self):
        q = TAUGHT['sponge'].joints['pick_grasp']
        capsule = MOTION['collision']['tool_capsule_flange_m']
        start, end = link_segments(q, MOTION['tcp_pose'], tool_capsule=capsule)['tool']
        self.assertAlmostEqual(math.dist(start, end), 0.20, places=9)
        _, longer = link_segments(q, MOTION['tcp_pose'], tool_end_offset=0.05, tool_capsule=capsule)['tool']
        self.assertAlmostEqual(math.dist(start, longer), 0.25, places=9)


class RouteAliasTests(unittest.TestCase):
    def test_alias_uses_the_whole_route_of_the_other_class(self):
        aliased = apply_route_aliases(TAUGHT, {'aluminum_can': 'plastic_bottle'})
        can, bottle = aliased['aluminum_can'], TAUGHT['plastic_bottle']
        self.assertEqual(can.object_class, 'aluminum_can')
        self.assertEqual(can.joints, bottle.joints)            # pick AND drop of the bottle
        self.assertEqual(can.flange_poses, bottle.flange_poses)
        self.assertFalse(can.incident_review_required)         # none of the can's own postures used
        self.assertIs(aliased['plastic_bottle'], bottle)
        self.assertTrue(TAUGHT['aluminum_can'].incident_review_required)   # input not modified

    def test_aliased_class_runs_without_incident_review(self):
        r = TaughtRoute(apply_route_aliases(TAUGHT, {'aluminum_can': 'plastic_bottle'}), CELL,
                        MOTION['tcp_pose'], incident_reviewed=())
        machine, executor, _ = run(r, 'aluminum_can')
        self.assertEqual(machine.state, 'COMPLETED')
        final = executor.current_joint_positions()
        want = end_joints('plastic_bottle')
        self.assertLess(max(abs(a - b) for a, b in zip(final, want)), 1e-9)

    def test_alias_refusals(self):
        with self.assertRaisesRegex(ValueError, 'no taught references'):
            apply_route_aliases(TAUGHT, {'aluminum_can': 'toothbrush'})
        with self.assertRaisesRegex(ValueError, 'no taught references'):
            apply_route_aliases(TAUGHT, {'toothbrush': 'plastic_bottle'})
        with self.assertRaisesRegex(ValueError, 'target zone'):
            apply_route_aliases(TAUGHT, {'aluminum_can': 'sponge'})
        with self.assertRaisesRegex(ValueError, 'chain'):
            apply_route_aliases(TAUGHT, {'aluminum_can': 'plastic_bottle', 'plastic_bottle': 'paper_cup'})
        with self.assertRaisesRegex(ValueError, 'incident'):
            apply_route_aliases(TAUGHT, {'plastic_bottle': 'aluminum_can'})


if __name__ == '__main__':
    unittest.main()
