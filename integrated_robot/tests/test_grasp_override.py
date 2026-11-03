import copy
import json
import math
from pathlib import Path
import tempfile
import unittest

from sorting_fsm.geometry import rotvec_to_matrix, tool_z_axis
from sorting_fsm.grasp_override import (apply_grasp_overrides, derive_pick, load_grasp_override,
                                        override_record, tool_tilt_deg)
from sorting_fsm.taught_route import TaughtRoute
from sorting_fsm.ur5_kinematics import forward, matrix_to_rotvec

from test_taught_route import CELL, MOTION, PLANNER, TAUGHT, run

SPONGE = TAUGHT['sponge']
HELD = CELL['grasp']['classes']['sponge']['held_object_length_m']


def derive(q, controller_pose, **kw):
    kw.setdefault('back_m', 0.08)
    kw.setdefault('lift_m', 0.08)
    return derive_pick(PLANNER, q, controller_pose, max_deviation_m=0.01, held_object_m=HELD, **kw)


def tilt_pose(pose, extra_deg, tool_length):
    """Rotate the tool further down by extra_deg about a horizontal axis, fingertips fixed."""
    rot = rotvec_to_matrix(pose[3:])
    axis = [row[2] for row in rot]
    h = [axis[0], axis[1], 0.0]
    n = math.hypot(h[0], h[1])
    hinge = [h[1] / n, -h[0] / n, 0.0]          # h x z: tilting about it turns the tool downward
    a = math.radians(extra_deg)
    turn = rotvec_to_matrix([hinge[0] * a, hinge[1] * a, 0.0])
    new = [[sum(turn[i][k] * rot[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    new_axis = [row[2] for row in new]
    if new_axis[2] > axis[2]:                    # wrong way: tilt the other side
        turn = rotvec_to_matrix([-hinge[0] * a, -hinge[1] * a, 0.0])
        new = [[sum(turn[i][k] * rot[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        new_axis = [row[2] for row in new]
    tip = [p + tool_length * x for p, x in zip(pose[:3], axis)]
    position = [t - tool_length * x for t, x in zip(tip, new_axis)]
    return position + matrix_to_rotvec(new)


def tilted_sponge_grasp(extra_deg):
    """Joints of the sponge grasp tilted further down by extra_deg, fingertips (0.20 m) fixed."""
    target = tilt_pose(forward(SPONGE.joints['pick_grasp'], [0] * 6), extra_deg, 0.20)
    candidates, _ = PLANNER.goal_candidates(target, SPONGE.joints['pick_grasp'])
    return candidates[0], target


class TestDerivePick(unittest.TestCase):
    def test_taught_sponge_grasp_gives_straight_approach_and_lift(self):
        q = SPONGE.joints['pick_grasp']
        result, problems = derive(q, SPONGE.flange_poses['pick_grasp'])
        self.assertEqual(problems, [])
        grasp = result['flange_poses']['pick_grasp']
        approach, lift = result['flange_poses']['pick_approach'], result['flange_poses']['pick_lift']
        axis = tool_z_axis(grasp[3:])
        back = [g - a for g, a in zip(grasp[:3], approach[:3])]
        self.assertAlmostEqual(math.dist(grasp[:3], approach[:3]), 0.08, places=4)
        self.assertAlmostEqual(sum(b * x for b, x in zip(back, axis)), 0.08, places=4)   # along the tool axis
        self.assertEqual(result['lift_mode'], 'vertical')
        self.assertAlmostEqual(lift[2] - grasp[2], 0.08, places=4)
        self.assertAlmostEqual(math.dist(lift[:2], grasp[:2]), 0.0, places=4)
        for wp in ('pick_approach', 'pick_lift'):
            self.assertLess(max(abs(a - b) for a, b in zip(result['joints'][wp], q)), 1.0)
        self.assertLessEqual(max(result['deviation_m'].values()), 0.01)
        self.assertAlmostEqual(result['tool_tilt_deg'], tool_tilt_deg(SPONGE.flange_poses['pick_grasp']), places=1)

    def test_tilted_sponge_grasp_accepted(self):
        q, pose = tilted_sponge_grasp(30)
        result, problems = derive(q, pose)
        self.assertEqual(problems, [])
        self.assertAlmostEqual(result['tool_tilt_deg'], tool_tilt_deg(SPONGE.flange_poses['pick_grasp']) + 30, delta=1.0)
        self.assertEqual(result['lift_mode'], 'tool_axis')   # vertical lift out of reach at this tilt
        grasp, lift = result['flange_poses']['pick_grasp'], result['flange_poses']['pick_lift']
        self.assertGreater(lift[2] - grasp[2], 0.03)
        self.assertLessEqual(max(result['deviation_m'].values()), 0.01)

    def test_pendant_tcp_not_zero_refused(self):
        q = SPONGE.joints['pick_grasp']
        pose = list(SPONGE.flange_poses['pick_grasp'])
        axis = tool_z_axis(pose[3:])
        controller = [p + 0.15 * a for p, a in zip(pose[:3], axis)] + pose[3:]   # TCP 150 mm on the pendant
        _, problems = derive(q, controller)
        self.assertTrue(any('TCP' in p for p in problems), problems)

    def test_colliding_grasp_refused(self):
        q = list(SPONGE.joints['pick_grasp'])
        q[1] = -0.2   # shoulder almost horizontal-down: arm under the virtual floor
        _, problems = derive(q, forward(q, [0] * 6))
        self.assertTrue(problems)


class TestOverrideFile(unittest.TestCase):
    def setUp(self):
        q, pose = tilted_sponge_grasp(30)
        result, problems = derive(q, pose)
        self.assertEqual(problems, [])
        self.record = override_record('sponge', result, q, pose, back_m=0.08, lift_m=0.08)
        self.dir = Path(tempfile.mkdtemp())
        self.path = self.dir / 'sponge.json'
        self.path.write_text(json.dumps(self.record), encoding='utf-8')

    def test_roundtrip(self):
        loaded = load_grasp_override(self.path, 'sponge')
        self.assertEqual(loaded['joints']['pick_grasp'], self.record['waypoints']['pick_grasp']['joints'])

    def test_changed_joint_refused(self):
        bad = copy.deepcopy(self.record)
        bad['waypoints']['pick_grasp']['joints'][2] += 0.05
        self.path.write_text(json.dumps(bad), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'pick_grasp'):
            load_grasp_override(self.path, 'sponge')

    def test_wrong_class_or_tcp_refused(self):
        with self.assertRaisesRegex(ValueError, 'class'):
            load_grasp_override(self.path, 'paper_cup')
        bad = copy.deepcopy(self.record)
        bad['tcp_pose'] = [0, 0, 0.15, 0, 0, 0]
        self.path.write_text(json.dumps(bad), encoding='utf-8')
        with self.assertRaisesRegex(ValueError, 'TCP'):
            load_grasp_override(self.path, 'sponge')

    def test_route_uses_override_pick_and_keeps_drop(self):
        taught = apply_grasp_overrides(TAUGHT, {'sponge': str(self.path)}, root=self.dir)
        self.assertEqual(TAUGHT['sponge'].joints['pick_grasp'], SPONGE.joints['pick_grasp'])   # original untouched
        r = TaughtRoute(taught, CELL, MOTION['tcp_pose'], incident_reviewed=('aluminum_can',))
        machine, executor, _ = run(r, 'sponge')
        self.assertEqual(machine.state, 'COMPLETED')
        moves = [c for c in executor.commands if c['action'] == 'move_joint_space']
        wp = self.record['waypoints']
        targets = [m['joints'] for m in moves]
        for name in ('pick_approach', 'pick_grasp', 'pick_lift'):
            self.assertTrue(any(max(abs(a - b) for a, b in zip(t, wp[name]['joints'])) < 1e-9 for t in targets), name)
        self.assertTrue(any(max(abs(a - b) for a, b in zip(t, SPONGE.joints['drop_release'])) < 1e-9 for t in targets))

    def test_override_for_unknown_class_refused(self):
        with self.assertRaisesRegex(ValueError, 'toothbrush'):
            apply_grasp_overrides(TAUGHT, {'toothbrush': str(self.path)}, root=self.dir)


if __name__ == '__main__':
    unittest.main()
