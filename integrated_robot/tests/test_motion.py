import copy
import math
import random
import unittest

from sorting_fsm.collision import segment_distance
from sorting_fsm.contracts import Action
from sorting_fsm.mock_executor import MockExecutor
from sorting_fsm.motion_planner import (MotionPlanner, MotionPlanningError, load_motion_config,
                                        trapezoid_duration, validate_motion_config)
from sorting_fsm.ur5_kinematics import forward, inverse, pose_to_matrix

START = [0.5, -1.2, 1.5, -1.9, -1.57, 0.0]
FLANGE = [0.0] * 6


def matrix_error(pose_a, pose_b):
    a, b = pose_to_matrix(pose_a), pose_to_matrix(pose_b)
    return max(abs(a[i][j] - b[i][j]) for i in range(3) for j in range(4))


def angle_gap(qa, qb):
    return max(abs(math.atan2(math.sin(a - b), math.cos(a - b))) for a, b in zip(qa, qb))


class KinematicsTests(unittest.TestCase):
    def test_zero_configuration_matches_ur5_datasheet(self):
        # UR5 at all-zero joints: flange at (a2 + a3, -(d4 + d6), d1 - d5).
        pose = forward([0.0] * 6, FLANGE)
        for got, want in zip(pose[:3], (-0.81725, -0.19145, -0.005491)):
            self.assertAlmostEqual(got, want, places=6)

    def test_inverse_reproduces_pose_and_finds_original(self):
        rng = random.Random(7)
        tcp = [0.0, 0.0, 0.15, 0.0, 0.0, 0.0]
        for _ in range(300):
            q = [rng.uniform(-math.pi, math.pi) for _ in range(6)]
            pose = forward(q, tcp)
            solutions = inverse(pose, tcp)
            self.assertTrue(solutions)
            for s in solutions:
                self.assertLess(matrix_error(forward(s, tcp), pose), 1e-6)
            self.assertTrue(any(angle_gap(s, q) < 1e-6 for s in solutions))

    def test_unreachable_pose_has_no_solution(self):
        self.assertEqual(inverse([2.0, 0.0, 0.3, 3.14159, 0.0, 0.0], FLANGE), [])


class CollisionPrimitiveTests(unittest.TestCase):
    def test_segment_distance(self):
        self.assertAlmostEqual(segment_distance([0, 0, 0], [1, 0, 0], [0, 1, 0], [1, 1, 0]), 1.0)
        self.assertAlmostEqual(segment_distance([-1, 0, 0], [1, 0, 0], [0, -1, 0.5], [0, 1, 0.5]), 0.5)
        self.assertAlmostEqual(segment_distance([0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0]), 1.0)
        self.assertAlmostEqual(segment_distance([0, 0, 0], [0, 0, 0], [0, 2, 0], [0, 2, 0]), 2.0)


class PlannerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_motion_config()
        self.planner = MotionPlanner(self.config)
        self.tcp = self.config['tcp_pose']

    def target(self, q):
        return forward(q, self.tcp)

    def assert_safe_plan(self, plan, planner, target_pose):
        self.assertEqual(plan.waypoints[0], START)
        self.assertLess(matrix_error(forward(plan.waypoints[-1], self.tcp), target_pose), 1e-6)
        for q in plan.waypoints:
            for value, (lo, hi) in zip(q, self.config['joint_limits_rad']):
                self.assertTrue(lo <= value <= hi)
        self.assertIsNone(planner.path_problem(plan.waypoints))
        # Independent dense resampling, without the conservative padding.
        for qa, qb in zip(plan.waypoints, plan.waypoints[1:]):
            for k in range(201):
                q = [a + (b - a) * k / 200 for a, b in zip(qa, qb)]
                self.assertIsNone(planner.collision.first_collision(q))
        self.assertEqual(len(plan.segment_durations_s), len(plan.waypoints) - 1)
        self.assertAlmostEqual(plan.total_duration_s, sum(plan.segment_durations_s))
        self.assertTrue(plan.simulation_only)

    def test_clear_motion_is_a_single_joint_space_segment(self):
        target = self.target([-0.4, -1.3, 1.6, -1.9, -1.57, 0.0])
        plan = self.planner.plan(START, target)
        self.assertEqual(plan.method, 'direct')
        self.assertEqual(len(plan.waypoints), 2)
        self.assert_safe_plan(plan, self.planner, target)

    def test_nearest_ik_solution_is_chosen(self):
        goal = [-0.4, -1.3, 1.6, -1.9, -1.57, 0.0]
        plan = self.planner.plan(START, self.target(goal))
        self.assertLess(max(abs(a - b) for a, b in zip(plan.waypoints[-1], goal)), 1e-6)

    def test_obstacle_on_straight_path_is_avoided_with_rrt_connect(self):
        target = self.target([-1.6, -1.2, 1.5, -1.9, -1.57, 0.0])
        self.assertIsNotNone(self.planner.segment_problem(START, [-1.6, -1.2, 1.5, -1.9, -1.57, 0.0]))
        plan = self.planner.plan(START, target)
        self.assertEqual(plan.method, 'rrt_connect')
        self.assertGreater(len(plan.waypoints), 2)
        self.assert_safe_plan(plan, self.planner, target)

    def test_planning_is_deterministic(self):
        target = self.target([-1.6, -1.2, 1.5, -1.9, -1.57, 0.0])
        self.assertEqual(self.planner.plan(START, target), MotionPlanner(self.config).plan(START, target))

    def test_already_at_target(self):
        plan = self.planner.plan(START, self.target(START))
        self.assertEqual((plan.method, plan.total_duration_s), ('already_there', 0.0))

    def test_unreachable_target_is_refused(self):
        with self.assertRaisesRegex(MotionPlanningError, 'No valid IK'):
            self.planner.plan(START, [1.5, 0.0, 0.3, 3.14159, 0.0, 0.0])

    def test_target_below_table_is_refused(self):
        pose = self.target(START)
        pose[2] = -0.05
        with self.assertRaisesRegex(MotionPlanningError, 'table'):
            self.planner.plan(START, pose)

    def test_target_inside_obstacle_is_refused(self):
        with self.assertRaisesRegex(MotionPlanningError, 'camera_mast'):
            self.planner.plan(START, [-0.40, 0.40, 0.40, 3.14159, 0.0, 0.0])

    def test_start_in_collision_is_refused(self):
        with self.assertRaisesRegex(MotionPlanningError, 'Start configuration invalid'):
            self.planner.plan([0.0, -0.3, 2.6, -2.3, -1.57, 0.0], self.target(START))

    def test_start_outside_joint_limits_is_refused(self):
        with self.assertRaisesRegex(MotionPlanningError, 'outside limits'):
            self.planner.plan([0.5, 0.3, 1.5, -1.9, -1.57, 0.0], self.target(START))

    def test_no_path_found_raises_instead_of_returning_unsafe_path(self):
        config = copy.deepcopy(self.config)
        config['planner']['rrt_max_iterations'] = 0
        with self.assertRaisesRegex(MotionPlanningError, 'RRT-Connect found no path'):
            MotionPlanner(config).plan_to_joints(START, [-1.6, -1.2, 1.5, -1.9, -1.57, 0.0])

    def test_invalid_inputs(self):
        for bad in ([0.0] * 5, [0.0] * 5 + [math.nan], 'q'):
            with self.assertRaises(ValueError):
                self.planner.plan(bad, self.target(START))
        with self.assertRaises(ValueError):
            self.planner.plan(START, [0.0, 0.0, math.inf, 0.0, 0.0, 0.0])

    def test_plan_to_joints(self):
        plan = self.planner.plan_to_joints(START, [-0.4, -1.3, 1.6, -1.9, -1.57, 0.3])
        self.assertEqual(plan.method, 'direct')
        self.assertIsNone(plan.goal_pose)

    def test_random_queries_never_return_an_unsafe_path(self):
        rng = random.Random(3)
        config = copy.deepcopy(self.config)
        config['planner']['rrt_max_iterations'] = 300
        planner = MotionPlanner(config)
        planned = 0
        for _ in range(15):
            goal = [rng.uniform(-math.pi, math.pi), rng.uniform(-2.0, -0.8), rng.uniform(0.5, 2.2),
                    rng.uniform(-2.5, -1.0), rng.uniform(-2.0, -1.0), rng.uniform(-1.0, 1.0)]
            try:
                plan = planner.plan(START, self.target(goal))
            except MotionPlanningError:
                continue
            planned += 1
            self.assert_safe_plan(plan, planner, self.target(goal))
        self.assertGreater(planned, 5)


class TimingAndConfigTests(unittest.TestCase):
    def test_trapezoid_duration(self):
        self.assertEqual(trapezoid_duration(0.0, 1.0, 1.0), 0.0)
        self.assertAlmostEqual(trapezoid_duration(2.0, 1.0, 1.0), 3.0)   # cruise phase
        self.assertAlmostEqual(trapezoid_duration(0.25, 1.0, 1.0), 1.0)  # triangle profile

    def test_config_validation(self):
        good = load_motion_config()
        mutations = [
            lambda c: c.__setitem__('frame', 'TOOL'),
            lambda c: c['joint_limits_rad'].__setitem__(0, [1.0, -1.0]),
            lambda c: c.__setitem__('max_joint_speed_rad_s', 0),
            lambda c: c['collision']['link_radii_m'].pop('tool'),
            lambda c: c['collision']['boxes'].append({'name': 'b', 'min': [0, 0, 0], 'max': [0, 1, 1]}),
            lambda c: c['planner'].__setitem__('motion_check_step_m', math.nan),
        ]
        for mutate in mutations:
            c = copy.deepcopy(good)
            mutate(c)
            with self.assertRaises(ValueError):
                validate_motion_config(c)


class MockJointMoveTests(unittest.TestCase):
    def test_joint_move_updates_simulated_joints_only_on_completion(self):
        now = [0.0]
        ex = MockExecutor(clock=lambda: now[0], delay_s=1.0)
        goal = [-0.4, -1.3, 1.6, -1.9, -1.57, 0.0]
        cid = ex.submit('t1', Action.MOVE_JOINTS, joints=goal)
        self.assertEqual(ex.poll(cid).status, 'running')
        self.assertNotEqual(ex.current_joint_positions(), goal)
        now[0] = 1.0
        self.assertEqual(ex.poll(cid).status, 'completed')
        self.assertEqual(ex.current_joint_positions(), goal)

    def test_joint_argument_only_on_joint_moves(self):
        ex = MockExecutor(clock=lambda: 0.0)
        with self.assertRaises(ValueError):
            ex.submit('t1', Action.OPEN, joints=START)
        with self.assertRaises(ValueError):
            ex.submit('t1', Action.MOVE_JOINTS, joints=[0.0] * 5)


if __name__ == '__main__':
    unittest.main()
