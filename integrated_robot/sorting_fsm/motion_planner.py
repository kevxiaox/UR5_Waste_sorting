"""Point-to-point motion planning for the UR5: current joints -> target TCP pose.

Standard pipeline (the one MoveIt runs for UR arms), using published algorithms only:
  1. Goal: closed-form UR5 IK (Hawkins 2013), every solution mapped into the joint limits,
     collision-checked, and ordered by joint-space distance from the current configuration.
  2. Path: the straight joint-space segment, which is exactly what the controller's `movej`
     executes. If it collides, RRT-Connect (Kuffner & LaValle, ICRA 2000; OMPL/MoveIt default
     planner), then random shortcutting (Geraerts & Overmars 2007).
  3. Validation: every segment is checked conservatively. Between two samples no point of the
     arm moves more than `motion_check_step_m` (bound sum |dq_i| * R_i, R_i = rigid chain
     length beyond joint i), and each sample is
     checked with capsules inflated by half that step, so a gap between samples cannot hide a
     collision with the modelled scene.
  4. Timing: synchronized trapezoidal profile per segment (leading joint at max speed and
     acceleration), the profile `movej(q, a, v)` uses; the robot stops at each waypoint so the
     executed path is the validated one (no blending).

Short approach/lift/retreat moves near the table use `plan_linear`: a straight TCP line (the
controller's `moveL`), interpolated every few millimetres with IK continuity checks (MoveIt's
`computeCartesianPath` with a jump threshold) and collision checks in which only the tool may
come close to the table.

Guarantees hold for the MODEL only (capsules, boxes, table, TCP in motion_config.json). They
say nothing about unmodelled obstacles, people, or a wrong TCP/calibration.
"""
from contextlib import contextmanager
from dataclasses import dataclass
import json
import math
from pathlib import Path
import random

from .collision import LINKS, CollisionModel, link_segments
from .contracts import validate_pose
from .geometry import rotation_between, slerp_rotvec
from .ur5_kinematics import forward, inverse, joint_frames, matmul, pose_to_matrix

CONFIG_PATH = Path(__file__).resolve().parents[1] / 'configs' / 'sim_cell' / 'motion_config.json'
GOAL_TOLERANCE_M = 1e-6


class MotionPlanningError(ValueError):
    """No safe motion found; nothing must be sent to the robot."""


@dataclass(frozen=True)
class MotionPlan:
    waypoints: list            # joint configurations (rad), waypoints[0] is the start
    segment_durations_s: list  # one per segment, trapezoidal profile
    total_duration_s: float
    method: str                # 'already_there' | 'direct' | 'rrt_connect'
    goal_pose: list | None     # TCP pose reached at the end (None for a joint-space goal)
    joint_speed_rad_s: float
    joint_acceleration_rad_s2: float
    simulation_only: bool = True


@dataclass(frozen=True)
class LinearPlan:
    start_joints: list
    end_joints: list           # configuration the controller's moveL will end in
    target_pose: list
    distance_m: float
    duration_s: float
    linear_speed_m_s: float
    linear_acceleration_m_s2: float
    simulation_only: bool = True


def load_motion_config(path=CONFIG_PATH):
    config = json.loads(Path(path).read_text(encoding='utf-8'))
    validate_motion_config(config)
    return config


def _finite(v, positive=False):
    return type(v) in (int, float) and math.isfinite(v) and (v > 0 if positive else True)


def _joints(q, label='Joint configuration'):
    if not isinstance(q, (list, tuple)) or len(q) != 6 or not all(_finite(v) for v in q):
        raise ValueError(f'{label} must be 6 finite numbers in radians')
    return [float(v) for v in q]


def validate_motion_config(c):
    if c.get('schema_version') != 1 or c.get('frame') != 'UR_BASE':
        raise ValueError('Unsupported motion config schema or frame')
    validate_pose(c.get('tcp_pose'))
    limits = c.get('joint_limits_rad')
    if (not isinstance(limits, list) or len(limits) != 6
            or not all(isinstance(l, list) and len(l) == 2 and _finite(l[0]) and _finite(l[1])
                       and l[0] < l[1] for l in limits)):
        raise ValueError('joint_limits_rad needs 6 finite [min, max] pairs')
    for key in ('max_joint_speed_rad_s', 'max_joint_acceleration_rad_s2',
                'max_linear_speed_m_s', 'max_linear_acceleration_m_s2'):
        if not _finite(c.get(key), positive=True):
            raise ValueError(f'{key} must be positive')
    col = c.get('collision', {})
    radii = col.get('link_radii_m', {})
    if not all(_finite(radii.get(name), positive=True) for name in LINKS):
        raise ValueError(f'link_radii_m needs a positive radius for each of {LINKS}')
    if not (_finite(col.get('environment_margin_m')) and col['environment_margin_m'] >= 0
            and _finite(col.get('self_margin_m')) and col['self_margin_m'] >= 0
            and _finite(col.get('table_z_m'))
            and _finite(col.get('max_held_object_length_m')) and col['max_held_object_length_m'] >= 0):
        raise ValueError('Invalid collision margins or table height')
    if not isinstance(col.get('boxes'), list):
        raise ValueError('collision.boxes must be a list')
    for box in col['boxes']:
        if (not isinstance(box, dict) or not isinstance(box.get('name'), str)
                or not all(isinstance(box.get(k), list) and len(box[k]) == 3
                           and all(_finite(v) for v in box[k]) for k in ('min', 'max'))
                or not all(lo < hi for lo, hi in zip(box['min'], box['max']))):
            raise ValueError('Each box needs a name and finite min < max corners')
    p = c.get('planner', {})
    for key in ('motion_check_step_m', 'rrt_step_rad', 'linear_step_m', 'linear_check_step_m',
                'jump_threshold_rad', 'max_linear_distance_m'):
        if not _finite(p.get(key), positive=True):
            raise ValueError(f'planner.{key} must be positive')
    if not _finite(p.get('approach_tool_table_margin_m')) or p['approach_tool_table_margin_m'] < 0:
        raise ValueError('planner.approach_tool_table_margin_m must be >= 0')
    rotation = p.get('max_linear_rotation_rad', 0.0)
    if not _finite(rotation) or not 0 <= rotation <= 0.5:
        raise ValueError('planner.max_linear_rotation_rad must be in [0, 0.5]')
    capsule = col.get('tool_capsule_flange_m')
    if capsule is not None and not (
            isinstance(capsule, dict)
            and all(isinstance(capsule.get(k), list) and len(capsule[k]) == 3
                    and all(_finite(v) for v in capsule[k]) for k in ('start', 'end'))
            and math.dist(capsule['start'], capsule['end']) > 1e-3):
        raise ValueError('collision.tool_capsule_flange_m needs distinct finite start/end [x, y, z]')
    for key in ('rrt_max_iterations', 'shortcut_iterations', 'seed'):
        if type(p.get(key)) is not int or p[key] < 0:
            raise ValueError(f'planner.{key} must be a non-negative integer')


def trapezoid_duration(distance, v, a):
    """Duration of a rest-to-rest move of `distance` with speed v and acceleration a."""
    if distance <= 0:
        return 0.0
    if distance >= v * v / a:
        return distance / v + v / a
    return 2 * math.sqrt(distance / a)


class MotionPlanner:
    def __init__(self, config=None):
        self.config = load_motion_config() if config is None else config
        validate_motion_config(self.config)
        c = self.config
        self.tcp_pose = c['tcp_pose']
        self.limits = c['joint_limits_rad']
        self.collision = CollisionModel(c['collision'], self.tcp_pose)
        p = c['planner']
        self.check_step = p['motion_check_step_m']
        self.pad = self.check_step / 2
        self.rrt_step, self.rrt_iterations = p['rrt_step_rad'], p['rrt_max_iterations']
        self.shortcut_iterations, self.seed = p['shortcut_iterations'], p['seed']
        self.v, self.a = c['max_joint_speed_rad_s'], c['max_joint_acceleration_rad_s2']
        self.v_lin, self.a_lin = c['max_linear_speed_m_s'], c['max_linear_acceleration_m_s2']
        self.linear_step, self.linear_check_step = p['linear_step_m'], p['linear_check_step_m']
        self.jump_threshold, self.max_linear = p['jump_threshold_rad'], p['max_linear_distance_m']
        self.approach_margin = p['approach_tool_table_margin_m']
        self.max_linear_rotation = p.get('max_linear_rotation_rad', 0.0)
        self._held, self._tool_margin = 0.0, None   # current checking mode, see _mode()

    @contextmanager
    def _mode(self, *, held_object_m=0.0, tool_table_margin=None, check_step=None):
        """Temporarily check with a held object / approach margin / finer motion step."""
        if not _finite(held_object_m) or not 0 <= held_object_m <= self.collision.max_held_object_m:
            raise ValueError('held_object_m must be within [0, max_held_object_length_m]')
        saved = self._held, self._tool_margin, self.check_step, self.pad
        self._held, self._tool_margin = float(held_object_m), tool_table_margin
        if check_step is not None:
            self.check_step, self.pad = check_step, check_step / 2
        try:
            yield
        finally:
            self._held, self._tool_margin, self.check_step, self.pad = saved

    # --- validity -----------------------------------------------------------------------
    def state_problem(self, q):
        """None if q is within limits and collision-free (with the motion-check padding)."""
        for i, (value, (lo, hi)) in enumerate(zip(q, self.limits)):
            if not lo <= value <= hi:
                return f'joint {i + 1} = {value:.3f} rad outside limits [{lo}, {hi}]'
        return self.collision.first_collision(q, self.pad, self._held, self._tool_margin)

    def segment_problem(self, qa, qb):
        """None if the straight joint-space motion qa -> qb is collision-free (conservative)."""
        # Upper bound on how far any point of the robot travels along the segment.
        swept = sum(abs(b - a) * r for a, b, r in zip(qa, qb, self.collision.joint_reach))
        n = max(1, math.ceil(swept / self.check_step))
        for k in range(1, n + 1):
            q = [a + (b - a) * k / n for a, b in zip(qa, qb)]
            problem = self.state_problem(q)
            if problem is not None:
                return problem
        return None

    def path_problem(self, path):
        problem = self.state_problem(path[0])
        for qa, qb in zip(path, path[1:]):
            problem = problem or self.segment_problem(qa, qb)
        return problem

    # --- goals --------------------------------------------------------------------------
    def _closest_in_limits(self, q_ik, q_ref):
        """Equivalent angle (q + 2k pi) inside the limits, closest to the reference joint."""
        out = []
        for value, ref, (lo, hi) in zip(q_ik, q_ref, self.limits):
            options = [value + 2 * math.pi * k for k in range(-2, 3)
                       if lo <= value + 2 * math.pi * k <= hi]
            if not options:
                return None
            out.append(min(options, key=lambda v: abs(v - ref)))
        return out

    def goal_candidates(self, target_pose, q_ref):
        """Valid IK solutions for target_pose, nearest (joint space) first, with rejections."""
        candidates, rejected = [], []
        for q in inverse(target_pose, self.tcp_pose, q6_singular=q_ref[5]):
            q = self._closest_in_limits(q, q_ref)
            if q is None:
                rejected.append('IK solution outside joint limits')
                continue
            problem = self.state_problem(q)
            if problem is not None:
                rejected.append(problem)
                continue
            if not _pose_matches(forward(q, self.tcp_pose), target_pose):
                rejected.append('IK solution does not reproduce the target pose')
                continue
            candidates.append(q)
        candidates.sort(key=lambda q: math.dist(q, q_ref))
        return candidates, rejected

    # --- planning -----------------------------------------------------------------------
    def plan(self, current_joints, target_pose, *, held_object_m=0.0, reference_joints=None):
        """Plan current joint configuration -> target TCP pose. Raises MotionPlanningError.

        `held_object_m`: length of the object hanging below the TCP during the move.
        `reference_joints`: prefer the IK solution nearest to these (e.g. a taught posture, to
        stay on its arm configuration) instead of nearest to the current joints.
        """
        start = _joints(current_joints, 'Current joint configuration')
        target = validate_pose(target_pose)
        reference = start if reference_joints is None else _joints(reference_joints, 'Reference joints')
        with self._mode(held_object_m=held_object_m):
            return self._plan_pose(start, target, reference)

    def _plan_pose(self, start, target, reference):
        self._check_start(start)
        goals, rejected = self.goal_candidates(target, reference)
        if not goals:
            detail = '; '.join(sorted(set(rejected))) or 'pose out of reach'
            raise MotionPlanningError(f'No valid IK solution for the target pose ({detail})')
        failures = []
        for goal in goals:
            try:
                return self._plan_joints(start, goal, target)
            except MotionPlanningError as exc:
                failures.append(str(exc))
        raise MotionPlanningError('No collision-free path to any IK solution: ' + ' | '.join(failures))

    def plan_to_joints(self, current_joints, goal_joints, *, held_object_m=0.0):
        start = _joints(current_joints, 'Current joint configuration')
        goal = _joints(goal_joints, 'Goal joint configuration')
        with self._mode(held_object_m=held_object_m):
            self._check_start(start)
            problem = self.state_problem(goal)
            if problem is not None:
                raise MotionPlanningError(f'Goal configuration invalid: {problem}')
            return self._plan_joints(start, goal, None)

    def plan_linear(self, current_joints, target_pose):
        """Straight TCP line (moveL) to target_pose, near the table allowed.

        The orientation is kept, or, up to planner.max_linear_rotation_rad (default 0), rotated
        along the shortest rotation as moveL does (taught endpoints differ by a few degrees).
        Only the tool may approach the table (approach_tool_table_margin_m); a held object is
        treated as part of that contact zone. Raises MotionPlanningError.
        """
        start = _joints(current_joints, 'Current joint configuration')
        target = validate_pose(target_pose)
        start_pose = forward(start, self.tcp_pose)
        start_m, target_m = pose_to_matrix(start_pose), pose_to_matrix(target)
        rotate = any(abs(start_m[i][j] - target_m[i][j]) > 1e-4 for i in range(3) for j in range(3))
        if rotate and rotation_between(start_pose[3:], target[3:]) > self.max_linear_rotation:
            raise MotionPlanningError('Linear moves keep the current tool orientation '
                                      '(beyond planner.max_linear_rotation_rad)')
        a = [start_m[i][3] for i in range(3)]
        b = [target_m[i][3] for i in range(3)]
        distance = math.dist(a, b)
        if distance > self.max_linear:
            raise MotionPlanningError(f'Linear move of {distance:.3f} m exceeds max_linear_distance_m')
        with self._mode(tool_table_margin=self.approach_margin, check_step=self.linear_check_step):
            self._check_start(start)
            q = start
            n = max(1, math.ceil(distance / self.linear_step))
            for k in range(1, n + 1):
                rotation = slerp_rotvec(start_pose[3:], target[3:], k / n) if rotate and k < n else target[3:]
                pose = [a[i] + (b[i] - a[i]) * k / n for i in range(3)] + list(rotation)
                options = [c for c in (self._closest_in_limits(s, q) for s in
                                       inverse(pose, self.tcp_pose, q6_singular=q[5])) if c]
                if not options:
                    raise MotionPlanningError(f'Linear path leaves the reachable space at step {k}/{n}')
                nxt = min(options, key=lambda c: math.dist(c, q))
                if max(abs(x - y) for x, y in zip(nxt, q)) > self.jump_threshold:
                    raise MotionPlanningError(f'Joint jump (singularity/flip) on the linear path at step {k}/{n}')
                problem = self.state_problem(nxt) or self.segment_problem(q, nxt)
                if problem is not None:
                    raise MotionPlanningError(f'Linear path blocked at step {k}/{n}: {problem}')
                q = nxt
        if not _pose_matches(forward(q, self.tcp_pose), target):
            raise MotionPlanningError('Linear path does not end on the target pose')
        return LinearPlan(list(start), q, list(target), distance,
                          trapezoid_duration(distance, self.v_lin, self.a_lin), self.v_lin,
                          self.a_lin, self.config.get('simulation_only', True))

    def joint_path_deviation(self, qa, qb, samples=50):
        """Largest distance (m) of the TCP and of the tool tip from their straight line during
        the joint-space move qa -> qb (what moveJ executes), sampled along the move."""
        def points(q):
            frames = joint_frames(q)
            tcp = matmul(frames[6], pose_to_matrix(self.tcp_pose))
            out = [[tcp[i][3] for i in range(3)]]
            if self.collision.tool_capsule is not None:
                out.append(link_segments(q, self.tcp_pose, tool_capsule=self.collision.tool_capsule)['tool'][1])
            return out
        first, last = points(qa), points(qb)
        worst = 0.0
        for k in range(1, samples):
            current = points([a + (b - a) * k / samples for a, b in zip(qa, qb)])
            for p, a, b in zip(current, first, last):
                worst = max(worst, _distance_to_segment(p, a, b))
        return worst

    def _check_start(self, start):
        problem = self.state_problem(start)
        if problem is not None:
            raise MotionPlanningError(f'Start configuration invalid, refusing to plan: {problem}')

    def _plan_joints(self, start, goal, goal_pose):
        if max(abs(a - b) for a, b in zip(start, goal)) < 1e-9:
            return self._finish([start], 'already_there', goal_pose)
        if self.segment_problem(start, goal) is None:
            return self._finish([start, goal], 'direct', goal_pose)
        rng = random.Random(self.seed)
        path = self._rrt_connect(start, goal, rng)
        if path is None:
            raise MotionPlanningError(f'RRT-Connect found no path within {self.rrt_iterations} iterations')
        return self._finish(self._shortcut(path, rng), 'rrt_connect', goal_pose)

    def _finish(self, path, method, goal_pose):
        # Independent re-validation of the final path before it can be returned.
        problem = self.path_problem(path)
        if problem is not None:
            raise MotionPlanningError(f'Final path failed validation: {problem}')
        if goal_pose is not None and not _pose_matches(forward(path[-1], self.tcp_pose), goal_pose):
            raise MotionPlanningError('Final configuration does not reach the target pose')
        durations = [trapezoid_duration(max(abs(b - a) for a, b in zip(qa, qb)), self.v, self.a)
                     for qa, qb in zip(path, path[1:])]
        return MotionPlan([list(q) for q in path], durations, sum(durations), method,
                          None if goal_pose is None else list(goal_pose), self.v, self.a,
                          self.config.get('simulation_only', True))

    # --- RRT-Connect --------------------------------------------------------------------
    def _sample(self, rng):
        return [rng.uniform(lo, hi) for lo, hi in self.limits]

    def _extend(self, tree, target):
        """One step of at most rrt_step from the nearest node towards target."""
        nodes, parents = tree
        i = min(range(len(nodes)), key=lambda k: math.dist(nodes[k], target))
        near = nodes[i]
        d = math.dist(near, target)
        new = target if d <= self.rrt_step else [n + (t - n) * self.rrt_step / d
                                                 for n, t in zip(near, target)]
        if self.state_problem(new) is not None or self.segment_problem(near, new) is not None:
            return 'trapped', None
        nodes.append(new)
        parents.append(i)
        return ('reached' if new is target else 'advanced'), len(nodes) - 1

    def _connect(self, tree, target):
        while True:
            status, index = self._extend(tree, target)
            if status != 'advanced':
                return status, index

    @staticmethod
    def _branch(tree, index):
        nodes, parents = tree
        out = []
        while index is not None:
            out.append(nodes[index])
            index = parents[index]
        return out  # from index back to the root

    def _rrt_connect(self, start, goal, rng):
        tree_a, tree_b = ([start], [None]), ([goal], [None])
        a_is_start = True
        for _ in range(self.rrt_iterations):
            status, ia = self._extend(tree_a, self._sample(rng))
            if status != 'trapped':
                status, ib = self._connect(tree_b, tree_a[0][ia])
                if status == 'reached':
                    branch_a = self._branch(tree_a, ia)[::-1]
                    branch_b = self._branch(tree_b, ib)[1:]
                    path = branch_a + branch_b
                    return path if a_is_start else path[::-1]
            tree_a, tree_b, a_is_start = tree_b, tree_a, not a_is_start
        return None

    def _shortcut(self, path, rng):
        path = list(path)
        for _ in range(self.shortcut_iterations):
            if len(path) <= 2:
                break
            i, j = sorted(rng.sample(range(len(path)), 2))
            if j - i > 1 and self.segment_problem(path[i], path[j]) is None:
                path = path[:i + 1] + path[j:]
        return path


def _distance_to_segment(p, a, b):
    d = [b[i] - a[i] for i in range(3)]
    length2 = sum(v * v for v in d)
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, sum((p[i] - a[i]) * d[i] for i in range(3)) / length2))
    return math.dist(p, [a[i] + t * d[i] for i in range(3)])


def _pose_matches(pose_a, pose_b, tol=GOAL_TOLERANCE_M):
    a, b = pose_to_matrix(pose_a), pose_to_matrix(pose_b)
    return all(abs(a[i][j] - b[i][j]) <= tol for i in range(3) for j in range(4))
