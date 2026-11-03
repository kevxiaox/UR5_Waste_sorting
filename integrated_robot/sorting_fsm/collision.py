"""Capsule model of the UR5 + gripper, checked against the table, boxes and itself.

Each link is a segment swept by a sphere (capsule). Segments follow the real UR5 geometry:
DH frame origins plus the shoulder/elbow lateral offsets of UR's UR5 description
(0.13585 m and 0.1197 m along the joint-2 axis). Radii, margins and boxes come from config.
The tool capsule ends so that its rounded tip is exactly at the TCP (fingertips), or further
down by the length of a held object. When the controller TCP is not at the fingertips (e.g. the
taught TCP = 0 references), `tool_capsule_flange_m` gives the gripper axis in the flange frame
instead; a held object then extends it beyond its end.
"""
import math

from .ur5_kinematics import UR5_DH, joint_frames, matmul, pose_to_matrix

SHOULDER_OFFSET_M = 0.13585
ELBOW_OFFSET_M = 0.1197
LINKS = ('base', 'shoulder', 'upper_arm', 'elbow', 'forearm', 'wrist_1', 'wrist_2', 'wrist_3', 'tool')
FIXED_LINKS = ('base', 'shoulder')  # never move vertically: excluded from the table check
# The upper arm starts on the shoulder axis, whose height is fixed by the mounting (d1): only
# its distal end can approach the table.
TABLE_CHECK_DISTAL_ONLY = ('upper_arm',)
# Self-collision is checked between links at least this far apart in the chain; closer pairs
# touch by construction at their shared joint.
SELF_PAIR_MIN_GAP = 3


def _add(p, v, s=1.0):
    return [p[i] + s * v[i] for i in range(3)]


def _transform_point(t, p):
    return [sum(t[i][k] * p[k] for k in range(3)) + t[i][3] for i in range(3)]


def link_segments(q, tcp_pose, dh=UR5_DH, tool_end_offset=0.0, tool_capsule=None):
    """{link name: (start point, end point)} in the base frame.

    The tool segment goes from the flange to TCP + tool_end_offset * (tool z axis), or, with
    `tool_capsule` ({'start', 'end'} in the flange frame), from start to end extended by
    tool_end_offset along start -> end.
    """
    f = joint_frames(q, dh)
    o = [[m[0][3], m[1][3], m[2][3]] for m in f]
    z1 = [f[1][0][2], f[1][1][2], f[1][2][2]]
    tcp = matmul(f[6], pose_to_matrix(tcp_pose))
    inner = SHOULDER_OFFSET_M - ELBOW_OFFSET_M
    return {
        'base': (o[0], o[1]),
        'shoulder': (o[1], _add(o[1], z1, SHOULDER_OFFSET_M)),
        'upper_arm': (_add(o[1], z1, SHOULDER_OFFSET_M), _add(o[2], z1, SHOULDER_OFFSET_M)),
        'elbow': (_add(o[2], z1, SHOULDER_OFFSET_M), _add(o[2], z1, inner)),
        'forearm': (_add(o[2], z1, inner), _add(o[3], z1, inner)),
        'wrist_1': (_add(o[3], z1, inner), o[4]),
        'wrist_2': (o[4], o[5]),
        'wrist_3': (o[5], o[6]),
        'tool': _tool_segment(f[6], tcp, tool_end_offset, tool_capsule),
    }


def _tool_segment(flange, tcp, tool_end_offset, tool_capsule):
    if tool_capsule is None:
        return ([flange[i][3] for i in range(3)],
                [tcp[i][3] + tool_end_offset * tcp[i][2] for i in range(3)])
    start = _transform_point(flange, tool_capsule['start'])
    end = _transform_point(flange, tool_capsule['end'])
    length = math.dist(start, end)
    return start, [end[i] + tool_end_offset * (end[i] - start[i]) / length for i in range(3)]


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def segment_distance(p1, q1, p2, q2):
    """Minimum distance between segments p1q1 and p2q2 (Ericson, Real-Time Collision Detection 5.1.9)."""
    d1, d2 = [q1[i] - p1[i] for i in range(3)], [q2[i] - p2[i] for i in range(3)]
    r = [p1[i] - p2[i] for i in range(3)]
    a, e, f = _dot(d1, d1), _dot(d2, d2), _dot(d2, r)
    eps = 1e-12
    if a <= eps and e <= eps:
        s = t = 0.0
    elif a <= eps:
        s, t = 0.0, max(0.0, min(1.0, f / e))
    else:
        c = _dot(d1, r)
        if e <= eps:
            t, s = 0.0, max(0.0, min(1.0, -c / a))
        else:
            b = _dot(d1, d2)
            denom = a * e - b * b
            s = max(0.0, min(1.0, (b * f - c * e) / denom)) if denom > eps else 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t, s = 0.0, max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t, s = 1.0, max(0.0, min(1.0, (b - c) / a))
    c1, c2 = _add(p1, d1, s), _add(p2, d2, t)
    return math.dist(c1, c2)


def point_box_distance(p, lo, hi):
    return math.sqrt(sum(max(lo[i] - p[i], 0.0, p[i] - hi[i]) ** 2 for i in range(3)))


def segment_box_clear(p, q, lo, hi, clearance, sample_step=0.01):
    """True if every point of segment pq is at least `clearance` from the box.

    Point-to-box distance is 1-Lipschitz along the segment, so requiring clearance + h/2 at
    samples h apart is conservative (never reports a colliding segment as clear).
    """
    mid = [(p[i] + q[i]) / 2 for i in range(3)]
    if point_box_distance(mid, lo, hi) - math.dist(p, q) / 2 >= clearance:
        return True  # the whole segment lies in a ball that clears the box
    n = max(1, math.ceil(math.dist(p, q) / sample_step))
    h = math.dist(p, q) / n
    for k in range(n + 1):
        point = [p[i] + (q[i] - p[i]) * k / n for i in range(3)]
        if point_box_distance(point, lo, hi) < clearance + h / 2:
            return False
    return True


# First link moved by each joint (links before it lie on that joint's axis or are not moved).
FIRST_MOVED_LINK = ('shoulder', 'shoulder', 'elbow', 'wrist_1', 'wrist_2', 'wrist_3')


class CollisionModel:
    def __init__(self, collision_config, tcp_pose, dh=UR5_DH):
        self.radii = collision_config['link_radii_m']
        self.env_margin = collision_config['environment_margin_m']
        self.self_margin = collision_config['self_margin_m']
        self.table_z = collision_config['table_z_m']
        self.boxes = collision_config['boxes']
        self.tcp_pose, self.dh = tcp_pose, dh
        self.tool_capsule = collision_config.get('tool_capsule_flange_m')
        self.self_pairs = [(LINKS[i], LINKS[j]) for i in range(len(LINKS))
                           for j in range(i + SELF_PAIR_MIN_GAP, len(LINKS))]
        # Bound on the distance from joint i's axis to any point of the robot it moves: chain
        # length from the joint onwards plus the largest radius. Link lengths are rigid, so this
        # holds for every configuration.
        # Computed with the longest tool (holding an object) so it bounds every mode.
        self.max_held_object_m = collision_config.get('max_held_object_length_m', 0.0)
        seg = link_segments([0.0] * 6, tcp_pose, dh, self._tool_end_offset(self.max_held_object_m),
                            self.tool_capsule)
        lengths = [math.dist(*seg[name]) for name in LINKS]
        widest = max(self.radii.values())
        self.joint_reach = [sum(lengths[LINKS.index(first):]) + widest for first in FIRST_MOVED_LINK]

    def _tool_end_offset(self, held_object_m):
        # Capsule tip at TCP (+ held object): the segment stops one radius short of it.
        # An explicit tool capsule already gives the axis end.
        if self.tool_capsule is not None:
            return held_object_m
        return held_object_m - self.radii['tool']

    def first_collision(self, q, pad=0.0, held_object_m=0.0, tool_table_margin=None):
        """None if q is collision-free with every capsule inflated by `pad`, else a reason.

        `held_object_m`: length of an object hanging below the TCP (added to the tool).
        `tool_table_margin`: smaller tool/table margin for approach and lift near the table.
        """
        if not 0.0 <= held_object_m <= self.max_held_object_m:
            raise ValueError('held object longer than max_held_object_length_m')
        seg = link_segments(q, self.tcp_pose, self.dh, self._tool_end_offset(held_object_m),
                            self.tool_capsule)
        for name, (p, e) in seg.items():
            r = self.radii[name] + pad
            low = e[2] if name in TABLE_CHECK_DISTAL_ONLY else min(p[2], e[2])
            margin = self.env_margin
            if name == 'tool' and tool_table_margin is not None:
                margin = tool_table_margin
            if name not in FIXED_LINKS and low - r < self.table_z + margin:
                return f'{name} too close to the table'
            for box in self.boxes:
                if not segment_box_clear(p, e, box['min'], box['max'], r + self.env_margin):
                    return f"{name} too close to {box['name']}"
        for a, b in self.self_pairs:
            limit = self.radii[a] + self.radii[b] + 2 * pad + self.self_margin
            if segment_distance(*seg[a], *seg[b]) < limit:
                return f'self-collision {a} / {b}'
        return None
