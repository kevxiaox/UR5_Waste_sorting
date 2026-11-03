"""UR5 forward and closed-form inverse kinematics. Pure Python, 4x4 matrices are row lists.

DH parameters are Universal Robots' published UR5 values (CB3/e-Series UR5). The inverse is
the analytic solution of K. P. Hawkins, "Analytic Inverse Kinematics for the Universal Robots
UR-5/UR-10 Arms" (Georgia Tech, 2013), the one used by ROS-Industrial `ur_kinematics`: up to
eight exact solutions, no iteration, no initial guess.

Poses are UR controller poses [x, y, z, rx, ry, rz] (metres, rotation vector) of the TCP in
the controller's base frame. The TCP offset is a parameter: it must equal the TCP configured
on the controller (configs/*/motion_config.json tcp_pose; checked by the real executor).
"""
import math

from .geometry import matrix_to_rotvec, rotvec_to_matrix

UR5_DH = {'d': (0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823),
          'a': (0.0, -0.425, -0.39225, 0.0, 0.0, 0.0),
          'alpha': (math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0)}


def matmul(a, b):
    """Product of two rigid transforms (last row [0, 0, 0, 1]), unrolled for speed."""
    b0, b1, b2 = b[0], b[1], b[2]
    out = []
    for r in a[:3]:
        x, y, z = r[0], r[1], r[2]
        out.append([x * b0[0] + y * b1[0] + z * b2[0], x * b0[1] + y * b1[1] + z * b2[1],
                    x * b0[2] + y * b1[2] + z * b2[2], x * b0[3] + y * b1[3] + z * b2[3] + r[3]])
    out.append([0., 0., 0., 1.])
    return out


def invert(t):
    """Inverse of a rigid transform."""
    r = [[t[j][i] for j in range(3)] for i in range(3)]
    p = [-sum(r[i][k] * t[k][3] for k in range(3)) for i in range(3)]
    return [r[0] + [p[0]], r[1] + [p[1]], r[2] + [p[2]], [0., 0., 0., 1.]]


def pose_to_matrix(pose):
    r = rotvec_to_matrix(pose[3:])
    return [r[0] + [pose[0]], r[1] + [pose[1]], r[2] + [pose[2]], [0., 0., 0., 1.]]


def matrix_to_pose(t):
    return [t[0][3], t[1][3], t[2][3]] + matrix_to_rotvec([row[:3] for row in t[:3]])


def dh_transform(theta, d, a, alpha):
    ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
    return [[ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0., sa, ca, d],
            [0., 0., 0., 1.]]


def _a(i, theta, dh):
    return dh_transform(theta, dh['d'][i], dh['a'][i], dh['alpha'][i])


def joint_frames(q, dh=UR5_DH):
    """Frames 0..6 (base to flange) in the base frame."""
    frames = [[[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]]
    for i in range(6):
        frames.append(matmul(frames[-1], _a(i, q[i], dh)))
    return frames


def forward(q, tcp_pose, dh=UR5_DH):
    """TCP pose for joint angles q (rad)."""
    return matrix_to_pose(matmul(joint_frames(q, dh)[6], pose_to_matrix(tcp_pose)))


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def inverse(pose, tcp_pose, dh=UR5_DH, q6_singular=0.0, eps=1e-9):
    """All closed-form solutions (each in (-pi, pi]) reaching `pose`. Empty if unreachable.

    At a wrist singularity (sin q5 = 0) q6 is not determined; `q6_singular` is used.
    """
    d1, _, _, d4, d5, d6 = dh['d']
    a2, a3 = dh['a'][1], dh['a'][2]
    t06 = matmul(pose_to_matrix(pose), invert(pose_to_matrix(tcp_pose)))
    t60 = invert(t06)
    p05 = [t06[i][3] - d6 * t06[i][2] for i in range(3)]
    r05 = math.hypot(p05[0], p05[1])
    if r05 < d4:
        return []
    psi, phi = math.atan2(p05[1], p05[0]), math.acos(d4 / r05)
    solutions = []
    for q1 in (psi + phi + math.pi / 2, psi - phi + math.pi / 2):
        s1, c1 = math.sin(q1), math.cos(q1)
        c5 = (t06[0][3] * s1 - t06[1][3] * c1 - d4) / d6
        if abs(c5) > 1 + eps:
            continue
        for q5 in (math.acos(max(-1., min(1., c5))), -math.acos(max(-1., min(1., c5)))):
            s5 = math.sin(q5)
            if abs(s5) < eps:
                q6 = q6_singular
            else:
                q6 = math.atan2((-t60[1][0] * s1 + t60[1][1] * c1) / s5,
                                (t60[0][0] * s1 - t60[0][1] * c1) / s5)
            t14 = matmul(matmul(invert(_a(0, q1, dh)), t06),
                         invert(matmul(_a(4, q5, dh), _a(5, q6, dh))))
            # Wrist-1 centre seen from frame 1: planar 2-link problem for q2, q3.
            p13 = [t14[0][3] - d4 * t14[0][1], t14[1][3] - d4 * t14[1][1]]
            n = math.hypot(*p13)
            c3 = (n * n - a2 * a2 - a3 * a3) / (2 * a2 * a3)
            if abs(c3) > 1 + eps or n < eps:
                continue
            for q3 in (math.acos(max(-1., min(1., c3))), -math.acos(max(-1., min(1., c3)))):
                q2 = -math.atan2(p13[1], -p13[0]) + math.asin(max(-1., min(1., a3 * math.sin(q3) / n)))
                t13 = matmul(_a(1, q2, dh), _a(2, q3, dh))
                t34 = matmul(invert(t13), t14)
                q4 = math.atan2(t34[1][0], t34[0][0])
                solutions.append([_wrap(v) for v in (q1, q2, q3, q4, q5, q6)])
    return solutions
