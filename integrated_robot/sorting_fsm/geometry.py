"""Pure-Python pose math in the UR base frame. No numpy; 3x3 matrices are row lists."""
import math


def _norm(v):
    return math.sqrt(sum(c * c for c in v))


def _unit(v):
    n = _norm(v)
    if n < 1e-9:
        raise ValueError('Zero-length direction')
    return [c / n for c in v]


def _cross(a, b):
    return [a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]]


def matrix_from_axes(x, y, z):
    """Rotation whose columns are the tool x, y, z axes expressed in the base frame."""
    return [[x[i], y[i], z[i]] for i in range(3)]


def tool_z_axis(rotvec):
    return [row[2] for row in rotvec_to_matrix(rotvec)]


def rotvec_to_matrix(rotvec):
    angle = _norm(rotvec)
    if angle < 1e-12:
        return [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    kx, ky, kz = (c / angle for c in rotvec)
    c, s, t = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[t*kx*kx + c, t*kx*ky - s*kz, t*kx*kz + s*ky],
            [t*kx*ky + s*kz, t*ky*ky + c, t*ky*kz - s*kx],
            [t*kx*kz - s*ky, t*ky*kz + s*kx, t*kz*kz + c]]


def matrix_to_rotvec(m):
    """Inverse Rodrigues, robust at 180 degrees (tool pointing straight down)."""
    cos_angle = max(-1., min(1., (m[0][0] + m[1][1] + m[2][2] - 1) / 2))
    antisym = [m[2][1] - m[1][2], m[0][2] - m[2][0], m[1][0] - m[0][1]]
    angle = math.atan2(_norm(antisym) / 2, cos_angle)  # well conditioned, unlike acos near pi
    if angle < 1e-9:
        return [0., 0., 0.]
    if math.pi - angle > 1e-2:
        s = 2 * math.sin(angle)
        axis = [c / s for c in antisym]
    else:
        # Near pi the antisymmetric part is tiny (dividing by it loses precision); recover the
        # axis from the symmetric part, its sign from the antisymmetric part.
        i = max(range(3), key=lambda k: m[k][k])
        axis = [0., 0., 0.]
        axis[i] = math.sqrt(max(0., (m[i][i] - cos_angle) / (1 - cos_angle)))
        for j in range(3):
            if j != i:
                axis[j] = (m[i][j] + m[j][i]) / (2 * (1 - cos_angle) * axis[i])
        axis = _unit(axis)
        if sum(a * b for a, b in zip(axis, antisym)) < 0:
            axis = [-a for a in axis]
    return [a * angle for a in axis]


def _matmul3(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _transpose3(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def rotation_between(rotvec_a, rotvec_b):
    """Angle (rad) of the rotation taking orientation a to orientation b."""
    return _norm(matrix_to_rotvec(_matmul3(_transpose3(rotvec_to_matrix(rotvec_a)),
                                           rotvec_to_matrix(rotvec_b))))


def slerp_rotvec(rotvec_a, rotvec_b, t):
    """Orientation a fraction t of the way from a to b along the shortest rotation (the
    interpolation the UR controller's moveL applies to the tool orientation)."""
    ra = rotvec_to_matrix(rotvec_a)
    relative = matrix_to_rotvec(_matmul3(_transpose3(ra), rotvec_to_matrix(rotvec_b)))
    return matrix_to_rotvec(_matmul3(ra, rotvec_to_matrix([c * t for c in relative])))


def _rotate_about(v, axis, angle):
    """Rodrigues rotation of v about a unit axis."""
    c, s = math.cos(angle), math.sin(angle)
    k_cross_v = _cross(axis, v)
    k_dot_v = sum(a * b for a, b in zip(axis, v))
    return [v[i]*c + k_cross_v[i]*s + axis[i]*k_dot_v*(1 - c) for i in range(3)]


def top_down_rotvec(yaw_rad):
    """Tool z points down; tool x lies horizontal at `yaw_rad` from base +X."""
    z = [0., 0., -1.]
    x = [math.cos(yaw_rad), math.sin(yaw_rad), 0.]
    return matrix_to_rotvec(matrix_from_axes(x, _cross(z, x), z))


def side_rotvec(direction_xy, roll_rad):
    """Tool z along the horizontal approach direction; tool x starts pointing down, then rolls."""
    z = _unit([direction_xy[0], direction_xy[1], 0.])
    x = _rotate_about([0., 0., -1.], z, roll_rad)
    return matrix_to_rotvec(matrix_from_axes(x, _cross(z, x), z))


def horizontal_direction(spec, object_xyz):
    """'radial' approaches away from the robot base; [dx, dy] is an explicit base-frame direction."""
    if spec == 'radial':
        return _unit([object_xyz[0], object_xyz[1]])
    return _unit(list(spec))
