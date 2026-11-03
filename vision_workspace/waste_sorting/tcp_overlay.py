"""Readonly, stationary snapshots for visual inspection of TCP hypotheses."""
import math
import time

import numpy as np

from .observations import flange_from_state
from .rtde_readonly import read_robot_state, read_tool_io_state
from .taught_poses import build_pose_record


def project_point(point, intrinsics, size):
    point = np.asarray(point, dtype=float)
    if point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0:
        return {'status': 'behind_camera_or_invalid', 'pixel': None}
    fx, fy, cx, cy = intrinsics
    if not all(math.isfinite(v) for v in intrinsics) or min(fx, fy) <= 0:
        raise ValueError('Invalid rectified LEFT intrinsics')
    u, v = fx*point[0]/point[2]+cx, fy*point[1]/point[2]+cy
    if not (0 <= u < size[0] and 0 <= v < size[1]):
        return {'status': 'outside_image', 'pixel': None}
    return {'status': 'projected_hypothesis_only', 'pixel': [float(u), float(v)]}


def axial_hypothesis(mm=None):
    """A point for display only. Axial alignment is an explicit assumption."""
    value = 195.0 if mm is None else float(mm)
    if not math.isfinite(value) or not 0 < value <= 1000:
        raise ValueError('Axial hypothesis must be finite and in (0, 1000] mm')
    return {
        'name': ('HYPOTHESIS 195mm (SPONGE GRIP ONLY)' if mm is None else
                 f'AXIAL HYPOTHESIS {value:g}mm (MOUNT AXIS ASSUMED)'),
        'axial_mm': value, 'flange_offset_xyz_m': [0., 0., value/1000],
        'flange_axis_alignment_assumed': True,
        'installed_flange_registration_verified': False,
        'controller_tcp_pose': None,
    }


def capture_overlay(camera, candidate, logo_offset, intrinsics, host='147.250.35.40',
                    axial_hypothesis_mm=None):
    hypothesis = axial_hypothesis(axial_hypothesis_mm)
    before = read_tool_io_state(host)
    pose = read_robot_state(host)
    frame_start = time.monotonic()
    frame = camera.read()
    frame_end = time.monotonic()
    after = read_tool_io_state(host)
    checked = build_pose_record('TCP_CHECK_GRASP', before, pose, after)
    if not (before['host_sample_monotonic_s'] <= frame_start <= frame_end
            <= after['host_sample_monotonic_s']):
        raise ValueError('Camera frame is outside the robot snapshot bracket')
    base_from_flange = flange_from_state(pose)
    camera_from_base = np.linalg.inv(candidate['transform'])
    offsets = {'LOGO fit': logo_offset,
               'CONFIG TCP 150mm' if np.allclose(pose['active_tcp_offset_flange_m_rotvec_rad'][:3], [0, 0, .15])
               else 'CONFIG TCP': pose['active_tcp_offset_flange_m_rotvec_rad'][:3],
               hypothesis['name']: hypothesis['flange_offset_xyz_m']}
    points = []
    h, w = frame.shape[:2]
    for name, offset in offsets.items():
        point = camera_from_base @ base_from_flange @ np.r_[offset, 1]
        points.append({'name': name, 'flange_offset_xyz_m': list(offset),
                       'candidate_camera_xyz_m': point[:3].tolist(),
                       **project_point(point[:3], intrinsics, (w, h))})
    return frame, {'type': 'frozen_readonly_tcp_hypothesis_overlay', 'points': points,
                   'axial_hypothesis': hypothesis,
                   'robot_pose_record': checked, 'frame_start_monotonic_s': frame_start,
                   'frame_end_monotonic_s': frame_end,
                   'rectified_left_intrinsics_fx_fy_cx_cy': list(intrinsics),
                   'jaw_outputs_before_after': [[s['tool_output_0'], s['tool_output_1']]
                                                for s in (before, after)],
                   'jaw_state_sensor_verified': False, 'depth_measured': False,
                   'tcp_verified': False, 'independent_physical_validation_passed': False,
                   'robot_execution_enabled': False, 'automatic_replay_enabled': False,
                   'grasp_pose': None, 'trajectory': None,
                   'note': 'Frozen image and bounded stationary endpoint checks only; no continuous motion observation. '
                           f"{hypothesis['axial_mm']:g}mm assumes flange-axis alignment and X=Y=0; "
                           'does not establish current jaw geometry or a registered CAD installation. '
                           '2D projection has no measured depth or independent accuracy claim.'}
