"""Candidate camera/robot associations; no external calibration or grasp targets."""
import math

import cv2
import numpy as np


def pose_matrix(pose):
    values = np.asarray(pose, dtype=float)
    if values.shape != (6,) or not np.isfinite(values).all():
        raise ValueError("Expected finite [x y z rx ry rz], in meters/radians")
    transform = np.eye(4)
    transform[:3,:3] = cv2.Rodrigues(values[3:])[0]
    transform[:3,3] = values[:3]
    return transform


def flange_from_state(state):
    base_from_tcp = pose_matrix(state["actual_tcp_pose_base_m_rotvec_rad"])
    flange_from_tcp = pose_matrix(state["active_tcp_offset_flange_m_rotvec_rad"])
    # A configured TCP is a mathematical frame. Its physical grasp location
    # need not be validated to undo that known frame offset.
    return base_from_tcp @ np.linalg.inv(flange_from_tcp)


def compare_robot_bracket(before, after, camera_start, camera_end):
    """Compare endpoints only; does NOT prove absence of motion between them."""
    result = {
        "status": "rejected_robot_bracket", "before": before, "after": after,
        "camera_start_host_monotonic_s": camera_start,
        "camera_end_host_monotonic_s": camera_end,
        "T_base_from_flange_before": None, "T_base_from_flange_after": None,
        "endpoint_translation_mm": None, "endpoint_rotation_deg": None,
        "max_joint_change_deg": None, "bracket_duration_s": None,
        "thresholds": {"translation_mm": 1.0, "rotation_deg": 0.1,
                       "joint_change_deg": 0.1, "bracket_duration_s": 10.0},
        "motion_between_endpoints_observed": False,
        "note": "Endpoint agreement only. Operator must keep robot still throughout; not synchronized motion capture.",
    }
    try:
        start = float(before["host_sample_monotonic_s"])
        end = float(after["host_sample_monotonic_s"])
        if not all(math.isfinite(v) for v in (start, end, camera_start, camera_end)):
            raise ValueError("Invalid host timestamps")
        if not start <= camera_start <= camera_end <= end or end-start > 10.0:
            return dict(result, status="rejected_bracket_timing")
        if (before["robot_ip"] != after["robot_ip"] or
                before["controller_version"] != after["controller_version"] or
                after["controller_timestamp_s"] <= before["controller_timestamp_s"]):
            return dict(result, status="rejected_robot_identity_or_time")
        old_offset = pose_matrix(before["active_tcp_offset_flange_m_rotvec_rad"])
        new_offset = pose_matrix(after["active_tcp_offset_flange_m_rotvec_rad"])
        if not np.allclose(old_offset, new_offset, atol=1e-7, rtol=0):
            return dict(result, status="rejected_tcp_offset_changed")
        a, b = flange_from_state(before), flange_from_state(after)
        q1, q2 = (np.asarray(s["actual_q_rad"], dtype=float) for s in (before, after))
        if q1.shape != (6,) or q2.shape != (6,) or not np.isfinite([q1,q2]).all():
            raise ValueError("Invalid joint values")
        translation_mm = float(np.linalg.norm(b[:3,3]-a[:3,3])*1000)
        cosine = np.clip((np.trace(a[:3,:3].T@b[:3,:3])-1)/2, -1, 1)
        rotation_deg = float(np.degrees(np.arccos(cosine)))
        # Deliberately conservative: a 2pi joint representation jump is rejected.
        joint_change = float(np.degrees(np.abs(q2-q1)).max())
        result.update({
            "T_base_from_flange_before": a.tolist(), "T_base_from_flange_after": b.tolist(),
            "endpoint_translation_mm": translation_mm, "endpoint_rotation_deg": rotation_deg,
            "max_joint_change_deg": joint_change, "bracket_duration_s": end-start,
        })
        result["status"] = ("robot_bracket_consistent" if translation_mm <= 1 and
                            rotation_deg <= 0.1 and joint_change <= 0.1 else "rejected_robot_pose_changed")
        return result
    except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as error:
        return dict(result, status="rejected_invalid_robot_state", error=str(error))
