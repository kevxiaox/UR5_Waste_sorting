"""Diagnostic camera-frame surface samples. No grasp or robot targets."""
from dataclasses import asdict, dataclass

import cv2
import numpy as np

from .zones import zone_at_pixel


@dataclass(frozen=True)
class DepthPolicy:
    radius_px: int = 2
    sample_frames: int = 20
    min_valid_frames: int = 12
    min_valid_fraction: float = 0.6
    max_patch_z_span_m: float = 0.02
    max_temporal_std_m: float = 0.01


def sample_surface(xyz, pixel, workspace, policy=DepthPolicy()):
    """Use the clicked center itself; NEVER replace invalid center by background.

    Neighborhood validity/spread are diagnostics only. They cannot establish
    object identity or grasp suitability. Full patch must lie inside PICK_ZONE.
    """
    u, v = pixel
    result = {"pixel": [u, v], "status": "waiting_camera_revalidation",
              "camera_xyz_m": None, "patch_valid_fraction": None,
              "patch_z_span_m": None}
    if workspace.get("camera_pose_revalidated") is not True:
        return result
    w, h = workspace["image_size"]
    if not isinstance(xyz, np.ndarray) or xyz.shape != (h, w, 3):
        return dict(result, status="rejected_xyz_shape")
    if type(u) is not int or type(v) is not int or not (0 <= u < w and 0 <= v < h):
        return dict(result, status="rejected_pixel")
    if zone_at_pixel(workspace, pixel) != ["PICK_ZONE"]:
        return dict(result, status="rejected_outside_pick_zone")
    r = policy.radius_px
    if u-r < 0 or v-r < 0 or u+r >= w or v+r >= h:
        return dict(result, status="rejected_patch_at_image_edge")
    polygon = np.asarray(workspace["zones"]["PICK_ZONE"]["polygon_px"], np.float32)
    if any(cv2.pointPolygonTest(polygon, (float(x), float(y)), False) <= 0
           for x in (u-r, u+r) for y in (v-r, v+r)):
        return dict(result, status="rejected_patch_at_zone_edge")
    return _sample_depth_patch(xyz, pixel, result, policy)


def sample_tool_point(xyz, pixel, workspace, policy=DepthPolicy()):
    """Unpaired camera diagnostic anywhere in the image, NOT a sorting target.

    This only tests depth at a manually selected candidate fixed-body feature.
    It does not establish feature identity, rigidity, robot pose or extrinsics.
    """
    u, v = pixel
    result = {"pixel": [u, v], "status": "waiting_camera_revalidation",
              "camera_xyz_m": None, "patch_valid_fraction": None,
              "patch_z_span_m": None, "measurement_scope": "tool_point_diagnostic",
              "calibration_pair": False}
    if workspace.get("camera_pose_revalidated") is not True:
        return result
    w, h = workspace["image_size"]
    if not isinstance(xyz, np.ndarray) or xyz.shape != (h,w,3):
        return dict(result, status="rejected_xyz_shape")
    if type(u) is not int or type(v) is not int or not (0 <= u < w and 0 <= v < h):
        return dict(result, status="rejected_pixel")
    r = policy.radius_px
    if u-r < 0 or v-r < 0 or u+r >= w or v+r >= h:
        return dict(result, status="rejected_patch_at_image_edge")
    return _sample_depth_patch(xyz, pixel, result, policy)


def _sample_depth_patch(xyz, pixel, result, policy):
    u, v = pixel
    r = policy.radius_px
    patch = xyz[v-r:v+r+1, u-r:u+r+1]
    valid = np.isfinite(patch).all(axis=2) & (patch[..., 2] > 0)
    fraction = float(valid.mean())
    result["patch_valid_fraction"] = fraction
    center = xyz[v, u]
    if not np.isfinite(center).all() or center[2] <= 0:
        return dict(result, status="rejected_invalid_center_depth")
    if fraction < policy.min_valid_fraction:
        return dict(result, status="rejected_sparse_depth")
    z = patch[..., 2][valid]
    span = float(np.percentile(z, 90) - np.percentile(z, 10))
    result["patch_z_span_m"] = span
    if span > policy.max_patch_z_span_m:
        return dict(result, status="rejected_depth_discontinuity")
    return dict(result, status="valid_camera_sample", camera_xyz_m=center.astype(float).tolist())


def summarize_samples(samples, policy=DepthPolicy()):
    accepted = [s["camera_xyz_m"] for s in samples if s["status"] == "valid_camera_sample"]
    result = {
        "coordinate_frame": "ZED_IMAGE_CAMERA", "units": "meter",
        "total_frames": len(samples), "valid_frames": len(accepted),
        "camera_xyz_m": None, "std_xyz_mm": None,
        "status": "rejected_insufficient_frames", "policy": asdict(policy),
        "base_xyz_m": None, "robot_execution_enabled": False,
        "note": "Camera surface diagnostic only; not absolute accuracy or a robot grasp target.",
    }
    if len(samples) != policy.sample_frames or len(accepted) < policy.min_valid_frames:
        return result
    data = np.asarray(accepted, dtype=np.float64)
    spread = data.std(axis=0)
    result["std_xyz_mm"] = (spread * 1000).tolist()
    result["camera_xyz_m"] = np.median(data, axis=0).tolist()
    result["status"] = ("camera_measurement_stable" if float(spread.max()) <= policy.max_temporal_std_m
                        else "rejected_unstable_depth")
    return result
