"""Detection adapter and fail-closed dry-run decisions; no hardware imports."""
import json
import math
from pathlib import Path

from .zones import validate_workspace, zone_at_pixel

POLICY_PATH = Path(__file__).resolve().parents[1] / "config" / "sorting_policy.json"
ROBOT_EXECUTION_ENABLED = False


def finite_vector(value, length):
    return (isinstance(value, (list, tuple)) and len(value) == length and
            all(type(v) in (float, int) and math.isfinite(v) for v in value))


def classify_detection(detection, *, preview_min_confidence=None):
    """Route detections; optional threshold is for explicitly labelled previews.

    assess_target and DryRunRobot retain the unmodified policy threshold.
    """
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    threshold = policy['min_confidence'] if preview_min_confidence is None else preview_min_confidence
    if type(threshold) not in (float,int) or not math.isfinite(threshold) or not 0 <= threshold <= 1:
        raise ValueError('Preview confidence must be finite and between 0 and 1')
    result = {"category": None, "target_zone": None, "status": "rejected_detection"}
    if not isinstance(detection, dict):
        return result
    cls = detection.get("class_name")
    if not isinstance(cls, str) or cls not in policy["class_to_category"]:
        return dict(result, status="rejected_unknown_class")
    confidence = detection.get("confidence")
    if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
        return dict(result, status="rejected_confidence")
    if confidence < threshold:
        return dict(result, status="rejected_low_confidence")
    bbox, center = detection.get("bbox"), detection.get("center_pixel")
    if not finite_vector(bbox, 4) or not finite_vector(center, 2):
        return result
    x1, y1, x2, y2 = bbox
    u, v = center
    if not (0 <= x1 < x2 and 0 <= y1 < y2 and x1 <= u <= x2 and y1 <= v <= y2):
        return result
    category = policy["class_to_category"][cls]
    return {"category": category, "target_zone": policy["category_to_zone"][category],
            "status": "ready_for_localization"}


def assess_target(detection, workspace, camera_xyz_m=None):
    """2D -> camera XYZ gate. Robot calibration is deliberately not implemented yet."""
    result = classify_detection(detection)
    result.update({"robot_execution_enabled": False, "coordinate_frame": "ZED_IMAGE_CAMERA",
                   "camera_xyz_m": None, "base_xyz_m": None})
    if result["status"] != "ready_for_localization":
        return result
    try:
        validate_workspace(workspace, require_complete=True)
    except (ValueError, TypeError, KeyError):
        return dict(result, status="waiting_workspace_validation")
    if workspace["camera_pose_revalidated"] is not True:
        return dict(result, status="waiting_camera_revalidation")
    width, height = workspace["image_size"]
    x1, y1, x2, y2 = detection["bbox"]
    if not (x2 < width and y2 < height):
        return dict(result, status="rejected_detection_outside_image")
    if zone_at_pixel(workspace, detection["center_pixel"]) != ["PICK_ZONE"]:
        return dict(result, status="rejected_outside_pick_zone")
    if not finite_vector(camera_xyz_m, 3) or camera_xyz_m[2] <= 0:
        return dict(result, status="waiting_valid_depth")
    return dict(result, camera_xyz_m=list(camera_xyz_m), status="waiting_base_calibration")
