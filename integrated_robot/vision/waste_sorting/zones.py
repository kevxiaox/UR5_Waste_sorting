"""Original LEFT-image pixel polygons; never robot coordinates."""
import copy
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np

ZONE_NAMES = ("LEFT_CELL", "MIDDLE_CELL", "RIGHT_CELL", "PICK_ZONE")
PURPOSES = dict(zip(ZONE_NAMES, ("recyclable", "unused", "residual", "pick")))
COLORS = {"LEFT_CELL": (60, 210, 60), "MIDDLE_CELL": (150, 150, 150),
          "RIGHT_CELL": (60, 100, 255), "PICK_ZONE": (255, 200, 30)}


def timestamp():
    return datetime.now(timezone.utc).isoformat()


def new_workspace(width, height, source=None):
    return {
        "schema_version": 1, "scheme": "three_cell_2026-10-22",
        "coordinate_frame": "ZED_LEFT_IMAGE_PIXELS", "image_size": [width, height],
        "source": source or {}, "saved_at_utc": None,
        "camera_pose_revalidated": False,
        "zones": {name: {
            "purpose": PURPOSES[name], "polygon_px": [],
            "physical_identity_confirmed": False,
            "robot_target": {
                "base_xyz_m": None, "tcp_rotation_vector_rad": None,
                "safe_approach_height_m": None, "validated_area_base_m": None,
                "reachable": False, "collision_checked": False,
                "human_validated": False,
            },
        } for name in ZONE_NAMES},
    }


def polygon_array(vertices, size):
    """Require simple, strictly convex polygons, in image bounds."""
    if not isinstance(vertices, list) or len(vertices) < 3:
        raise ValueError("Need at least 3 polygon vertices")
    for p in vertices:
        if (not isinstance(p, (list, tuple)) or len(p) != 2 or
                any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                    not math.isfinite(v) for v in p)):
            raise ValueError("Vertices must be finite pixel pairs")
    pts = np.asarray(vertices, dtype=np.float32)
    w, h = size
    if np.any(pts < 0) or np.any(pts[:, 0] >= w) or np.any(pts[:, 1] >= h):
        raise ValueError("Polygon outside original image")
    if len(set(map(tuple, vertices))) != len(vertices):
        raise ValueError("Repeated polygon vertices")
    if not cv2.isContourConvex(pts) or abs(cv2.contourArea(pts)) < 4:
        raise ValueError("Use a convex polygon; click corners in perimeter order")
    return pts


def validate_workspace(config, require_complete=False):
    if not isinstance(config, dict):
        raise ValueError("Workspace must be an object")
    if (config.get("schema_version") != 1 or
            config.get("scheme") != "three_cell_2026-10-22" or
            config.get("coordinate_frame") != "ZED_LEFT_IMAGE_PIXELS"):
        raise ValueError("Unsupported workspace schema/frame/scheme")
    size = config.get("image_size")
    if (not isinstance(size, list) or len(size) != 2 or
            any(type(v) is not int or v <= 0 for v in size)):
        raise ValueError("image_size must be positive integer [width,height]")
    if not isinstance(config.get("source"), dict):
        raise ValueError("source must be an object")
    if type(config.get("camera_pose_revalidated")) is not bool:
        raise ValueError("Invalid camera verification flag")
    zones = config.get("zones")
    if not isinstance(zones, dict) or set(zones) != set(ZONE_NAMES):
        raise ValueError("Only the three cells and PICK_ZONE are allowed")
    polygons = {}
    for name in ZONE_NAMES:
        zone = zones[name]
        if not isinstance(zone, dict) or zone.get("purpose") != PURPOSES[name]:
            raise ValueError(f"Invalid purpose for {name}")
        if type(zone.get("physical_identity_confirmed")) is not bool:
            raise ValueError(f"Invalid confirmation for {name}")
        if not isinstance(zone.get("robot_target"), dict):
            raise ValueError(f"Missing robot target metadata for {name}")
        vertices = zone.get("polygon_px")
        if vertices == []:
            if require_complete:
                raise ValueError(f"{name}: polygon missing")
        else:
            polygons[name] = polygon_array(vertices, size)
        if require_complete and zone["physical_identity_confirmed"] is not True:
            raise ValueError(f"{name}: physical identity not confirmed")
    names = list(polygons)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            area, _ = cv2.intersectConvexConvex(polygons[a], polygons[b])
            if area > 0.01:
                raise ValueError(f"Overlapping zones: {a}, {b}")
    return config


def save_workspace(path, config):
    validate_workspace(config)
    payload = copy.deepcopy(config)
    payload["saved_at_utc"] = timestamp()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=2,
                                    allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def load_workspace(path, image_size=None):
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_workspace(config)
    if image_size is not None and list(image_size) != config["image_size"]:
        raise ValueError("Image resolution changed: re-annotate; no automatic scaling")
    # Saved approval is history, never proof that the tripod is unchanged today.
    config["camera_pose_revalidated"] = False
    return config


def zone_at_pixel(config, pixel):
    """2D membership only. A shared boundary is deliberately ambiguous."""
    hits = []
    for name, zone in config["zones"].items():
        if zone["polygon_px"] and cv2.pointPolygonTest(
                np.asarray(zone["polygon_px"], np.float32), tuple(map(float, pixel)), False) >= 0:
            hits.append(name)
    return hits


def display_to_original(x, y, original_size, display_size):
    w, h = original_size
    dw, dh = display_size
    if not (0 <= x < dw and 0 <= y < dh):
        return None
    return [min(w - 1, int(x * w / dw)), min(h - 1, int(y * h / dh))]


def overlay(frame, config):
    out = frame.copy()
    for name in ZONE_NAMES:
        zone = config["zones"][name]
        if not zone["polygon_px"]:
            continue
        pts = np.asarray(zone["polygon_px"], np.int32)
        tint = out.copy()
        cv2.fillPoly(tint, [pts], COLORS[name])
        out = cv2.addWeighted(tint, 0.18, out, 0.82, 0)
        cv2.polylines(out, [pts], True, COLORS[name], 2)
        moments = cv2.moments(pts)
        center = (int(moments["m10"] / moments["m00"]),
                  int(moments["m01"] / moments["m00"]))
        cv2.drawMarker(out, center, COLORS[name], cv2.MARKER_CROSS, 14, 2)
        label = f"{name}: {zone['purpose']} " + (
            "ID OK" if zone["physical_identity_confirmed"] else "ID UNCONFIRMED")
        cv2.putText(out, label, (max(2, center[0] - 90), max(18, center[1] - 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, COLORS[name], 1, cv2.LINE_AA)
    return out
