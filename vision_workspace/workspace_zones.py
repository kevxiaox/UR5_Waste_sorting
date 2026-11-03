"""Annotate/check polygons on a ZED LEFT image, or rehearse on an archived image."""
import argparse
from contextlib import nullcontext
import json
from pathlib import Path

import cv2
import numpy as np

from waste_sorting.camera import ZedSource
from waste_sorting.decision import assess_target
from waste_sorting.zones import (
    ZONE_NAMES, COLORS, display_to_original, load_workspace, new_workspace,
    overlay, polygon_array, save_workspace, validate_workspace, zone_at_pixel,
)

ROOT = Path(__file__).resolve().parent
WINDOW = "ZED workspace - 2D ONLY - ROBOT DISABLED"


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", type=Path, help="Offline rehearsal; cannot validate today's camera pose")
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace_zones.json")
    parser.add_argument("--scale", type=float, default=0.85, help="Explicit display scale; original pixels saved")
    parser.add_argument("--check", action="store_true", help="Inspect mode; Tab switches editing")
    parser.add_argument("--detections", type=Path, help="JSON list of detections for overlay; no YOLO model implied")
    parser.add_argument("--render", type=Path, help="Headless overlay PNG; requires --image")
    args = parser.parse_args()
    if not 0.2 <= args.scale <= 2:
        parser.error("--scale must be between 0.2 and 2")
    if args.render and not args.image:
        parser.error("--render requires --image")
    return args


def main():
    args = arguments()
    offline = args.image is not None
    detections = []
    if args.detections:
        detections = json.loads(args.detections.read_text(encoding="utf-8"))
        if not isinstance(detections, list):
            raise ValueError("Detections must be a JSON list")
    source_context = nullcontext(None) if offline else ZedSource()
    with source_context as camera:
        frame = cv2.imread(str(args.image)) if offline else camera.read()
        if frame is None:
            raise ValueError(f"Cannot read image: {args.image}")
        h, w = frame.shape[:2]
        source = ({"kind": "offline_image", "path": str(args.image.resolve())} if offline else
                  {"kind": "zed_live", "serial_number": camera.serial, "view": "LEFT"})
        config = load_workspace(args.config, [w, h]) if args.config.exists() else new_workspace(w, h, source)
        saved_serial = config["source"].get("serial_number")
        if not offline and saved_serial not in (None, camera.serial):
            raise ValueError("Different camera serial: use a new --config and re-annotate")
        if args.render:
            args.render.parent.mkdir(parents=True, exist_ok=True)
            view = overlay(frame, config)
            cv2.putText(view, "OFFLINE / CAMERA NOT REVALIDATED / ROBOT DISABLED", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            if not cv2.imwrite(str(args.render), view):
                raise IOError("Could not write overlay")
            print(f"Offline overlay: {args.render}")
            return
        display_size = (max(1, round(w * args.scale)), max(1, round(h * args.scale)))
        # AUTOSIZE prevents an extra, implicit OS/OpenCV resize of the image area.
        # +/- explicitly resize the rendered image and update the inverse mapping.
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        state = {"selected": "LEFT_CELL", "draft": [], "inspect": args.check,
                 "frozen": offline, "probe": None}

        def invalidate():
            config["camera_pose_revalidated"] = False

        def mouse(event, x, y, flags, userdata):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            pixel = display_to_original(x, y, [w, h], display_size)
            if pixel is None:
                return
            if state["inspect"]:
                state["probe"] = pixel
                print("2D pixel", pixel, "zones:", zone_at_pixel(config, pixel))
            elif state["frozen"]:
                state["draft"].append(pixel)
                invalidate()
            else:
                print("Press SPACE to freeze the image before drawing")

        cv2.setMouseCallback(WINDOW, mouse)
        print("1=physical LEFT/recyclable 2=MIDDLE/unused 3=physical RIGHT/residual 4=PICK")
        print("SPACE freeze/live | click corners in perimeter order | Enter commit | U undo draft")
        print("Y confirm selected physical identity | V validate ALL against LIVE view this session")
        print("S save | L reload | R clear selected | Tab inspect/edit | +/- zoom | Q quit")
        print("Offline image:", offline, "| No robot connection or motion")
        try:
            while True:
                if not state["frozen"]:
                    frame = camera.read()
                    if frame.shape[:2] != (h, w):
                        raise ValueError("Camera image size changed")
                view = overlay(frame, config)
                draft = state["draft"]
                if draft:
                    pts = np.asarray(draft, np.int32)
                    cv2.polylines(view, [pts], False, COLORS[state["selected"]], 2)
                    for p in draft:
                        cv2.circle(view, tuple(p), 4, (255, 255, 255), -1)
                if state["probe"]:
                    p = state["probe"]
                    cv2.drawMarker(view, tuple(p), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
                    label = f"2D {p}: {zone_at_pixel(config, p)}"
                    cv2.putText(view, label, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                for detection in detections:
                    decision = assess_target(detection, config)
                    if isinstance(detection, dict):
                        p = detection.get("center_pixel")
                        if isinstance(p, list) and len(p) == 2 and all(
                                type(v) in (int, float) and np.isfinite(v) for v in p):
                            u, v = map(int, p)
                            if 0 <= u < w and 0 <= v < h:
                                cv2.drawMarker(view, (u, v), (0, 255, 255), cv2.MARKER_CROSS, 14, 1)
                                cv2.putText(view, decision["status"], (u, max(20, v - 8)),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 255), 1)
                lines = [
                    f"{'OFFLINE' if offline else 'LIVE'} {'FROZEN' if state['frozen'] else ''} | "
                    f"{'INSPECT' if state['inspect'] else 'EDIT'} | selected: {state['selected']}",
                    "1 LEFT/recycle  2 MIDDLE/unused  3 RIGHT/residual  4 PICK | SPACE freeze | Tab inspect",
                    "Click corners > Enter > Y confirm ID | V verify live | S save | L load | R clear | Q quit",
                    f"Camera pose this session: {'VERIFIED' if config['camera_pose_revalidated'] else 'UNVERIFIED'}"
                    " | 2D pixels only | ROBOT DISABLED",
                ]
                # Keep the instruction bar OUTSIDE the camera image. Footer clicks
                # are rejected by display_to_original, so even top-edge marks remain visible.
                rendered = cv2.resize(view, display_size)
                footer = np.full((110, display_size[0], 3), 20, dtype=np.uint8)
                for i, line in enumerate(lines):
                    cv2.putText(footer, line, (10, 21 + 25 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.44,
                                (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(WINDOW, np.vstack((rendered, footer)))
                key = cv2.waitKey(15) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                    break
                try:
                    if key in map(ord, "1234"):
                        state["selected"] = ZONE_NAMES[key - ord("1")]
                        state["draft"] = []
                    elif key == 9:
                        state["inspect"] = not state["inspect"]
                        state["draft"] = []
                    elif key == ord(" ") and not offline:
                        state["frozen"] = not state["frozen"]
                        state["draft"] = []
                    elif key in (ord("+"), ord("="), ord("-")):
                        args.scale = min(2, max(0.2, args.scale + (-0.1 if key == ord("-") else 0.1)))
                        display_size = (max(1, round(w * args.scale)), max(1, round(h * args.scale)))
                    elif key == ord("u") and draft:
                        draft.pop()
                    elif key in (10, 13) and not state["inspect"]:
                        polygon_array(draft, [w, h])
                        candidate = json.loads(json.dumps(config))
                        zone = candidate["zones"][state["selected"]]
                        zone["polygon_px"] = draft.copy()
                        zone["physical_identity_confirmed"] = False
                        validate_workspace(candidate)
                        config = candidate
                        invalidate()
                        state["draft"] = []
                        print("Polygon committed:", state["selected"], zone["polygon_px"])
                    elif key == ord("r"):
                        zone = config["zones"][state["selected"]]
                        zone["polygon_px"] = []
                        zone["physical_identity_confirmed"] = False
                        state["draft"] = []
                        invalidate()
                    elif key == ord("y"):
                        if draft:
                            raise ValueError("Commit draft with Enter first")
                        zone = config["zones"][state["selected"]]
                        polygon_array(zone["polygon_px"], [w, h])
                        zone["physical_identity_confirmed"] = True
                        invalidate()
                        print("Physical identity explicitly confirmed:", state["selected"])
                    elif key == ord("v"):
                        if offline or state["frozen"] or draft:
                            raise ValueError("Verification requires LIVE unfrozen image and no pending draft")
                        validate_workspace(config, require_complete=True)
                        config["camera_pose_revalidated"] = True
                        config["source"] = source
                        print("Confirmed current camera/table alignment for this session (2D only)")
                    elif key == ord("s"):
                        if draft:
                            raise ValueError("Commit draft with Enter before saving")
                        save_workspace(args.config, config)
                        print("Saved:", args.config, "| camera verification expires on reload/restart")
                    elif key == ord("l"):
                        candidate = load_workspace(args.config, [w, h])
                        serial = candidate["source"].get("serial_number")
                        if not offline and serial not in (None, camera.serial):
                            raise ValueError("Different camera serial: use a new config")
                        config = candidate
                        state["draft"] = []
                        print("Loaded; inspect alignment then V to revalidate live")
                except (ValueError, OSError, TypeError, KeyError) as exc:
                    print("NOT accepted:", exc)
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, cv2.error) as error:
        raise SystemExit(str(error))
