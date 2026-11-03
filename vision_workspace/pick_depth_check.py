"""Record camera-depth frames; optionally preview candidate Base surface coordinates."""
import argparse
from collections import Counter
from datetime import datetime
import hashlib
import json
from pathlib import Path
import time

import cv2
import numpy as np

from waste_sorting.camera import ZedSource
from waste_sorting.candidate_preview import load_candidate, project_surface, candidate_provenance
from waste_sorting.localization import DepthPolicy, sample_surface, sample_tool_point, summarize_samples
from waste_sorting.observations import compare_robot_bracket
from waste_sorting.rtde_readonly import read_robot_state
from waste_sorting.zones import (
    display_to_original, overlay, timestamp, validate_workspace, zone_at_pixel,
)

ROOT = Path(__file__).resolve().parent
WINDOW = "PICK depth diagnostic - CAMERA FRAME ONLY - ROBOT DISABLED"


def save_measurement(folder, frame, config, config_hash, pixel, samples, policy, tool_point=False,
                     robot_bracket=None, feature_id=None, candidate=None):
    if candidate is not None and (tool_point or robot_bracket is not None):
        raise ValueError('Candidate surface preview cannot be combined with tool observations')
    folder.mkdir(parents=True, exist_ok=False)
    summary = summarize_samples(samples, policy)
    scope = "tool_point_diagnostic" if tool_point else "pick_zone_diagnostic"
    summary["measurement_scope"] = scope
    summary["calibration_pair"] = False
    if tool_point:
        summary["note"] = "Unpaired candidate tool feature; identity/rigidity and camera-to-base transform NOT validated."
    if robot_bracket is not None:
        scope = "candidate_tool_robot_observation"
        summary["measurement_scope"] = scope
        summary["robot_bracket_status"] = robot_bracket["status"]
        summary["robot_endpoint_translation_mm"] = robot_bracket.get("endpoint_translation_mm")
        summary["robot_endpoint_rotation_deg"] = robot_bracket.get("endpoint_rotation_deg")
        summary["observation_status"] = (
            "candidate_observation_recorded" if summary["status"] == "camera_measurement_stable"
            and robot_bracket["status"] == "robot_bracket_consistent" else "rejected_candidate_observation")
        summary["note"] = "Candidate association with bracketed robot poses; feature offset unknown; no extrinsics or grasp target."
    if candidate is not None:
        scope = 'candidate_base_surface_preview'
        summary['measurement_scope'] = scope
        summary['candidate_preview'] = project_surface(summary,pixel,config,candidate)
    payload = {
        "timestamp_utc": timestamp(),
        "type": ("candidate_tool_robot_observation" if robot_bracket is not None else
                 "unpaired_tool_point_diagnostic" if tool_point else "camera_surface_diagnostic"),
        "measurement_scope": scope, "calibration_pair": False,
        "rigid_point_identity_confirmed": False,
        "feature_id": feature_id,
        "robot_bracket": robot_bracket,
        "workspace_snapshot": config, "workspace_sha256": config_hash,
        "pixel": pixel, "summary": summary, "samples": samples,
        "rejection_counts": dict(Counter(s["status"] for s in samples
                                         if s["status"] != "valid_camera_sample")),
        "snapshot_note": "RGB from final sample; object/camera must remain stationary during all frames",
    }
    if candidate is not None:
        payload['type'] = scope
        payload['candidate_provenance'] = candidate_provenance(candidate)
    if not cv2.imwrite(str(folder / "rgb.png"), frame):
        raise IOError("Failed to save diagnostic RGB")
    view = overlay(frame, config)
    cv2.drawMarker(view, tuple(pixel), (0, 255, 255), cv2.MARKER_CROSS, 24, 2)
    label = ("CANDIDATE OBSERVATION / NO VALIDATED EXTRINSICS" if robot_bracket is not None else
             "TOOL POINT DIAGNOSTIC / NOT A CALIBRATION PAIR" if tool_point else
             "CAMERA SURFACE ONLY / NOT A GRASP TARGET")
    if candidate is not None:
        label = 'CANDIDATE BASE SURFACE ONLY / TCP UNVERIFIED / ROBOT DISABLED'
    cv2.putText(view, label, (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if candidate is not None:
        point = summary['candidate_preview']['candidate_base_xyz_m']
        line = (f'Candidate Base XYZ mm: {np.round(np.asarray(point)*1000,2).tolist()}' if point is not None else
                'Candidate Base unavailable: '+summary['candidate_preview']['status'])
        cv2.putText(view,line,(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),2)
    if not cv2.imwrite(str(folder / "marked.png"), view):
        raise IOError("Failed to save diagnostic overlay")
    (folder / "measurement.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return summary, payload["rejection_counts"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config/workspace_zones.json")
    parser.add_argument("--output", type=Path, help="Override the mode-specific output directory")
    parser.add_argument("--tool-point", action="store_true",
                        help="Allow any image pixel for candidate tool depth diagnostics only; no robot pose pairing")
    parser.add_argument("--pair-robot", action="store_true",
                        help="With --tool-point, read robot poses before/after sampling; no control commands")
    parser.add_argument('--candidate-base',action='store_true',help='Preview unvalidated Base surface coordinates in PICK_ZONE; no robot connection')
    parser.add_argument('--candidate-report',type=Path,default=ROOT/'data/calibration_analysis/candidate_report.json')
    parser.add_argument('--training-manifest',type=Path,default=ROOT/'data/calibration_analysis/training_manifest.json')
    parser.add_argument("--robot-ip", default="147.250.35.40")
    parser.add_argument("--feature-id", default="onrobot_body_logo_center",
                        help="Name of the SAME fixed-body physical feature in all candidate observations")
    parser.add_argument("--scale", type=float, default=0.85)
    args = parser.parse_args()
    if not 0.2 <= args.scale <= 2:
        parser.error("--scale must be between 0.2 and 2")
    if args.pair_robot and not args.tool_point:
        parser.error("--pair-robot requires --tool-point; not enabled for sorting")
    if args.candidate_base and (args.tool_point or args.pair_robot):
        parser.error('--candidate-base cannot be combined with --tool-point or --pair-robot')
    if not args.feature_id.strip():
        parser.error("--feature-id cannot be empty")
    if args.output is None:
        args.output = ROOT / "data" / ('candidate_base_checks' if args.candidate_base else "tool_robot_observations" if args.pair_robot else
                                      "tool_point_checks" if args.tool_point else "localization_checks")
    window = ("CANDIDATE tool/robot observation - ROBOT DISABLED" if args.pair_robot else
              "TOOL point depth - UNPAIRED - ROBOT DISABLED" if args.tool_point else WINDOW)
    if args.candidate_base:
        window = 'CANDIDATE BASE surface preview - TCP UNVERIFIED - ROBOT DISABLED'
    # Read the exact file once; hash and snapshot refer to the same bytes.
    config_bytes = args.config.read_bytes()
    config = json.loads(config_bytes)
    validate_workspace(config, require_complete=True)
    config["camera_pose_revalidated"] = False
    config_hash = hashlib.sha256(config_bytes).hexdigest()
    candidate = load_candidate(args.candidate_report,args.training_manifest,config_hash) if args.candidate_base else None
    policy = DepthPolicy()
    with ZedSource() as camera:
        frame, xyz = camera.read_with_xyz()
        h, w = frame.shape[:2]
        if config["image_size"] != [w, h]:
            raise ValueError("Image size changed: re-annotate workspace")
        if (config["source"].get("kind") != "zed_live" or
                config["source"].get("view") != "LEFT" or
                config["source"].get("serial_number") != camera.serial):
            raise ValueError("Workspace must be verified with this live ZED camera first")
        display_size = (round(w * args.scale), round(h * args.scale))
        state = {"pixel": None, "samples": None, "latest": None,
                 "robot_before": None, "sample_times": []}

        def mouse(event, x, y, flags, userdata):
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            pixel = display_to_original(x, y, [w,h], display_size)
            if pixel is None:
                return
            state.update(pixel=pixel, samples=None, latest=None)
            print("Selected pixel:", pixel, "| 2D zones:", zone_at_pixel(config, pixel))

        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window, mouse)
        print("Close other ZED programs first. Robot and gripper execution DISABLED.")
        if candidate is not None:
            print('CANDIDATE BASE PREVIEW: raw evidence/frozen fit rechecked. No robot connection.')
            print('Coordinates are unvalidated surface points, NOT grasp/TCP targets. Saved base_xyz_m stays null.')
        if args.pair_robot:
            print(f"READ-ONLY robot OUTPUTS: {args.robot_ip}:30004 | feature_id={args.feature_id}")
            print("M reads pre-pose > 20 camera frames > post-pose. Keep robot stationary for the entire capture.")
            print("Endpoint agreement does NOT prove no movement between endpoints. No camera-to-base transform is generated.")
        if args.tool_point:
            print("TOOL POINT DIAGNOSTIC: any original-image pixel allowed; NOT a validated calibration pair.")
            print("Inspect overlays > V confirm camera alignment > click a stationary fixed-body feature > M")
            print("Do not move the camera or robot. Sampling cannot prove point identity/rigidity or absolute accuracy.")
        else:
            print("Inspect overlays > V confirm today's camera alignment > click surface in PICK_ZONE > M")
        print("M records/saves 20 NEW frames; keep object and camera still. C clears | +/- zoom | Q quits.")
        try:
            while True:
                frame, xyz = camera.read_with_xyz()
                frame_time = time.monotonic()
                if frame.shape[:2] != (h, w):
                    raise ValueError("Camera resolution changed during capture")
                pixel = state["pixel"]
                view = overlay(frame, config)
                if pixel is not None:
                    sampler = sample_tool_point if args.tool_point else sample_surface
                    sample = sampler(xyz, pixel, config, policy)
                    cv2.drawMarker(view, tuple(pixel), (0,255,255), cv2.MARKER_CROSS, 20, 2)
                    if state["samples"] is not None:
                        sample["timestamp_utc"] = timestamp()
                        state["samples"].append(sample)
                        state["sample_times"].append(frame_time)
                        if len(state["samples"]) == policy.sample_frames:
                            bracket = None
                            if args.pair_robot:
                                try:
                                    after = read_robot_state(args.robot_ip)
                                    bracket = compare_robot_bracket(
                                        state["robot_before"], after,
                                        state["sample_times"][0], state["sample_times"][-1])
                                except (OSError, ValueError) as error:
                                    bracket = {"status": "rejected_robot_read", "error": str(error),
                                               "before": state["robot_before"], "after": None}
                            folder = args.output / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                            summary, rejections = save_measurement(
                                folder, frame, config, config_hash, pixel, state["samples"], policy,
                                tool_point=args.tool_point, robot_bracket=bracket,
                                feature_id=args.feature_id if args.pair_robot else None, candidate=candidate)
                            state["latest"] = summary
                            state["samples"] = None
                            print(json.dumps(summary, ensure_ascii=False, indent=2))
                            print("Rejections:", rejections, "| Saved:", folder / "measurement.json")
                    live_line = (f"Pixel {pixel} | {sample['status']} | "
                                 f"patch valid: {sample['patch_valid_fraction']}")
                else:
                    live_line = ("Select candidate fixed-body point anywhere in image; not fingers, joints or cables"
                                 if args.tool_point else
                                 "Select visible object surface inside PICK_ZONE; avoid edges/highlights")
                latest = state["latest"]
                progress = len(state["samples"]) if state["samples"] is not None else None
                lines = [
                    f"{'TOOL POINT' if args.tool_point else 'PICK ZONE'} | "
                    f"Camera alignment: {'VERIFIED' if config['camera_pose_revalidated'] else 'UNVERIFIED'}"
                    " | V confirm | Click surface > M record 20 frames | C clear | +/- zoom | Q quit",
                    live_line,
                    f"Recording: {progress}/20" if progress is not None else
                    (f"Last: {latest['status']} | {latest['valid_frames']}/20 | "
                     f"std XYZ mm: {latest['std_xyz_mm']}" if latest else "No measurement recorded yet"),
                    ("CANDIDATE OBSERVATION | M: pre-pose / camera / post-pose | ROBOT DISABLED" if args.pair_robot else
                     "UNPAIRED TOOL DIAGNOSTIC | camera XYZ only | ROBOT DISABLED" if args.tool_point else
                     "CAMERA XYZ (meters) ONLY | no Base transform | ROBOT EXECUTION DISABLED"),
                ]
                if candidate is not None:
                    preview = project_surface(sample,pixel,config,candidate) if pixel is not None else None
                    base = preview['candidate_base_xyz_m'] if preview else None
                    lines[-1] = (f'CANDIDATE Base XYZ mm: {np.round(np.asarray(base)*1000,2).tolist()}' if base is not None else
                                  f"Candidate Base: {preview['status'] if preview else 'click inside PICK_ZONE'}")
                    lines.append('UNVALIDATED SURFACE | TCP UNVERIFIED | NO ROBOT CONNECTION | ROBOT DISABLED')
                footer = np.full((10+25*len(lines), display_size[0], 3), 20, np.uint8)
                for i, line in enumerate(lines):
                    cv2.putText(footer, line, (8, 21 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                                0.43, (255,255,255), 1, cv2.LINE_AA)
                cv2.imshow(window, np.vstack((cv2.resize(view, display_size), footer)))
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                    if state["samples"] is not None:
                        print("Incomplete recording discarded; no summary was saved")
                    break
                if key == ord("v"):
                    config["camera_pose_revalidated"] = True
                    state.update(samples=None, latest=None)
                    print("Current camera/table alignment confirmed for this diagnostic session only")
                elif key == ord("m"):
                    if config["camera_pose_revalidated"] is not True:
                        print("NOT recording: inspect overlays then press V")
                    elif pixel is None:
                        print("NOT recording: click a surface point first")
                    elif not args.tool_point and zone_at_pixel(config, pixel) != ["PICK_ZONE"]:
                        print("NOT recording: click inside PICK_ZONE first")
                    elif state["samples"] is not None:
                        print("Already recording; keep the camera and object still")
                    else:
                        before = None
                        if args.pair_robot:
                            try:
                                before = read_robot_state(args.robot_ip)
                            except (OSError, ValueError) as error:
                                print("NOT recording: read-only robot snapshot failed:", error)
                                continue
                        state.update(samples=[], latest=None, robot_before=before, sample_times=[])
                        print("Recording 20 new frames...")
                elif key == ord("c"):
                    state.update(pixel=None, samples=None, latest=None)
                elif key in (ord("+"), ord("="), ord("-")):
                    args.scale = min(2, max(0.2, args.scale + (-0.1 if key == ord("-") else 0.1)))
                    display_size = (round(w * args.scale), round(h * args.scale))
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except (ValueError, RuntimeError, OSError, cv2.error) as error:
        raise SystemExit(str(error))
