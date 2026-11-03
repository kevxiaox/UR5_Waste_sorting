"""Capture Eye-to-Hand camera-side reference points with a ZED Mini.

NO robot motion. Data must later be paired with independently measured UR5
base-frame coordinates of the same physical marks.
"""
import csv
import json
import math
import os
from datetime import datetime
from pathlib import Path

# Keep DLL directory handles alive for the whole process.
DLL_DIRS = [
    r"C:\Program Files (x86)\ZED SDK\bin",
    r"D:\Robotique\bin\x64",
]
DLL_HANDLES = []
for dll_dir in DLL_DIRS:
    if not os.path.isdir(dll_dir):
        raise FileNotFoundError(f"DLL directory not found: {dll_dir}")
    DLL_HANDLES.append(os.add_dll_directory(dll_dir))

import cv2
import numpy as np
import pyzed.sl as sl

POINT_COUNT = 8
SAMPLE_FRAMES = 20
MIN_VALID_FRAMES = 12
MAX_STD_MM = 10.0  # Diagnostic threshold, not a guarantee of accuracy
OUTPUT_ROOT = Path(r"D:\UR5_Waste_Sorting\calibration\sessions")
WINDOW = "ZED Eye-to-Hand - Click marker, then S to save"

selected = None
points = []


def mouse_callback(event, x, y, flags, userdata):
    global selected
    if event == cv2.EVENT_LBUTTONDOWN:
        selected = (int(x), int(y))


def capture_xyz(zed, runtime, cloud, x, y):
    samples = []
    for _ in range(SAMPLE_FRAMES):
        if zed.grab(runtime) != sl.ERROR_CODE.SUCCESS:
            continue
        zed.retrieve_measure(cloud, sl.MEASURE.XYZ)
        err, vec = cloud.get_value(x, y)
        if err != sl.ERROR_CODE.SUCCESS:
            continue
        xyz = np.asarray(vec[:3], dtype=np.float64)
        if np.all(np.isfinite(xyz)) and xyz[2] > 0:
            samples.append(xyz)
    if len(samples) < MIN_VALID_FRAMES:
        return None, f"Only {len(samples)}/{SAMPLE_FRAMES} valid depth frames"
    data = np.asarray(samples)
    center = np.median(data, axis=0)
    spread_mm = np.std(data, axis=0) * 1000.0
    if float(np.max(spread_mm)) > MAX_STD_MM:
        return None, (f"Unstable XYZ: std(mm)="
                      f"{np.round(spread_mm, 1).tolist()}; try a clearer mark")
    return {
        "camera_xyz_m": [round(float(v), 6) for v in center],
        "std_xyz_mm": [round(float(v), 2) for v in spread_mm],
        "valid_frames": len(samples),
    }, None


def save_results(folder, last_image):
    folder.mkdir(parents=True, exist_ok=True)
    payload = {
        "type": "camera_reference_points_only",
        "coordinate_frame": "ZED_IMAGE_CAMERA",
        "units": "meters",
        "resolution": "1280x720",
        "depth_mode": "NEURAL",
        "note": "Not calibrated to UR5. DO NOT use as robot motion commands.",
        "points": points,
    }
    (folder / "camera_points.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    with (folder / "camera_points.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["point_id", "pixel_x", "pixel_y", "Xc_m", "Yc_m", "Zc_m", "valid_frames", "std_x_mm", "std_y_mm", "std_z_mm"])
        for p in points:
            writer.writerow([
                p["point_id"], *p["pixel_xy"], *p["camera_xyz_m"],
                p["valid_frames"], *p["std_xyz_mm"]
            ])
    if last_image is not None:
        annotated = last_image.copy()
        for p in points:
            x, y = p["pixel_xy"]
            cv2.drawMarker(annotated, (x, y), (0, 255, 255),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.putText(annotated, p["point_id"], (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(annotated, p["point_id"], (x + 10, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imwrite(str(folder / "marked_reference_points.png"), annotated)
    print(f"Saved {len(points)} point(s) to {folder}")


def main():
    global selected
    folder = OUTPUT_ROOT / datetime.now().strftime("%Y%m%d_%H%M%S")
    print("NO ROBOT MOTION. The camera and table must remain still while recording.")
    print("Mouse click: select physical cross mark | S: measure point | U: undo | Q: quit")
    print(f"Output: {folder}")
    zed = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print("ZED open failed:", status)
        return

    runtime = sl.RuntimeParameters()
    image = sl.Mat()
    cloud = sl.Mat()
    last_image = None
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)  # one image pixel = one click pixel
    cv2.setMouseCallback(WINDOW, mouse_callback)
    try:
        failures = 0
        while True:
            status = zed.grab(runtime)
            if status != sl.ERROR_CODE.SUCCESS:
                failures += 1
                if failures > 120:
                    print("Video frames failed repeatedly:", status)
                    break
                cv2.waitKey(1)
                continue
            failures = 0
            zed.retrieve_image(image, sl.VIEW.LEFT)
            raw = image.get_data()
            if raw.ndim == 3 and raw.shape[2] == 4:
                bgr = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
            elif raw.ndim == 3 and raw.shape[2] == 3:
                bgr = raw.copy()
            else:
                print("Unexpected image format:", raw.shape)
                break
            last_image = bgr.copy()
            h, w = bgr.shape[:2]
            view = bgr.copy()
            cv2.putText(view, f"Points: {len(points)}/{POINT_COUNT} | Click -> S | U undo | Q quit",
                        (15, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            for p in points:
                x, y = p["pixel_xy"]
                cv2.drawMarker(view, (x, y), (0, 255, 0), cv2.MARKER_CROSS, 18, 2)
                cv2.putText(view, p["point_id"], (x + 8, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            if selected is not None:
                sx, sy = selected
                if 0 <= sx < w and 0 <= sy < h:
                    cv2.drawMarker(view, selected, (0, 255, 255), cv2.MARKER_CROSS, 25, 2)
            cv2.imshow(WINDOW, view)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("u"):
                if points:
                    deleted = points.pop()
                    print("Undone:", deleted["point_id"])
                    save_results(folder, last_image)
                continue
            if key == ord("s"):
                if len(points) >= POINT_COUNT:
                    print("Already captured 8 points; press Q to finish.")
                    continue
                if selected is None:
                    print("Click the CENTER of one physical mark before pressing S.")
                    continue
                x, y = selected
                if not (0 <= x < w and 0 <= y < h):
                    print("Clicked pixel outside image.")
                    continue
                print(f"Measuring P{len(points)+1:02d} at pixel {(x,y)}...")
                result, error = capture_xyz(zed, runtime, cloud, x, y)
                if error:
                    print("NOT saved:", error)
                    continue
                result["point_id"] = f"P{len(points)+1:02d}"
                result["pixel_xy"] = [x, y]
                points.append(result)
                selected = None
                print("Accepted:", result)
                save_results(folder, last_image)
                if len(points) == POINT_COUNT:
                    print("All eight recorded. Press Q when ready.")
    finally:
        save_results(folder, last_image)
        zed.close()
        cv2.destroyAllWindows()
        print("Camera closed. No robot commands were sent.")


if __name__ == "__main__":
    main()
