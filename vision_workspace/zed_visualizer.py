
import os
import json
import math
from datetime import datetime
from pathlib import Path

# ============================================================
# UR5 Waste Sorting Robot
# ZED Mini Real-Time 3D Visualizer
#
# Controls:
#   Left click : Measure XYZ at selected pixel
#   S          : Save RGB, depth, XYZ and metadata
#   C          : Clear selected pixel
#   Q / ESC    : Quit
#
# IMPORTANT:
# Coordinates are in the CAMERA frame, NOT the UR5 base.
# This script NEVER commands robot movement.
# ============================================================

# 1. DLL configuration
ZED_DLL = r"C:\Program Files (x86)\ZED SDK\bin"
CUDA_DLL = r"D:\Robotique\bin\x64"

dll_handles = []

for folder in (ZED_DLL, CUDA_DLL):
    if not os.path.isdir(folder):
        raise FileNotFoundError(
            f"Missing DLL directory: {folder}"
        )
    dll_handles.append(os.add_dll_directory(folder))

import cv2
import numpy as np
import pyzed.sl as sl


# 2. Project configuration
OUTPUT_DIR = Path(
    r"D:\UR5_Waste_Sorting\data\captures"
)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

WINDOW_NAME = "UR5 - ZED Mini 3D Visualizer"

selected_pixel = None
pending_pixel = None


# 3. Mouse callback
def on_mouse(event, x, y, flags, userdata):
    global pending_pixel

    if event == cv2.EVENT_LBUTTONDOWN:
        pending_pixel = (x, y)


# 4. Read one 3D point
def measure_point(depth_mat, xyz_mat, x, y):
    depth_status, distance = depth_mat.get_value(x, y)
    xyz_status, point = xyz_mat.get_value(x, y)

    result = {
        "pixel": [int(x), int(y)],
        "valid": False,
        "depth_m": None,
        "xyz_m": None,
    }

    if (
        depth_status != sl.ERROR_CODE.SUCCESS
        or xyz_status != sl.ERROR_CODE.SUCCESS
    ):
        return result

    distance = float(distance)
    xyz = [float(point[i]) for i in range(3)]

    if (
        not math.isfinite(distance)
        or distance <= 0
        or not all(math.isfinite(v) for v in xyz)
    ):
        return result

    result["valid"] = True
    result["depth_m"] = distance
    result["xyz_m"] = xyz

    return result


# 5. Save captured data
def save_capture(bgr, depth_mat, xyz_mat, measurement):
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S_%f"
    )

    prefix = OUTPUT_DIR / f"zed_{timestamp}"

    rgb_path = str(prefix) + "_rgb.png"
    depth_path = str(prefix) + "_depth.npy"
    xyz_path = str(prefix) + "_xyz.npy"
    json_path = str(prefix) + "_metadata.json"

    # Save original image, without overlay graphics
    if not cv2.imwrite(rgb_path, bgr):
        raise IOError("Failed to save RGB image")

    # Save raw measurements in meters
    depth_array = np.array(
        depth_mat.get_data(),
        dtype=np.float32,
        copy=True
    )

    xyz_array = np.array(
        xyz_mat.get_data()[..., :3],
        dtype=np.float32,
        copy=True
    )

    np.save(depth_path, depth_array)
    np.save(xyz_path, xyz_array)

    metadata = {
        "timestamp_local": datetime.now().isoformat(),
        "camera": "ZED Mini",
        "resolution": [int(bgr.shape[1]), int(bgr.shape[0])],
        "coordinate_frame": "ZED_IMAGE_CAMERA",
        "coordinate_units": "meter",
        "depth_mode": "NEURAL",
        "measurement": measurement,
        "object_class": None,
        "waste_category": None,
        "calibrated_to_ur5": False,
        "note": (
            "Camera-frame measurement only. "
            "Not a validated robot grasp target."
        )
    }

    with open(json_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)

    print("\n[SAVED]")
    print("RGB:", rgb_path)
    print("Depth:", depth_path)
    print("XYZ:", xyz_path)
    print("Metadata:", json_path)


# 6. Main application
def main():
    global selected_pixel, pending_pixel

    print("=" * 60)
    print("UR5 WASTE SORTING - ZED VISUALIZER")
    print("=" * 60)
    print("Left click: XYZ measurement")
    print("S: Save capture")
    print("C: Clear selection")
    print("Q / ESC: Quit")

    zed = sl.Camera()

    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 30
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

    print("\nOpening ZED Mini...")

    status = zed.open(init)

    if status != sl.ERROR_CODE.SUCCESS:
        print("Failed to open camera:", status)
        return

    print("Camera opened successfully!")

    runtime = sl.RuntimeParameters()

    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()

    window_created = False

    try:
        cv2.namedWindow(
            WINDOW_NAME,
            cv2.WINDOW_NORMAL
        )

        window_created = True

        cv2.setMouseCallback(
            WINDOW_NAME,
            on_mouse
        )

        failures = 0

        while True:
            grab_status = zed.grab(runtime)

            if grab_status != sl.ERROR_CODE.SUCCESS:
                failures += 1

                if failures >= 60:
                    print(
                        "Too many failed camera frames:",
                        grab_status
                    )
                    break

                continue

            failures = 0

            zed.retrieve_image(image, sl.VIEW.LEFT)
            zed.retrieve_measure(
                depth, sl.MEASURE.DEPTH
            )
            zed.retrieve_measure(
                point_cloud, sl.MEASURE.XYZ
            )

            raw = image.get_data()

            # ZED images are BGRA on this API.
            if raw.shape[2] == 4:
                bgr = cv2.cvtColor(
                    raw, cv2.COLOR_BGRA2BGR
                )
            else:
                bgr = raw.copy()

            height, width = bgr.shape[:2]

            # Use image pixels for clicks and measurements.
            if pending_pixel is not None:
                px, py = pending_pixel
                pending_pixel = None

                if 0 <= px < width and 0 <= py < height:
                    selected_pixel = (px, py)

            measurement = None

            if selected_pixel is not None:
                x, y = selected_pixel
                measurement = measure_point(
                    depth, point_cloud, x, y
                )

            display = bgr.copy()

            # On-screen instructions
            cv2.putText(
                display,
                "Click: XYZ | S: Save | C: Clear | Q: Quit",
                (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            if measurement is not None:
                x, y = selected_pixel

                cv2.drawMarker(
                    display,
                    (x, y),
                    (0, 255, 255),
                    cv2.MARKER_CROSS,
                    24,
                    2
                )

                if measurement["valid"]:
                    px, py, pz = measurement["xyz_m"]
                    distance = measurement["depth_m"]

                    text1 = (
                        f"Pixel: ({x}, {y}) "
                        f"Depth: {distance:.3f} m"
                    )

                    text2 = (
                        f"Camera XYZ: "
                        f"{px:.3f}, {py:.3f}, {pz:.3f} m"
                    )

                else:
                    text1 = f"Pixel: ({x}, {y})"
                    text2 = "Invalid depth / XYZ"

                cv2.rectangle(
                    display,
                    (10, height - 94),
                    (min(width - 10, 700), height - 10),
                    (20, 20, 20),
                    -1
                )

                cv2.putText(
                    display,
                    text1,
                    (20, height - 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2
                )

                cv2.putText(
                    display,
                    text2,
                    (20, height - 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2
                )

            cv2.imshow(WINDOW_NAME, display)

            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            elif key == ord("c"):
                selected_pixel = None
                pending_pixel = None
                print("Selection cleared.")

            elif key == ord("s"):
                try:
                    save_capture(
                        bgr,
                        depth,
                        point_cloud,
                        measurement
                    )
                except Exception as exc:
                    print("Save failed:", exc)

    finally:
        zed.close()

        if window_created:
            cv2.destroyAllWindows()

        print("\nZED camera closed.")
        print("Program finished.")


if __name__ == "__main__":
    main()
