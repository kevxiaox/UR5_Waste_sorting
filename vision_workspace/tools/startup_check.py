"""Read-only startup checks. --frame optionally acquires one ZED RGB frame, no GUI."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

from waste_sorting.camera import ZedSource, DLL_HANDLES

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frame", action="store_true")
    args = parser.parse_args()
    print("Python:", sys.version.split()[0], "| executable:", sys.executable)
    print("OpenCV:", cv2.__version__, "| NumPy:", np.__version__)
    session = ROOT / "calibration/sessions/20261021_140123/camera_points.json"
    payload = json.loads(session.read_text(encoding="utf-8"))
    points = np.asarray([p["camera_xyz_m"] for p in payload["points"]])
    center = points.mean(axis=0)
    _, _, vt = np.linalg.svd(points - center)
    residuals_mm = (points - center) @ vt[-1] * 1000
    print(f"Archived camera points: {len(points)} | plane RMS {np.sqrt(np.mean(residuals_mm**2)):.3f} mm"
          f" | max {max(abs(residuals_mm)):.3f} mm")
    print("Plane residuals are NOT absolute localization accuracy or robot calibration.")
    import os
    for folder in (r"C:\Program Files (x86)\ZED SDK\bin", r"D:\Robotique\bin\x64"):
        DLL_HANDLES.append(os.add_dll_directory(folder))
    import pyzed.sl as sl
    print("ZED SDK:", sl.Camera.get_sdk_version())
    devices = sl.Camera.get_device_list()
    print("Available ZED devices:", devices)
    if not devices:
        print("No openable camera found. Replug USB; close other ZED apps; retry.")
        return 2
    if args.frame:
        with ZedSource() as camera:
            frame = camera.read()
            print("LIVE RGB frame:", frame.shape, "| camera serial:", camera.serial)
    print("Robot execution: DISABLED. No robot or gripper connection attempted.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print("Startup check failed:", exc)
        raise SystemExit(1)
