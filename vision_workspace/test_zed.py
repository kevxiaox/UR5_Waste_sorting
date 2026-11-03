
import os
import math

# ============================================================
# UR5 Waste Sorting Robot
# Test 01: ZED Mini RGB, Depth and XYZ Acquisition
# ============================================================

# ------------------------------------------------------------
# 1. Configure DLL paths
# ------------------------------------------------------------

ZED_SDK_PATH = r"C:\Program Files (x86)\ZED SDK\bin"
CUDA_PATH = r"D:\Robotique\bin\x64"

for path in [ZED_SDK_PATH, CUDA_PATH]:
    if not os.path.isdir(path):
        raise FileNotFoundError(
            f"Required DLL directory not found: {path}"
        )

# Keep handles alive for the entire program.
dll_handles = [
    os.add_dll_directory(ZED_SDK_PATH),
    os.add_dll_directory(CUDA_PATH)
]

import pyzed.sl as sl


# ------------------------------------------------------------
# 2. Initialize ZED Mini
# ------------------------------------------------------------

zed = sl.Camera()

init = sl.InitParameters()

# Camera settings
init.camera_resolution = sl.RESOLUTION.HD720
init.camera_fps = 30

# Use current depth mode
init.depth_mode = sl.DEPTH_MODE.NEURAL

# All distances are measured in meters
init.coordinate_units = sl.UNIT.METER

# Camera coordinate system:
# X -> right
# Y -> down
# Z -> forward
init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE

print("=" * 55)
print("UR5 WASTE SORTING - ZED MINI TEST")
print("=" * 55)

print("\nOpening ZED Mini...")

status = zed.open(init)

if status != sl.ERROR_CODE.SUCCESS:
    print("ERROR: Camera open failed!")
    print("Status:", status)
    raise SystemExit(1)

print("Camera opened successfully!")


# ------------------------------------------------------------
# 3. Prepare image, depth and point cloud
# ------------------------------------------------------------

runtime = sl.RuntimeParameters()

image = sl.Mat()
depth = sl.Mat()
point_cloud = sl.Mat()


# ------------------------------------------------------------
# 4. Acquire camera data
# ------------------------------------------------------------

try:

    frame_acquired = False

    for attempt in range(60):

        grab_status = zed.grab(runtime)

        if grab_status != sl.ERROR_CODE.SUCCESS:
            continue

        # Retrieve left RGB image
        zed.retrieve_image(
            image,
            sl.VIEW.LEFT
        )

        # Retrieve depth map
        zed.retrieve_measure(
            depth,
            sl.MEASURE.DEPTH
        )

        # Retrieve 3D point cloud
        zed.retrieve_measure(
            point_cloud,
            sl.MEASURE.XYZ
        )

        frame_acquired = True
        break

    if not frame_acquired:
        print("ERROR: Failed to acquire a camera frame.")

    else:

        print("\nCamera data acquired successfully!")

        # ----------------------------------------------------
        # 5. Image information
        # ----------------------------------------------------

        width = image.get_width()
        height = image.get_height()

        print("\n--- RGB IMAGE ---")

        print("Resolution:", width, "x", height)

        # Center pixel
        cx = width // 2
        cy = height // 2

        print("Center pixel:", (cx, cy))

        # ----------------------------------------------------
        # 6. Read depth
        # ----------------------------------------------------

        depth_error, distance = depth.get_value(cx, cy)

        print("\n--- DEPTH INFORMATION ---")

        print("Depth status:", depth_error)

        if (
            depth_error == sl.ERROR_CODE.SUCCESS
            and math.isfinite(float(distance))
            and float(distance) > 0
        ):
            print(
                f"Depth at center: {float(distance):.4f} m"
            )
        else:
            print("Invalid depth value at center pixel.")

        # ----------------------------------------------------
        # 7. Read 3D coordinates
        # ----------------------------------------------------

        xyz_error, xyz = point_cloud.get_value(cx, cy)

        print("\n--- 3D COORDINATES ---")

        print("XYZ status:", xyz_error)

        if xyz_error == sl.ERROR_CODE.SUCCESS:

            x = float(xyz[0])
            y = float(xyz[1])
            z = float(xyz[2])

            if all(math.isfinite(v) for v in [x, y, z]):

                print(f"X = {x:.4f} m")
                print(f"Y = {y:.4f} m")
                print(f"Z = {z:.4f} m")

                print("\n3D Position (camera frame):")

                print(
                    f"P_camera = [{x:.4f}, "
                    f"{y:.4f}, {z:.4f}] m"
                )

            else:
                print("Invalid XYZ coordinates.")

        else:
            print("Failed to retrieve XYZ coordinates.")

        # ----------------------------------------------------
        # 8. Summary
        # ----------------------------------------------------

        print("\n--- TEST SUMMARY ---")

        print("RGB image: acquired")
        print("Depth measurement: checked")
        print("3D point cloud: checked")

        print("\nCoordinate system: CAMERA FRAME")

        print(
            "Note: These are NOT robot base coordinates."
        )

finally:

    # --------------------------------------------------------
    # 9. Close camera
    # --------------------------------------------------------

    zed.close()

    print("\nCamera closed.")

    print("=" * 55)
    print("TEST FINISHED")
    print("=" * 55)
