"""ZED settings reused from the already working project scripts."""
import os

import cv2
import numpy as np

DLL_HANDLES = []


class ZedSource:
    def __init__(self):
        self.camera = None

    def __enter__(self):
        # Windows DLL folders: the ZED SDK one is required, the others are added when present
        # (D:\Robotique\bin\x64 on the lab PC). ZED_DLL_DIRS (os.pathsep-separated) overrides.
        if os.environ.get("ZED_DLL_DIRS"):
            folders = os.environ["ZED_DLL_DIRS"].split(os.pathsep)
        else:
            folders = [os.path.join(os.environ.get("ZED_SDK_ROOT_DIR", r"C:\Program Files (x86)\ZED SDK"), "bin"),
                       r"D:\Robotique\bin\x64"]
        if hasattr(os, "add_dll_directory"):  # Windows only
            if not os.path.isdir(folders[0]):
                raise FileNotFoundError(f"Missing ZED SDK DLL directory: {folders[0]} "
                                        "(install the ZED SDK or set ZED_DLL_DIRS)")
            DLL_HANDLES.extend(os.add_dll_directory(f) for f in folders if os.path.isdir(f))
        import pyzed.sl as sl
        self.sl = sl
        self.camera = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD720
        init.camera_fps = 30
        init.depth_mode = sl.DEPTH_MODE.NEURAL
        init.coordinate_units = sl.UNIT.METER
        init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
        result = self.camera.open(init)
        if result != sl.ERROR_CODE.SUCCESS:
            self.camera.close()
            raise RuntimeError(f"ZED open failed: {result}; check USB and other camera apps")
        try:
            self.runtime = sl.RuntimeParameters()
            self.image = sl.Mat()
            self.cloud = sl.Mat()
            self.serial = int(self.camera.get_camera_information().serial_number)
        except Exception:
            self.camera.close()
            raise
        return self

    def read(self):
        return self._read_capture(with_xyz=False)

    def read_with_xyz(self):
        """Return BGR and an owned XYZ array from the SAME successful grab."""
        return self._read_capture(with_xyz=True)

    def _read_capture(self, with_xyz):
        # Bounded retry, including when disconnected after opening.
        sl = self.sl
        for _ in range(60):
            if self.camera.grab(self.runtime) == sl.ERROR_CODE.SUCCESS:
                result = self.camera.retrieve_image(self.image, sl.VIEW.LEFT)
                if result != sl.ERROR_CODE.SUCCESS:
                    continue
                if with_xyz:
                    result = self.camera.retrieve_measure(self.cloud, sl.MEASURE.XYZ)
                    if result != sl.ERROR_CODE.SUCCESS:
                        continue
                raw = self.image.get_data()
                bgr = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR) if raw.shape[2] == 4 else raw.copy()
                if with_xyz:
                    xyz = np.array(self.cloud.get_data()[..., :3], dtype=np.float32, copy=True)
                    if xyz.shape[:2] != bgr.shape[:2]:
                        raise RuntimeError("RGB/XYZ resolutions differ; refusing pixel lookup")
                    return bgr, xyz
                return bgr
        raise RuntimeError("ZED frame acquisition failed 60 times; camera closed")

    def __exit__(self, *args):
        self.camera.close()
