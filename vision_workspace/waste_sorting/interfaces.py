"""Future integration contracts. Preview is a task outline, never a trajectory."""
import json
from pathlib import Path
from typing import Protocol

from .decision import assess_target
from .zones import timestamp


class Detector(Protocol):
    def detect_waste(self, image_bgr) -> tuple[str, object]:
        """Original-resolution JSON string plus annotated BGR image."""
        ...


class CameraLocalizer(Protocol):
    def localize(self, center_pixel: list[float]) -> list[float] | None:
        """Return matching-frame ZED IMAGE XYZ in meters, or None."""
        ...


class BaseTransform(Protocol):
    def camera_to_base(self, camera_xyz_m: list[float]) -> list[float]:
        """Requires measured extrinsics and independent validation; not implemented."""
        ...


class MotionPlanner(Protocol):
    def preview(self, validated_base_target: dict) -> dict: ...


class Gripper(Protocol):
    def command(self, action: str) -> dict: ...


class DryRunRobot:
    def preview(self, detection, workspace, camera_xyz_m=None):
        return {
            "mode": "dry_run", "timestamp_utc": timestamp(),
            "decision": assess_target(detection, workspace, camera_xyz_m),
            "task_outline": ["localize", "transform_to_base", "validate_grasp_and_drop",
                             "plan_and_review", "grasp", "place"],
            "trajectory": None, "execution_status": "disabled",
        }

    def execute(self, request):
        raise RuntimeError("Robot execution disabled: no verified robot transport exists")

    def command(self, action):
        raise RuntimeError("Gripper execution disabled: no verified gripper transport exists")


def record_preview(path, preview):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(preview, allow_nan=False, ensure_ascii=False) + "\n")
