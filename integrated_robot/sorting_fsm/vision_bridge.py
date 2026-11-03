"""Vision brick -> PickRequest: the team's YOLO detector output becomes one task for the machine.

Input: the JSON string returned by the team's `detect_waste(image_bgr)`
(vision/detector.py):
  {"image_resolution": [w, h], "total_detected": n,
   "items": [{"class_name", "waste_category", "confidence", "bbox_xyxy", "center_pixel", "status"}]}

VisionGate (fail closed, same rules as the team's decision.py / task_review.py):
  * detector status OK, class in the sorting policy, confidence >= the POLICY threshold (0.60; the
    detector's own 0.45 only marks boxes), waste_category consistent with the policy category;
  * box centre strictly inside PICK_ZONE (pixel polygon of configs/workspace_zones.json) and frame
    size equal to the workspace's; camera/table alignment confirmed by the operator this session;
  * exactly one candidate in the pick zone (two OK detections there = ambiguous, nothing picked);
  * the same class with a centre moving <= max_center_shift_px for frames_required consecutive
    frames (one bad frame restarts the count).
A detection only says "this class is probably there"; it is not a grasp pose.

Placement: where the object is in the robot base frame.
  * FixedPlacement (option A): the taught pick spot of the class (taught_route); the operator
    confirms the object was put there. The camera position is not used.
  * MeasuredPlacement (option B): camera XYZ at the box centre (ZED point cloud of the same frame)
    mapped by a camera -> base transform. Refused unless that transform file says validated: true
    (the team's current fit is an unvalidated candidate).
"""
from dataclasses import dataclass
import json
import math
from pathlib import Path

from .cell import make_request, point_in_polygon

EXPECTED_WASTE_CATEGORY = {'recyclable': 'Recyclable', 'residual': 'General_Waste'}


@dataclass(frozen=True)
class Candidate:
    class_name: str
    category: str
    target_zone: str
    confidence: float
    center_pixel: tuple
    bbox_xyxy: tuple
    frame_id: int


@dataclass(frozen=True)
class GateResult:
    status: str                  # ready | collecting | no_candidate | ambiguous | camera_not_confirmed
    candidate: Candidate | None  # set for ready / collecting
    stable_frames: int
    rejected: tuple              # (item index, class, reason) for every item not retained


def parse_detector_output(raw, image_size=None):
    """Items of a detect_waste JSON string, structure checked. Raises ValueError."""
    def no_constants(value):
        raise ValueError(f'Non-finite value in detector JSON: {value}')
    data = json.loads(raw, parse_constant=no_constants) if isinstance(raw, str) else raw
    if not isinstance(data, dict) or not isinstance(data.get('items'), list):
        raise ValueError('Detector output needs an items list')
    if type(data.get('total_detected')) is not int or data['total_detected'] != len(data['items']):
        raise ValueError('Detector total_detected differs from the number of items')
    if image_size is not None and data.get('image_resolution') != list(image_size):
        raise ValueError(f"Detector resolution {data.get('image_resolution')} differs from the workspace {list(image_size)}")
    return data['items']


def _numbers(value, n):
    return (isinstance(value, list) and len(value) == n
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


class VisionGate:
    def __init__(self, policy, workspace, *, frames_required=5, max_center_shift_px=8.0):
        if type(frames_required) is not int or frames_required < 1:
            raise ValueError('frames_required must be a positive integer')
        if not (type(max_center_shift_px) in (int, float) and 0 < max_center_shift_px <= 100):
            raise ValueError('max_center_shift_px must be in (0, 100]')
        zone = workspace.get('zones', {}).get('PICK_ZONE', {}).get('polygon_px')
        if not isinstance(zone, list) or len(zone) < 3:
            raise ValueError('workspace needs a PICK_ZONE polygon_px')
        self.policy, self.image_size, self.pick_zone = policy, workspace['image_size'], zone
        self.threshold = policy['min_confidence']
        self.frames_required, self.max_shift = frames_required, float(max_center_shift_px)
        self.camera_confirmed = False
        self.reset()

    def confirm_camera_alignment(self):
        """Operator checked the zone overlay on the live image (team's 'V' key)."""
        self.camera_confirmed = True
        self.reset()

    def reset(self):
        self._first, self._count = None, 0

    def _check_item(self, item):
        """(Candidate fields, None) or (None, reason)."""
        if not isinstance(item, dict):
            return None, 'not an object'
        if item.get('status') != 'OK':
            return None, f"detector status {item.get('status')}"
        name = item.get('class_name')
        category = self.policy['class_to_category'].get(name)
        if category is None:
            return None, 'class not in the sorting policy'
        confidence = item.get('confidence')
        if type(confidence) not in (int, float) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            return None, 'invalid confidence'
        if confidence < self.threshold:
            return None, f'confidence {confidence:.2f} < policy {self.threshold:.2f}'
        if item.get('waste_category') != EXPECTED_WASTE_CATEGORY[category]:
            return None, 'detector category differs from the sorting policy'
        bbox, center = item.get('bbox_xyxy'), item.get('center_pixel')
        if not _numbers(bbox, 4) or not _numbers(center, 2):
            return None, 'invalid box'
        (x1, y1, x2, y2), (u, v) = bbox, center
        w, h = self.image_size
        if not (0 <= x1 < x2 < w and 0 <= y1 < y2 < h and x1 <= u <= x2 and y1 <= v <= y2):
            return None, 'box outside the image or centre outside the box'
        if not point_in_polygon(u, v, self.pick_zone):
            return None, 'outside PICK_ZONE'
        zone = self.policy['category_to_zone'][category]
        return (name, category, zone, float(confidence), tuple(center), tuple(bbox)), None

    def observe(self, raw, frame_id):
        """Feed one frame's detector output; returns the gate decision for this frame."""
        if not self.camera_confirmed:
            self.reset()
            return GateResult('camera_not_confirmed', None, 0, ())
        items = parse_detector_output(raw, self.image_size)
        kept, rejected = [], []
        for index, item in enumerate(items):
            fields, reason = self._check_item(item)
            if fields is None:
                rejected.append((index, item.get('class_name') if isinstance(item, dict) else None, reason))
            else:
                kept.append(Candidate(*fields, frame_id))
        if not kept:
            self.reset()
            return GateResult('no_candidate', None, 0, tuple(rejected))
        if len(kept) > 1:
            self.reset()
            return GateResult('ambiguous', None, 0, tuple(rejected))
        candidate = kept[0]
        first = self._first
        if (first is None or first.class_name != candidate.class_name
                or math.dist(first.center_pixel, candidate.center_pixel) > self.max_shift):
            self._first, self._count = candidate, 1
        else:
            self._count += 1
        status = 'ready' if self._count >= self.frames_required else 'collecting'
        return GateResult(status, candidate, self._count, tuple(rejected))


class FixedPlacement:
    """Option A: the object is on its class's taught pick spot (operator confirmation)."""
    mode = 'fixed_placement'
    needs_operator_confirmation = True

    def __init__(self, route):
        self.route = route

    def locate(self, candidate, camera_xyz_m=None):
        return tuple(self.route.grasp_position(candidate.class_name))


class CameraToBase:
    """Rigid camera -> UR base transform from a JSON file:
    {"schema_version": 1, "T_base_from_camera": 4x4, "validated": bool, "method": str, ...}."""
    def __init__(self, matrix, *, validated, source=''):
        if (not isinstance(matrix, list) or len(matrix) != 4 or not all(_numbers(row, 4) for row in matrix)
                or [float(v) for v in matrix[3]] != [0.0, 0.0, 0.0, 1.0]):
            raise ValueError('T_base_from_camera must be a finite 4x4 homogeneous matrix')
        r = [row[:3] for row in matrix[:3]]
        for i in range(3):
            for j in range(3):
                if abs(sum(r[k][i] * r[k][j] for k in range(3)) - (1.0 if i == j else 0.0)) > 1e-6:
                    raise ValueError('T_base_from_camera rotation is not orthonormal')
        det = (r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
               + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]))
        if abs(det - 1.0) > 1e-6:
            raise ValueError('T_base_from_camera rotation must be proper (det +1)')
        self.matrix, self.validated, self.source = matrix, validated is True, source

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding='utf-8'))
        if data.get('schema_version') != 1:
            raise ValueError('Unsupported camera->base file schema')
        return cls(data['T_base_from_camera'], validated=data.get('validated'), source=str(path))

    def apply(self, point):
        m = self.matrix
        return tuple(sum(m[i][k] * point[k] for k in range(3)) + m[i][3] for i in range(3))


class MeasuredPlacement:
    """Option B: camera XYZ of the box centre -> base frame. Needs a validated transform."""
    mode = 'measured_position'
    needs_operator_confirmation = False

    def __init__(self, camera_to_base):
        if not camera_to_base.validated:
            raise ValueError('camera->base transform is not validated: measured positions disabled '
                             '(use fixed placement, or validate the calibration first)')
        self.camera_to_base = camera_to_base

    def locate(self, candidate, camera_xyz_m=None):
        try:
            xyz = [float(v) for v in camera_xyz_m]   # ZED values are numpy floats
        except (TypeError, ValueError):
            xyz = None
        if xyz is None or len(xyz) != 3 or not all(math.isfinite(v) for v in xyz) or xyz[2] <= 0:
            raise ValueError('No valid camera depth at the box centre')
        return self.camera_to_base.apply(xyz)


def make_vision_request(task_id, candidate, placement, policy, camera_xyz_m=None):
    """PickRequest for a ready candidate; the drop zone comes from the policy, not the detector."""
    xyz = placement.locate(candidate, camera_xyz_m)
    source = (f'vision:{placement.mode}:frame{candidate.frame_id}:{candidate.class_name}'
              f':{candidate.confidence:.2f}@{list(candidate.center_pixel)}')
    return make_request(task_id, candidate.class_name, xyz, policy, source=source)
