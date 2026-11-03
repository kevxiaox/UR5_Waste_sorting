"""Single-object temporal surface diagnostics. Never grants execution permission."""
from collections import deque
import math

import numpy as np


class TemporalSurfaceObserver:
    def __init__(self, frames=20):
        if type(frames) is not int or frames < 2:
            raise ValueError('Need at least two frames')
        self.frames = frames
        self.samples = deque(maxlen=frames)
        self.previous = None

    def reset(self):
        self.samples.clear()
        self.previous = None

    def observe(self, result, host_time, sensor_timestamp_ns):
        report = {'status': 'collecting', 'valid_processed_frames': 0,
                  'required_frames': self.frames, 'samples': [],
                  'limits': {'max_gap_s': 1.0, 'max_window_s': 10.0,
                             'max_center_shift_px': 8.0, 'min_bbox_iou': .5,
                             'max_axis_std_mm': 5.0, 'max_axis_span_mm': 20.0},
                  'scope': 'single_visible_surface_diagnostic_only',
                  'camera_dropped_frames_assessed': False,
                  'physical_object_identity_verified': False,
                  'association_method': 'single_candidate_class_route_bbox_and_pixel_proximity',
                  'tcp_verified': False, 'robot_execution_enabled': False,
                  'grasp_pose': None, 'trajectory': None}

        def reject(reason):
            self.samples.clear()
            return dict(report, status=reason)

        frame_id = result.get('frame_id')
        if (type(frame_id) is not int or not math.isfinite(host_time)
                or type(sensor_timestamp_ns) is not int or sensor_timestamp_ns <= 0):
            self.reset()
            return reject('reset_invalid_timing')
        current = (frame_id, host_time, sensor_timestamp_ns)
        prior = self.previous
        self.previous = current
        if prior and (frame_id != prior[0]+1 or host_time <= prior[1]
                      or sensor_timestamp_ns <= prior[2]):
            return reject('reset_nonmonotonic_or_duplicate_frame')
        reset_reason = None
        if prior and host_time-prior[1] > 1.0:
            self.samples.clear()
            reset_reason = 'gap_over_one_second'
        items = result.get('items', [])
        if len(items) != 1 or items[0].get('status') != 'candidate_surface_preview_only':
            return reject('reset_requires_exactly_one_valid_candidate')
        item = items[0]
        raw = item['detector_item']
        point = np.asarray(item.get('camera_xyz_m'), dtype=float)
        center = np.asarray(raw.get('center_pixel'), dtype=float)
        box = np.asarray(raw.get('bbox_xyxy'), dtype=float)
        if (point.shape != (3,) or not np.isfinite(point).all() or point[2] <= 0
                or center.shape != (2,) or not np.isfinite(center).all()
                or box.shape != (4,) or not np.isfinite(box).all()
                or box[2] <= box[0] or box[3] <= box[1]):
            return reject('reset_invalid_sample_geometry')
        identity = (raw.get('class_name'), item.get('category'), item.get('target_zone'),
                    tuple(result.get('image_resolution', [])))
        if self.samples:
            anchor = self.samples[0]
            a = np.array(anchor['bbox_xyxy'])
            low, high = np.maximum(a[:2], box[:2]), np.minimum(a[2:], box[2:])
            intersection = float(np.prod(np.maximum(0, high-low)))
            union = float(np.prod(a[2:]-a[:2])+np.prod(box[2:]-box[:2])-intersection)
            if (identity != anchor['identity'] or np.linalg.norm(center-anchor['center_pixel']) > 8
                    or intersection/union < .5 or host_time-anchor['host_time_s'] > 10):
                self.samples.clear()
                reset_reason = 'identity_position_or_window_changed'
        self.samples.append({'frame_id': frame_id, 'host_time_s': host_time,
                             'sensor_timestamp_ns': sensor_timestamp_ns, 'identity': identity,
                             'center_pixel': center.tolist(), 'bbox_xyxy': box.tolist(),
                             'camera_xyz_m': point.tolist(),
                             'candidate_base_xyz_m': item.get('candidate_base_xyz_m')})
        samples = list(self.samples)
        points = np.array([s['camera_xyz_m'] for s in samples])
        std = points.std(axis=0)*1000
        span = np.ptp(points, axis=0)*1000
        status = ('collecting' if len(samples) < self.frames else
                  'stable_surface_observation_only' if max(std) <= 5 and max(span) <= 20 else
                  'unstable_surface_observation')
        report.update(status=status, valid_processed_frames=len(samples), samples=samples,
                      reset_reason=reset_reason, camera_axis_std_mm=std.tolist(),
                      camera_axis_span_mm=span.tolist(), window_seconds=host_time-samples[0]['host_time_s'])
        return report
