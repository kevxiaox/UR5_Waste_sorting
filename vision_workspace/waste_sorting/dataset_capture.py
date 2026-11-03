"""Model-free RGB evidence writer. Scenario intent is never a reviewed label."""
import copy
import hashlib
import json
import math
from pathlib import Path

import cv2
import numpy as np

SCENARIOS = ('empty_pick', 'outside_known', 'unknown_in_pick')


class DatasetCapture:
    def __init__(self, folder, workspace, config_hash, scenario, planned):
        if scenario not in SCENARIOS or type(planned) is not int or not 1 <= planned <= 20:
            raise ValueError('Invalid capture scenario or count')
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=False)
        self.workspace = copy.deepcopy(workspace)
        self.workspace['camera_pose_revalidated'] = False
        self.last_timestamp = None
        self.data = {'type': 'unfiltered_rgb_evaluation_capture', 'scenario_intent': scenario,
                     'planned_attempts': planned, 'workspace_sha256': config_hash,
                     'workspace_snapshot': self.workspace, 'attempts': [], 'status': 'waiting_alignment',
                     'label_review_status': 'unreviewed', 'expected_classes_in_pick_zone': None,
                     'robot_execution_enabled': False,
                     'note': 'No model loaded. All attempts retained, including failures. '
                             'Same scene frames are correlated; not independent trials. '
                             'Scenario is operator intent, not ground truth. No depth or robot connection.'}
        self.flush()

    def flush(self):
        path = self.folder/'session.json'
        temporary = self.folder/'session.tmp'
        temporary.write_text(json.dumps(self.data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        temporary.replace(path)

    def confirm_alignment(self):
        self.workspace['camera_pose_revalidated'] = True
        self.data['status'] = 'ready'
        self.flush()

    def record(self, frame, sensor_timestamp_ns, host_time):
        if not self.workspace['camera_pose_revalidated']:
            raise ValueError('Session alignment confirmation required')
        if len(self.data['attempts']) >= self.data['planned_attempts']:
            raise ValueError('Planned sample attempts exhausted')
        index = len(self.data['attempts'])+1
        attempt = {'attempt': index, 'status': 'failed', 'expected_classes_in_pick_zone': None,
                   'label_review_status': 'unreviewed'}
        self.data['attempts'].append(attempt)
        try:
            if not math.isfinite(host_time):
                raise ValueError('Invalid host timestamp')
            attempt['host_time_s'] = host_time
            if (type(sensor_timestamp_ns) is not int or sensor_timestamp_ns <= 0
                    or (self.last_timestamp is not None and sensor_timestamp_ns <= self.last_timestamp)):
                raise ValueError('Invalid, duplicate or reversed sensor timestamp')
            attempt['sensor_timestamp_ns'] = sensor_timestamp_ns
            self.last_timestamp = sensor_timestamp_ns
            w, h = self.workspace['image_size']
            if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.shape != (h, w, 3):
                raise ValueError('Original RGB size/type mismatch')
            path = self.folder/f'rgb_{index:02d}.png'
            if not cv2.imwrite(str(path), frame):
                raise OSError('Image write failed')
            attempt.update(status='saved', image_path=str(path.resolve()),
                           image_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        except (ValueError, OSError, cv2.error) as error:
            attempt['error'] = str(error)
        self.flush()
        return attempt

    def finish(self, status):
        self.data['status'] = status
        self.data['saved_images'] = sum(a['status'] == 'saved' for a in self.data['attempts'])
        self.flush()
