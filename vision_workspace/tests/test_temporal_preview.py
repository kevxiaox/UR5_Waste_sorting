import unittest

from waste_sorting.temporal_preview import TemporalSurfaceObserver


def frame(index, z=1.0):
    return {'frame_id': index, 'image_resolution': [1280, 720], 'items': [{
        'status': 'candidate_surface_preview_only', 'camera_xyz_m': [.1, .2, z],
        'candidate_base_xyz_m': [.3, .4, .5], 'category': 'residual', 'target_zone': 'RIGHT_CELL',
        'detector_item': {'class_name': 'sponge', 'center_pixel': [100, 100],
                          'bbox_xyxy': [80, 80, 120, 120]}}]}


class TemporalPreviewTests(unittest.TestCase):
    def observe(self, observer, index, result=None):
        return observer.observe(result or frame(index), index*.1, index*100000000)

    def test_twenty_distinct_samples_do_not_enable_grasp_or_motion(self):
        observer = TemporalSurfaceObserver()
        for index in range(1, 21):
            result = self.observe(observer, index)
            if index < 20:
                self.assertEqual(result['status'], 'collecting')
        self.assertEqual(result['status'], 'stable_surface_observation_only')
        self.assertEqual([s['frame_id'] for s in result['samples']], list(range(1, 21)))
        self.assertFalse(result['robot_execution_enabled'])
        self.assertFalse(result['tcp_verified'])
        self.assertFalse(result['camera_dropped_frames_assessed'])
        self.assertIsNone(result['grasp_pose'])

    def test_depth_jitter_is_not_reported_as_stable(self):
        observer = TemporalSurfaceObserver()
        for index in range(1, 21):
            result = self.observe(observer, index, frame(index, 1+(.02 if index % 2 else -.02)))
        self.assertEqual(result['status'], 'unstable_surface_observation')

    def test_invalid_frame_or_multiple_objects_discards_old_window(self):
        for change in ('empty', 'multiple', 'rejected', 'nan'):
            observer = TemporalSurfaceObserver()
            self.observe(observer, 1)
            bad = frame(2)
            if change == 'empty':
                bad['items'] = []
            elif change == 'multiple':
                bad['items'] *= 2
            elif change == 'rejected':
                bad['items'][0]['status'] = 'rejected_low_confidence'
            else:
                bad['items'][0]['camera_xyz_m'][2] = float('nan')
            result = self.observe(observer, 2, bad)
            self.assertEqual(result['valid_processed_frames'], 0)
            self.assertEqual(self.observe(observer, 3)['valid_processed_frames'], 1)

    def test_duplicate_sensor_timestamp_and_frame_sequence_cannot_fill_window(self):
        observer = TemporalSurfaceObserver()
        self.observe(observer, 1)
        result = observer.observe(frame(2), .2, 100000000)
        self.assertEqual(result['status'], 'reset_nonmonotonic_or_duplicate_frame')
        self.assertEqual(result['samples'], [])
        result = observer.observe(frame(4), .4, 400000000)
        self.assertEqual(result['valid_processed_frames'], 0)

    def test_class_change_and_slow_pixel_drift_restart_collection(self):
        observer = TemporalSurfaceObserver()
        self.observe(observer, 1)
        changed = frame(2)
        changed['items'][0]['detector_item']['class_name'] = 'paper_cup'
        self.assertEqual(self.observe(observer, 2, changed)['valid_processed_frames'], 1)
        observer.reset()
        for index in range(1, 11):
            moved = frame(index)
            moved['items'][0]['detector_item']['center_pixel'][0] += index-1
            result = self.observe(observer, index, moved)
        self.assertEqual(result['valid_processed_frames'], 1)
        self.assertEqual(result['reset_reason'], 'identity_position_or_window_changed')

    def test_long_pause_and_revalidation_discard_previous_samples(self):
        observer = TemporalSurfaceObserver()
        self.observe(observer, 1)
        result = observer.observe(frame(2), 2, 200000000)
        self.assertEqual(result['valid_processed_frames'], 1)
        self.assertEqual(result['reset_reason'], 'gap_over_one_second')
        observer.reset()
        self.assertEqual(self.observe(observer, 3)['valid_processed_frames'], 1)
