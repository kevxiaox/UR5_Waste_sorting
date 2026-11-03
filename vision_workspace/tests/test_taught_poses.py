"""Review-only pose capture cannot turn into a validated trajectory."""
import copy
import unittest

from waste_sorting.taught_poses import build_pose_record, capture_pose
from unittest.mock import patch


def snapshots():
    result=[]
    for i in range(3):
        result.append({'robot_ip':'test-only','controller_version':'3.15.5.0',
                       'controller_timestamp_s':100+i,'host_sample_monotonic_s':10+i,
                       'actual_q_rad':[0.1]*6,'actual_qd_rad_s':[0.0]*6,
                       'actual_tcp_pose_base_m_rotvec_rad':[0.1,0.2,0.7,0,0,0],
                       'active_tcp_offset_flange_m_rotvec_rad':[0,0,0.15,0,0,0],
                       'runtime_state':1,'robot_status_bits':1,'safety_mode':1})
    return result


class TaughtPoseTests(unittest.TestCase):
    def test_distinct_grasp_release_labels_remain_intent_only(self):
        for label in ('PICK_GRASP', 'LEFT_DROP_RELEASE', 'RIGHT_DROP_RELEASE', 'TCP_CHECK_GRASP'):
            with self.subTest(label=label):
                with patch('waste_sorting.taught_poses.read_tool_io_state',
                           side_effect=[snapshots()[0], snapshots()[2]]) as io_read, \
                     patch('waste_sorting.taught_poses.read_robot_state',
                           return_value=snapshots()[1]) as pose_read:
                    record = capture_pose(label, 'offline-test')
                self.assertEqual(record['label'], label)
                self.assertEqual(record['label_scope'], 'operator_intent_only_not_physical_verification')
                self.assertEqual(io_read.call_count, 2)
                pose_read.assert_called_once_with('offline-test')
                for flag in ('robot_motion_commands_enabled', 'sorting_execution_enabled',
                             'tcp_verified', 'path_validated', 'automatic_replay_enabled'):
                    self.assertFalse(record[flag])

    def test_stationary_posture_is_review_record_only(self):
        record=build_pose_record('HOME',*snapshots())
        self.assertEqual(record['actual_q_rad'],[0.1]*6)
        for flag in ('robot_motion_commands_enabled','sorting_execution_enabled','tcp_verified',
                     'path_validated','automatic_replay_enabled'):
            self.assertFalse(record[flag])

    def test_moving_running_wrong_identity_or_time_rejected(self):
        for index,patch in ((0,{'actual_qd_rad_s':[0.02]*6}),
                            (2,{'runtime_state':2}), (2,{'robot_status_bits':3}),
                            (2,{'robot_ip':'other'}),(2,{'controller_timestamp_s':99}),
                            (2,{'safety_mode':3}),(1,{'actual_q_rad':[0.2]*6})):
            values=snapshots()
            values[index].update(patch)
            with self.assertRaises(ValueError):
                build_pose_record('HOME',*values)
        with self.assertRaises(ValueError):
            build_pose_record('unknown',*snapshots())


if __name__ == '__main__':
    unittest.main()
