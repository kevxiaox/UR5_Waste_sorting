"""Fixed secondary script, read-only RTDE preflight and single-write semantics."""
import copy
from pathlib import Path
import struct
import tempfile
import unittest
from unittest.mock import MagicMock,patch

from test_robot_observations import FakeConnection,server_packet
from waste_sorting.gripper_test import build_gripper_script,check_preflight,run_gripper_test
from waste_sorting.rtde_readonly import TOOL_OUTPUT_NAMES,TOOL_OUTPUT_TYPES,read_tool_io_state


def state(level=False):
    return {'robot_ip':'test-only','controller_version':'3.15.5.0','runtime_state':1,
            'safety_mode':1,'robot_status_bits':1,'tool_output_voltage_v':24,
            'actual_qd_rad_s':[0.0]*6,'actual_q_rad':[0.0]*6,
            'tool_output_0':level,'tool_output_1':False}


class GripperIOTests(unittest.TestCase):
    def test_rtde_recipe_decodes_tool_bits_not_standard_outputs(self):
        values = [12.0,(1<<16)|1]+[0.0]*12+[1,1,1,24]
        replies = b''.join([server_packet(86,b'\x01'),server_packet(118,struct.pack('!4I',3,15,5,0)),
                            server_packet(79,b'\x03'+TOOL_OUTPUT_TYPES),server_packet(83,b'\x01'),
                            server_packet(85,b'\x03'+struct.pack('!dQ12diIIi',*values))])
        connection = FakeConnection(replies)
        with patch('waste_sorting.rtde_readonly.socket.create_connection',return_value=connection):
            result = read_tool_io_state('test-only')
        self.assertTrue(result['tool_output_0'])
        self.assertFalse(result['tool_output_1'])
        self.assertEqual(result['tool_output_voltage_v'],24)
        self.assertEqual([x[2] for x in connection.writes],[86,118,79,83])
        self.assertEqual(connection.writes[2][3:],struct.pack('!d',10.0)+TOOL_OUTPUT_NAMES)
        self.assertTrue(connection.closed)

    def test_exact_fixed_script_and_invalid_action(self):
        self.assertEqual(build_gripper_script('open'),
                         b'sec isolated_gripper_io_test():\n  set_tool_digital_out(0, False)\nend\n')
        self.assertEqual(build_gripper_script('close'),
                         b'sec isolated_gripper_io_test():\n  set_tool_digital_out(0, True)\nend\n')
        with self.assertRaises(ValueError):
            build_gripper_script('close\nmovej()')

    def test_preflight_rejects_program_motion_safety_voltage_and_wrong_version(self):
        check_preflight(state())
        for change in ({'runtime_state':2},{'robot_status_bits':3},{'safety_mode':3},
                       {'tool_output_voltage_v':12},{'actual_qd_rad_s':[0.02]*6},
                       {'controller_version':'5.19.0.0'}):
            with self.assertRaises(ValueError):
                check_preflight(dict(state(),**change))

    def test_default_preview_never_connects(self):
        with tempfile.TemporaryDirectory() as directory, \
             patch('waste_sorting.gripper_test.read_tool_io_state') as read, \
             patch('waste_sorting.gripper_test.socket.create_connection') as connect:
            result = run_gripper_test('close','test-only',Path(directory)/'audit.json')
        read.assert_not_called()
        connect.assert_not_called()
        self.assertFalse(result['write_attempted'])

    def test_send_one_command_only_and_verify_level_not_jaw_motion(self):
        connection = MagicMock()
        with tempfile.TemporaryDirectory() as directory, \
             patch('waste_sorting.gripper_test.read_tool_io_state',side_effect=[state(False),state(True)]), \
             patch('waste_sorting.gripper_test.socket.create_connection',return_value=connection) as connect, \
             patch('waste_sorting.gripper_test.time.sleep'):
            result = run_gripper_test('close','test-only',Path(directory)/'audit.json',send=True)
        connect.assert_called_once_with(('test-only',30002),timeout=3.0)
        connection.__enter__.return_value.sendall.assert_called_once_with(build_gripper_script('close'))
        self.assertEqual(result['status'],'output_level_readback_matches')
        self.assertFalse(result['gripper_motion_verified'])
        self.assertFalse(result['sorting_execution_enabled'])

    def test_no_write_when_preflight_fails_or_output_already_set(self):
        for snapshot in (dict(state(),runtime_state=2),state(True)):
            with tempfile.TemporaryDirectory() as directory, \
                 patch('waste_sorting.gripper_test.read_tool_io_state',return_value=snapshot), \
                 patch('waste_sorting.gripper_test.socket.create_connection') as connect:
                result = run_gripper_test('close','test-only',Path(directory)/'audit.json',send=True)
            connect.assert_not_called()
            self.assertFalse(result['write_attempted'])

    def test_post_send_failure_never_resends_or_resets(self):
        connection = MagicMock()
        for replies in ([state(False),OSError('read failed')],[state(False) for _ in range(4)]):
            with tempfile.TemporaryDirectory() as directory, \
                 patch('waste_sorting.gripper_test.read_tool_io_state',side_effect=replies), \
                 patch('waste_sorting.gripper_test.socket.create_connection',return_value=connection) as connect, \
                 patch('waste_sorting.gripper_test.time.sleep'):
                result = run_gripper_test('close','test-only',Path(directory)/'audit.json',send=True)
            connect.assert_called_once()
            self.assertNotEqual(result['status'],'output_level_readback_matches')
            self.assertTrue(result['write_attempted'])
        self.assertEqual(connection.__enter__.return_value.sendall.call_count,2)


if __name__ == '__main__':
    unittest.main()
