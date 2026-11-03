"""Reported polarity must not reverse or turn into hardware output."""
import json
import unittest
from unittest.mock import patch

from waste_sorting.gripper import CONFIG_PATH, ToolOutputGripperPreview


class GripperPreviewTests(unittest.TestCase):
    def test_latest_reported_polarity_channel_and_no_execution(self):
        gripper = ToolOutputGripperPreview()
        for action,level in (('open',False),('close',True)):
            result = gripper.preview(action)
            self.assertIs(result['intended_output_level'],level)
            self.assertEqual(result['tool_output_channel'],0)
            self.assertFalse(result['robot_execution_enabled'])
            self.assertFalse(result['software_execution_verified'])
            with self.assertRaises(RuntimeError):
                gripper.command(action)
        with self.assertRaises(ValueError):
            gripper.preview('toggle')

    def test_reversed_polarity_other_channel_or_enable_rejected(self):
        original = json.loads(CONFIG_PATH.read_text(encoding='utf-8'))
        for change in ({'open_level':True,'close_level':False},{'tool_output_channel':1},
                       {'open_level':0},{'robot_execution_enabled':True},{'controlled_by':'OnRobot'}):
            with patch('waste_sorting.gripper.json.loads',return_value=dict(original,**change)):
                with self.assertRaises(ValueError):
                    ToolOutputGripperPreview()


if __name__ == '__main__':
    unittest.main()
