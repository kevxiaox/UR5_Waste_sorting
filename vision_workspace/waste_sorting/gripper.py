"""Intent preview for the onsite-reported Tool Output 0 mapping. No transport."""
import json
from pathlib import Path

CONFIG_PATH = Path(__file__).resolve().parents[1]/'config/gripper_io.json'


class ToolOutputGripperPreview:
    def __init__(self,path=CONFIG_PATH):
        self.config = json.loads(Path(path).read_text(encoding='utf-8'))
        c = self.config
        if (c.get('schema_version') != 1 or c.get('control_mode') != 'tool_digital_output' or
                c.get('controlled_by') != 'User' or type(c.get('tool_output_channel')) is not int or
                c['tool_output_channel'] != 0 or c.get('open_level') is not False or
                c.get('close_level') is not True or c.get('robot_execution_enabled') is not False or
                c.get('software_execution_verified') is not False):
            raise ValueError('Expected reported output-0 mapping: OFF opens, ON closes; preview only')

    def preview(self,action):
        if action not in ('open','close'):
            raise ValueError('Gripper action must be open or close')
        return {'action':action,'control_mode':'tool_digital_output','tool_output_channel':0,
                'intended_output_level':self.config[action+'_level'],
                'mapping_source':self.config['source'],
                'execution_status':'disabled','robot_execution_enabled':False,
                'software_execution_verified':False,
                'note':'Intent only; no output written, no startup or shutdown output changes.'}

    def command(self,action):
        raise RuntimeError('Gripper execution disabled: this class has no robot transport')
