"""Isolated, explicit single-action Tool DO0 test; never used by sorting loop.

Uses a secondary program containing exactly one set_tool_digital_out call.
No automatic retries of writes, reset on exit, or presumed jaw feedback.
"""
import json
import math
from pathlib import Path
import socket
import time

from .gripper import ToolOutputGripperPreview
from .rtde_readonly import read_tool_io_state
from .zones import timestamp


def build_gripper_script(action):
    intent = ToolOutputGripperPreview().preview(action)
    value = 'True' if intent['intended_output_level'] else 'False'
    return f'sec isolated_gripper_io_test():\n  set_tool_digital_out(0, {value})\nend\n'.encode('ascii')


def check_preflight(state):
    if not state.get('controller_version','').startswith('3.15.'):
        raise ValueError('This isolated test is prepared for the current CB3 3.15 controller')
    if state['runtime_state'] != 1 or state['robot_status_bits'] & 2:
        raise ValueError('Stop the robot program on the pendant before the gripper-only test')
    if state['safety_mode'] not in (1,2):
        raise ValueError('Robot is not in NORMAL/REDUCED safety mode; resolve on site without this script')
    qd = state['actual_qd_rad_s']
    if len(qd) != 6 or any(not math.isfinite(v) or abs(v)>0.01 for v in qd):
        raise ValueError('Robot joints are moving; no gripper command sent')
    if state['tool_output_voltage_v'] != 24:
        raise ValueError('Expected existing 24 V tool supply; this script does not change voltage')


def run_gripper_test(action,host,report_path,*,send=False):
    """Default is entirely offline. Only send=True permits one fixed I/O write."""
    script = build_gripper_script(action)
    record = {'type':'isolated_gripper_io_test','timestamp_utc':timestamp(),'robot_ip':host,
              'intent':ToolOutputGripperPreview().preview(action),
              'script':script.decode('ascii'),'send_requested':send,'status':'preview_only',
              'write_attempted':False,'output_level_matches':None,'gripper_motion_verified':False,
              'robot_motion_commands_enabled':False,'sorting_execution_enabled':False}
    path = Path(report_path)
    path.parent.mkdir(parents=True,exist_ok=True)

    def save():
        path.write_text(json.dumps(record,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')

    # An audit file must be writable before any robot connection or output write.
    save()
    if not send:
        return record
    try:
        before = read_tool_io_state(host)
        record['before'] = before
        check_preflight(before)
        desired = record['intent']['intended_output_level']
        if before['tool_output_0'] is desired:
            record.update(status='already_at_requested_output_no_write',output_level_matches=True)
            save()
            return record
        record.update(status='write_attempt_may_change_output',write_attempted=True)
        save()
        with socket.create_connection((host,30002),timeout=3.0) as connection:
            connection.sendall(script)
        record['status'] = 'bytes_sent_output_not_yet_verified'
        save()
        for _ in range(3):
            time.sleep(0.15)
            after = read_tool_io_state(host)
            record['after'] = after
            record['output_level_matches'] = after['tool_output_0'] is desired
            record['tool_output_1_unchanged'] = after['tool_output_1'] is before['tool_output_1']
            record['joint_endpoint_max_change_deg'] = max(abs(a-b) for a,b in zip(
                after['actual_q_rad'],before['actual_q_rad']))*180/math.pi
            if record['output_level_matches']:
                break
        record['status'] = ('output_level_readback_matches' if record['output_level_matches'] else
                            'output_readback_mismatch_no_automatic_retry')
        if not record['tool_output_1_unchanged'] or record['joint_endpoint_max_change_deg']>0.1:
            record['status'] = 'unexpected_other_output_or_joint_change_review_on_site'
        save()
        return record
    except (OSError,ValueError,RuntimeError) as error:
        record.update(status='error_output_state_may_be_unknown' if record['write_attempted'] else
                      'preflight_failed_no_write',error=str(error))
        save()
        return record
