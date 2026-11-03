"""One-action gripper test. Preview by default; --send explicitly changes Tool DO0."""
import argparse
from datetime import datetime
import json
from pathlib import Path

from waste_sorting.gripper_test import run_gripper_test
from waste_sorting.rtde_readonly import read_tool_io_state

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=('status','open','close'))
    parser.add_argument('--robot-ip',default='147.250.35.40')
    parser.add_argument('--send',action='store_true',help='Send ONE open/close I/O command; keep gripper empty and clear, robot program stopped, ownership User')
    args = parser.parse_args()
    if args.action == 'status' and args.send:
        parser.error('status is read-only; omit --send')
    output = ROOT/'data/gripper_io_tests'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')/'result.json'
    if args.action == 'status':
        result = read_tool_io_state(args.robot_ip)
        result['type'] = 'gripper_output_readonly_status'
        result['gripper_motion_verified'] = False
        output.parent.mkdir(parents=True,exist_ok=False)
        output.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    else:
        result = run_gripper_test(args.action,args.robot_ip,output,send=args.send)
    print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
    print('Saved:',output)
    if result.get('write_attempted'):
        print('No automatic output reset on exit. Verify the actual jaw position on site.')
    if result.get('status','').startswith(('preflight_failed','error_','output_readback_mismatch','unexpected_')):
        raise SystemExit(1)


if __name__ == '__main__':
    try:
        main()
    except (OSError,ValueError,RuntimeError) as error:
        raise SystemExit(str(error))
