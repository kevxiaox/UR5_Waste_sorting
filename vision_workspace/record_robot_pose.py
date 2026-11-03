"""Read and save a posture selected manually on site. Never moves the robot."""
import argparse
from datetime import datetime
import json
from pathlib import Path

from waste_sorting.taught_poses import LABELS,LABEL_PURPOSES,capture_pose

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label',choices=LABELS,
                        help='用途标签；TCP_CHECK_GRASP 为TCP核对用待夹持位置，不限定区域；标签不代表动作或TCP已验证')
    parser.add_argument('--robot-ip',default='147.250.35.40')
    args = parser.parse_args()
    result = capture_pose(args.label,args.robot_ip)
    folder = ROOT/'data/taught_poses'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True,exist_ok=False)
    path = folder/'pose.json'
    path.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print('READ ONLY. No robot or gripper command sent.')
    print('Label:',args.label)
    print('Purpose:',LABEL_PURPOSES[args.label])
    print('Purpose is operator intent only, not grasp or release verification.')
    print('Actual joints deg:',result['actual_q_deg'])
    print('Configured TCP Base pose m/rad:',result['configured_tcp_pose_base_m_rotvec_rad'])
    print('TCP and paths remain unvalidated; automatic replay disabled.')
    print('Saved:',path)


if __name__ == '__main__':
    try:
        main()
    except (OSError,ValueError,RuntimeError,KeyError) as error:
        raise SystemExit(str(error))
