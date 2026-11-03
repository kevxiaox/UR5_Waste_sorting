"""READ-ONLY robot check: never moves the robot, never writes an output.

python tools/check_robot.py --ip 147.250.35.40            # one report
python tools/check_robot.py --ip 147.250.35.40 --watch    # refresh every second (Ctrl+C to stop)
python tools/check_robot.py --ip 147.250.35.40 --tcp 0 0 0.15 0 0 0   # try another TCP hypothesis

Reports robot/safety state, actual joints and TCP pose, and compares the controller's TCP pose
with our UR5 forward kinematics (validates DH + TCP + base frame), lists our IK solutions and
whether the current configuration is valid in our collision model (a "too close to the table"
here while the arm is clearly high means table_z_m is wrong).
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sorting_fsm.adapters.ur5_rtde_executor import ROBOT_MODE_RUNNING, pose_gap  # noqa: E402
from sorting_fsm.motion_planner import MotionPlanner, load_motion_config  # noqa: E402
from sorting_fsm.ur5_kinematics import forward, inverse  # noqa: E402


def fmt(values, digits=4):
    return '[' + ', '.join(f'{v:+.{digits}f}' for v in values) + ']'


def report(receive, planner):
    q, tcp = list(receive.getActualQ()), list(receive.getActualTCPPose())
    mode = receive.getRobotMode()
    print(f'robot mode {mode} ({"RUNNING" if mode == ROBOT_MODE_RUNNING else "NOT running"}), '
          f'safety mode {receive.getSafetyMode()}, protective stop {receive.isProtectiveStopped()}, '
          f'emergency stop {receive.isEmergencyStopped()}')
    print(f'actual joints (rad)     : {fmt(q)}')
    print(f'controller TCP pose     : {fmt(tcp)}')
    predicted = forward(q, planner.tcp_pose)
    position_error, orientation_error = pose_gap(predicted, tcp)
    verdict = 'OK' if position_error < 0.003 and orientation_error < 0.1 else 'MISMATCH'
    print(f'our FK with tcp {planner.tcp_pose}: {fmt(predicted)}')
    print(f'  -> position error {position_error * 1000:.2f} mm, orientation error {orientation_error:.4f}  [{verdict}]')
    solutions = inverse(tcp, planner.tcp_pose, q6_singular=q[5])
    gaps = [max(abs(math.atan2(math.sin(a - b), math.cos(a - b))) for a, b in zip(s, q)) for s in solutions]
    print(f'our IK: {len(solutions)} solutions; closest one differs from actual by '
          f'{min(gaps, default=float("nan")):.4f} rad (should be ~0)')
    problem = planner.state_problem(q)
    print(f'collision model at current configuration: {problem or "valid"}')
    try:
        bits = receive.getActualDigitalOutputBits()
        print(f'digital output bits     : {bits:#x} (tool outputs are bits 16-17)')
    except AttributeError:
        pass


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--watch', action='store_true')
    parser.add_argument('--tcp', nargs=6, type=float, metavar='V', help='override motion_config tcp_pose')
    parser.add_argument('--motion', type=Path, default=ROOT / 'configs' / 'real_cell' / 'motion_config.json')
    args = parser.parse_args()
    config = load_motion_config(args.motion)
    if args.tcp:
        config['tcp_pose'] = args.tcp
    planner = MotionPlanner(config)
    import rtde_receive  # read-only interface only: no control script is uploaded
    try:
        receive = rtde_receive.RTDEReceiveInterface(args.ip)
    except RuntimeError as error:
        raise SystemExit(f'cannot reach the robot at {args.ip} ({error}).\n'
                         'Check: Ethernet cable/adapter plugged, PC address 147.250.35.x (ipconfig), '
                         f'VPN off, robot powered on, `ping {args.ip}` answers.')
    try:
        while True:
            report(receive, planner)
            if not args.watch:
                break
            print('-' * 60)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        receive.disconnect()
    print(json.dumps({'note': 'read-only check finished; nothing was commanded'}))


if __name__ == '__main__':
    main()
