"""READ-ONLY teaching tool: teach a new grasp for one class (e.g. a tilted sponge grasp).

python tools/teach_grasp.py --ip 147.250.35.40 --class sponge

This script only READS the robot (RTDE receive interface); it never moves it or the gripper.
  1. On the pendant: TCP = 0, gripper open (I/O > Tool Digital Output 0 OFF).
  2. Hold the freedrive button and place the gripper around the object, tilted as wanted
     (stays in the same arm posture up to ~40 deg below horizontal). Release the button.
  3. Type  g  : the current posture is recorded as the GRASP. The approach (back along the tool
     axis) and the lift (vertical, or back along the tool axis when vertical is out of reach) are
     computed and checked: joint limits, collisions, straight tool path (same rules as the team's
     references). Adjust and type g again as often as needed: only the last one counts.
  4. Type  s  to save data/grasp_overrides/<class>.json (refused if a check failed) and,
     if you confirm, enable it in configs/real_cell/cell_config.json.  q  quits without saving.
The drop part of the class is unchanged. Remove the class from taught_route.grasp_overrides in the
cell config to go back to the team's taught grasp.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sorting_fsm.cell import load_cell, validate_cell  # noqa: E402
from sorting_fsm.grasp_override import derive_pick, load_grasp_override, override_record, tool_tilt_deg  # noqa: E402
from sorting_fsm.motion_planner import MotionPlanner, load_motion_config  # noqa: E402

REAL_CELL = ROOT / 'configs' / 'real_cell'


def fmt(values):
    return '[' + ', '.join(f'{v:+.4f}' for v in values) + ']'


def report(result, problems):
    if result is not None:
        print(f"  tool tilt      : {result['tool_tilt_deg']:.1f} deg below horizontal")
        for name in ('pick_approach', 'pick_grasp', 'pick_lift'):
            if name in result['joints']:
                pose = result['flange_poses'][name]
                print(f"  {name:<14} : joints {fmt(result['joints'][name])}  flange z {pose[2]:.3f} m")
        lift = 'vertical' if result['lift_mode'] == 'vertical' else 'back along the tool axis (vertical out of reach)'
        print(f'  lift           : {lift}')
        for move, value in result['deviation_m'].items():
            print(f'  {move + " move":<14} : tool path {value * 1000:.1f} mm from a straight line')
    if problems:
        print('  NOT SAVEABLE:')
        for p in problems:
            print(f'   - {p}')
    else:
        print('  all checks passed: type s to save')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--class', dest='class_name', required=True)
    parser.add_argument('--back', type=float, default=0.08, help='approach distance along the tool axis (m)')
    parser.add_argument('--lift', type=float, default=0.08, help='lift distance (m)')
    parser.add_argument('--cell', type=Path, default=REAL_CELL / 'cell_config.json')
    parser.add_argument('--motion', type=Path, default=REAL_CELL / 'motion_config.json')
    parser.add_argument('--out', type=Path, help='default data/grasp_overrides/<class>.json')
    args = parser.parse_args()
    cell, motion = load_cell(args.cell), load_motion_config(args.motion)
    if motion['tcp_pose'] != [0] * 6 and motion['tcp_pose'] != [0.0] * 6:
        raise SystemExit('motion_config tcp_pose must be 0 (grasps are recorded as flange poses)')
    grasp_cfg = cell['grasp']['classes'].get(args.class_name, cell['grasp']['default'])
    planner = MotionPlanner(motion)
    out = args.out or ROOT / 'data' / 'grasp_overrides' / f'{args.class_name}.json'
    import rtde_receive  # read-only: no control script, no outputs
    try:
        receive = rtde_receive.RTDEReceiveInterface(args.ip)
    except RuntimeError as error:
        raise SystemExit(f'cannot reach the robot at {args.ip} ({error}).\n'
                         'Check: Ethernet cable/adapter plugged, PC address 147.250.35.x (ipconfig), '
                         f'VPN off, robot powered on, `ping {args.ip}` answers.')
    print(__doc__)
    last = None
    try:
        while True:
            q, tcp = list(receive.getActualQ()), list(receive.getActualTCPPose())
            print(f'\njoints {fmt(q)}\nTCP    {fmt(tcp)}   tool tilt {tool_tilt_deg(tcp):.1f} deg below horizontal')
            choice = input('[g = record grasp / s = save / q = quit] > ').strip().lower()
            if choice == 'g':
                # read the arm AGAIN at the key press: it may have been moved while the prompt waited
                q, tcp = list(receive.getActualQ()), list(receive.getActualTCPPose())
                print(f'recorded at joints {fmt(q)}  tool tilt {tool_tilt_deg(tcp):.1f} deg below horizontal')
                result, problems = derive_pick(planner, q, tcp, back_m=args.back, lift_m=args.lift,
                                               max_deviation_m=cell['taught_route']['max_path_deviation_m'],
                                               held_object_m=grasp_cfg['held_object_length_m'])
                report(result, problems)
                last = (q, tcp, result) if not problems else None
            elif choice == 's':
                if last is None:
                    print('nothing valid recorded yet (type g with the arm on the grasp)')
                    continue
                q, tcp, result = last
                record = override_record(args.class_name, result, q, tcp, back_m=args.back, lift_m=args.lift,
                                         note=f'taught with tools/teach_grasp.py on {args.ip}')
                out.parent.mkdir(parents=True, exist_ok=True)
                if out.exists():
                    shutil.copy2(out, out.with_suffix('.json.bak'))
                out.write_text(json.dumps(record, indent=2) + '\n', encoding='utf-8')
                load_grasp_override(out, args.class_name)   # re-read exactly as run_sorting.py will
                print(f'saved {out}')
                relative = out.resolve().relative_to(ROOT).as_posix()
                current = cell['taught_route'].get('grasp_overrides', {})
                if current.get(args.class_name) == relative:
                    print('already enabled in the cell config')
                elif input(f'Enable it for {args.class_name} in {args.cell.name}? Type "yes": ').strip() == 'yes':
                    raw = json.loads(args.cell.read_text(encoding='utf-8'))
                    raw['taught_route'].setdefault('grasp_overrides', {})[args.class_name] = relative
                    validate_cell(raw)
                    shutil.copy2(args.cell, args.cell.with_suffix('.json.bak'))
                    args.cell.write_text(json.dumps(raw, indent=2) + '\n', encoding='utf-8')
                    print(f'enabled: taught_route.grasp_overrides.{args.class_name} = {relative}')
                else:
                    print(f'not enabled; add "grasp_overrides": {{"{args.class_name}": "{relative}"}} '
                          'to taught_route in the cell config to use it')
                break
            elif choice == 'q':
                print('quit without saving')
                break
    except (KeyboardInterrupt, EOFError):
        print('\nquit without saving')
    finally:
        receive.disconnect()


if __name__ == '__main__':
    main()
