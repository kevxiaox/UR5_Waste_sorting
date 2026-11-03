"""READ-ONLY teaching tool: record the cell by placing the arm by hand (freedrive on the pendant).

python tools/teach_cell.py --ip 147.250.35.40          # edits configs/real_cell/*.json

This script only READS the robot (RTDE receive interface). Move the arm with the pendant
(freedrive button or jog), then choose what the current position represents:
  o  observe position (arm out of the camera view; start/end of every cycle)
  b  obstacle box: touch two opposite corners (table, bin wall, camera mast...) -> motion boxes
  p  pick-zone corner (measured positions only): touch the table at each corner
  d  drop pose for a zone (geometric route only)
  t  table height probe (horizontal-plane model only)
The recorded point is the controller TCP: with TCP = 0 (real cell) it is the FLANGE centre, not
the fingertips; touch with the flange or add the gripper length by hand.
  x  remove every SIM_ONLY_ placeholder box from motion_config
  s  save and quit   |   q  quit without saving
Files are backed up (*.bak) before being overwritten. Review them before running the robot.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sorting_fsm.cell import validate_cell  # noqa: E402
from sorting_fsm.geometry import tool_z_axis  # noqa: E402
from sorting_fsm.motion_planner import validate_motion_config  # noqa: E402

REAL_CELL = ROOT / 'configs' / 'real_cell'


def fmt(values):
    return '[' + ', '.join(f'{v:+.4f}' for v in values) + ']'


def stamp():
    return datetime.now(timezone.utc).isoformat()


def save(path, data, validate):
    validate(data)
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + '.bak'))
    path.write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    print(f'saved {path}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ip', required=True)
    parser.add_argument('--cell', type=Path, default=REAL_CELL / 'cell_config.json')
    parser.add_argument('--motion', type=Path, default=REAL_CELL / 'motion_config.json')
    args = parser.parse_args()
    cell = json.loads(args.cell.read_text(encoding='utf-8'))
    motion = json.loads(args.motion.read_text(encoding='utf-8'))
    import rtde_receive  # read-only: no control script, no outputs
    try:
        receive = rtde_receive.RTDEReceiveInterface(args.ip)
    except RuntimeError as error:
        raise SystemExit(f'cannot reach the robot at {args.ip} ({error}).\n'
                         'Check: Ethernet cable/adapter plugged, PC address 147.250.35.x (ipconfig), '
                         f'VPN off, robot powered on, `ping {args.ip}` answers.')
    corners, table_probes, box_corners = [], [], []
    print(__doc__)
    try:
        while True:
            q, tcp = list(receive.getActualQ()), list(receive.getActualTCPPose())
            print(f'\njoints {fmt(q)}\nTCP    {fmt(tcp)}')
            choice = input('[o/p/d/t/b/x/s/q] > ').strip().lower()
            # read the arm AGAIN at the key press: it may have been moved while the prompt waited
            q, tcp = list(receive.getActualQ()), list(receive.getActualTCPPose())
            if choice in ('o', 'p', 'd', 't', 'b'):
                print(f'recorded at joints {fmt(q)}\n            TCP    {fmt(tcp)}')
            if choice == 'o':
                cell['observe'] = {'joints': q, 'measured': True, 'taught_at_utc': stamp(),
                                   'comment': 'arm out of the camera view'}
                print('observe position recorded')
            elif choice == 'p':
                corners.append(tcp[:3])
                print(f'pick-zone corner {len(corners)} recorded (record them in order around the zone)')
                if len(corners) >= 3:
                    zs = [c[2] for c in corners]
                    cell['pick_zone'] = {'polygon_xy_m': [c[:2] for c in corners],
                                         'surface_z_m': sum(zs) / len(zs), 'measured': True,
                                         'taught_at_utc': stamp()}
                    spread = max(zs) - min(zs)
                    print(f'pick zone: {len(corners)} corners, surface z {sum(zs) / len(zs):.4f} m, '
                          f'z spread {spread * 1000:.1f} mm' + ('  <-- > 5 mm: table not flat or TCP wrong?' if spread > 0.005 else ''))
            elif choice == 'd':
                name = input('zone name (e.g. LEFT_CELL / RIGHT_CELL): ').strip()
                if not name:
                    continue
                down = tool_z_axis(tcp[3:])
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, -down[2]))))
                if tilt > 5:
                    print(f'WARNING: tool is {tilt:.1f} deg from vertical; descents are vertical lines')
                old = cell['drop_zones'].get(name, {})
                cell['drop_zones'][name] = {'release_pose': tcp,
                                            'approach_height_m': old.get('approach_height_m', 0.10),
                                            'measured': True, 'taught_at_utc': stamp()}
                print(f'drop pose for {name} recorded')
            elif choice == 't':
                table_probes.append(tcp[2])
                z = sum(table_probes) / len(table_probes)
                print(f'table probes: {len(table_probes)}, mean z {z:.4f} m')
                if input('use as collision.table_z_m? [y/N] ').strip().lower() == 'y':
                    motion['collision']['table_z_m'] = round(z, 4)
            elif choice == 'b':
                box_corners.append(tcp[:3])
                if len(box_corners) == 2:
                    lo = [min(a, b) for a, b in zip(*box_corners)]
                    hi = [max(a, b) for a, b in zip(*box_corners)]
                    name = input('obstacle name: ').strip() or f'box_{len(motion["collision"]["boxes"]) + 1}'
                    motion['collision']['boxes'].append({'name': name, 'min': lo, 'max': hi,
                                                         'taught_at_utc': stamp()})
                    print(f'box {name} min {fmt(lo)} max {fmt(hi)} added (add margin by editing if needed)')
                    box_corners = []
                else:
                    print('first corner recorded; touch the opposite corner and press b again')
            elif choice == 'x':
                before = len(motion['collision']['boxes'])
                motion['collision']['boxes'] = [b for b in motion['collision']['boxes']
                                                if not b['name'].startswith('SIM_ONLY_')]
                print(f"removed {before - len(motion['collision']['boxes'])} placeholder box(es)")
            elif choice == 's':
                save(args.cell, cell, validate_cell)
                save(args.motion, motion, validate_motion_config)
                break
            elif choice == 'q':
                print('quit without saving')
                break
    finally:
        receive.disconnect()


if __name__ == '__main__':
    main()
