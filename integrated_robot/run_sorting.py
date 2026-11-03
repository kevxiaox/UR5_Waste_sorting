"""Sorting loop: YOLO detection -> category -> pick-and-place through the taught references.

Option A (default, --placement fixed): the object is put by hand on its class's taught pick spot;
vision decides WHICH class it is (and that exactly one object is in the pick zone), the robot
replays that class's taught postures, planned and collision-checked from its actual joints.
Option B (--placement measured --camera-to-base FILE, live ZED only): the object position comes
from the camera; refused until the transform file says validated: true and the cell allows
max_pick_offset_m > 0.

Vision sources:
  --replay FILE.jsonl   detector outputs, one JSON per line (offline, no YOLO needed)
  --image FILE          the team's YOLO on one image, as if every frame showed it
  --zed                 live ZED camera + YOLO (needs the ZED SDK and requirements-vision.txt)

Simulation (default, mock executor):
  python run_sorting.py --replay fixtures/vision/sponge_stable.jsonl
Real robot (step mode by default, Ctrl+C = controlled stop):
  python run_sorting.py --zed --robot-ip 147.250.35.40 --allow-unmeasured
"""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

from sorting_fsm.cell import load_cell
from sorting_fsm.motion_planner import MotionPlanner, load_motion_config
from sorting_fsm.pick_place_machine import PickPlaceMachine
from sorting_fsm.runner import open_executor, run_task
from sorting_fsm.taught_route import TaughtRoute
from sorting_fsm.vision_bridge import (CameraToBase, FixedPlacement, MeasuredPlacement, VisionGate,
                                       make_vision_request)

ROOT = Path(__file__).resolve().parent
REAL_CELL = ROOT / 'configs' / 'real_cell'
VISION_CODE = ROOT / 'vision'
WINDOW = 'Sorting - PICK_ZONE check: V confirm alignment, Q quit'


def replay_frames(path):
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        if line.strip():
            yield line, None, None


def yolo(vision_code):
    sys.path.insert(0, str(Path(vision_code).resolve()))
    from detector import detect_waste   # the team's adapter (cv2, ultralytics, best.pt)
    return detect_waste


def image_frames(path, vision_code):
    import cv2
    detect_waste = yolo(vision_code)
    frame = cv2.imread(str(path))
    if frame is None:
        raise SystemExit(f'Cannot read image {path}')
    raw, view = detect_waste(frame)
    while True:
        yield raw, None, view


def zed_frames(vision_code):
    detect_waste = yolo(vision_code)
    from waste_sorting.camera import ZedSource
    with ZedSource() as camera:
        while True:
            frame, xyz = camera.read_with_xyz()
            raw, view = detect_waste(frame)
            yield raw, xyz, view


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--replay', type=Path)
    source.add_argument('--image', type=Path)
    source.add_argument('--zed', action='store_true')
    parser.add_argument('--vision-code', type=Path, default=VISION_CODE, help='folder of the YOLO detector (vision/)')
    parser.add_argument('--placement', choices=('fixed', 'measured'), default='fixed')
    parser.add_argument('--camera-to-base', type=Path, help='measured placement: validated transform JSON')
    parser.add_argument('--cell', type=Path, default=REAL_CELL / 'cell_config.json')
    parser.add_argument('--motion', type=Path, default=REAL_CELL / 'motion_config.json')
    parser.add_argument('--frames-required', type=int, default=5, help='stable frames before a task')
    parser.add_argument('--cycles', type=int, default=1, help='objects to sort before stopping')
    parser.add_argument('--max-frames', type=int, default=2000)
    parser.add_argument('--robot-ip', default=None, help='real robot; omit for simulation')
    parser.add_argument('--confirm-before-picking', action='store_true',
                        help='wait at the pre-grasp pose; one Enter starts the automatic picking')
    parser.add_argument('--no-step', action='store_true')
    parser.add_argument('--allow-unmeasured', action='store_true')
    parser.add_argument('--journal', default=None)
    args = parser.parse_args()

    policy = json.loads((ROOT / 'configs' / 'sorting_policy.json').read_text(encoding='utf-8'))
    gripper = json.loads((ROOT / 'configs' / 'gripper_io.json').read_text(encoding='utf-8'))
    workspace = json.loads((ROOT / 'configs' / 'workspace_zones.json').read_text(encoding='utf-8'))
    cell, motion = load_cell(args.cell), load_motion_config(args.motion)
    route = TaughtRoute.from_cell(cell, policy, motion['tcp_pose'], root=ROOT)
    if args.placement == 'measured':
        if not args.zed or args.camera_to_base is None:
            parser.error('--placement measured needs --zed and --camera-to-base')
        if route.max_offset <= 0:
            parser.error('measured placement needs taught_route.max_pick_offset_m > 0 in the cell config')
        try:
            placement = MeasuredPlacement(CameraToBase.load(args.camera_to_base))
        except ValueError as error:
            raise SystemExit(f'REFUSED: {error}')
    else:
        placement = FixedPlacement(route)

    real = args.robot_ip is not None
    step_mode = not args.no_step if real else False
    if real:
        missing = [] if motion.get('validated_on_hardware') is True else ['motion.validated_on_hardware']
        if motion.get('collision_geometry_measured') is not True:
            missing.append('motion.collision_geometry_measured')
        if cell['observe'].get('measured') is not True:
            missing.append('cell.observe (arm stays at the drop retreat, may hide the camera view)')
        if route.max_offset > 0 and cell['pick_zone'].get('measured') is not True:
            missing.append('cell.pick_zone')
        if missing and not args.allow_unmeasured:
            raise SystemExit(f'Refusing real run, not measured/validated yet: {missing}. '
                             'Pass --allow-unmeasured for supervised tests.')
        if missing:
            step_mode = True
            print(f'WARNING placeholders in use: {missing}. Step mode forced.')
    first = route.taught[route.classes()[0]].joints['drop_retreat']
    executor, clock, advance = open_executor(args.robot_ip, motion=motion, gripper=gripper, cell=cell,
                                             sim_start_joints=cell['observe']['joints'] or first)
    journal = args.journal or str(ROOT / 'logs' / ('robot_sorting.jsonl' if real else 'sim_sorting.jsonl'))
    machine = PickPlaceMachine(executor, cell=cell, clock=clock, journal_path=journal,
                               planner=MotionPlanner(motion), step_mode=step_mode,
                               confirm_phases=('picking',) if args.confirm_before_picking else (),
                               require_simulation=not real, route=route)
    gate = VisionGate(policy, workspace, frames_required=args.frames_required)
    print(f'{"REAL ROBOT" if real else "SIMULATION"} | placement {placement.mode} | taught classes '
          f'{route.classes()} | journal {journal}')

    if args.replay:
        frames = replay_frames(args.replay)
    elif args.image:
        frames = image_frames(args.image, args.vision_code)
    else:
        frames = zed_frames(args.vision_code)
    if not args.zed:
        gate.confirm_camera_alignment()   # recorded frames: no live camera to align
        print('offline frames: camera alignment check not applicable')
    sorted_count, last_status, code = 0, None, 0
    try:
        for frame_id, (raw, xyz, view) in enumerate(frames, start=1):
            if frame_id > args.max_frames:
                print(f'stopping after {args.max_frames} frames')
                break
            if args.zed and not show_and_check(view, workspace, gate):
                break
            result = gate.observe(raw, frame_id)
            if result.status != last_status:
                detail = '' if result.candidate is None else f' {result.candidate.class_name} {result.candidate.confidence:.2f}'
                rejected = '; '.join(f'{c}: {r}' for _, c, r in result.rejected)
                print(f'frame {frame_id}: {result.status}{detail}{" | rejected " + rejected if rejected else ""}')
                last_status = result.status
            if result.status != 'ready':
                continue
            candidate = result.candidate
            task_id = datetime.now().strftime('sort-%Y%m%d-%H%M%S-') + str(frame_id)
            camera_xyz = sample_depth(xyz, candidate, workspace) if xyz is not None else None
            try:
                request = make_vision_request(task_id, candidate, placement, policy, camera_xyz)
            except ValueError as error:
                print(f'REFUSED, nothing sent: {error}')
                gate.reset()
                continue
            print(f'TASK {task_id}: {candidate.class_name} ({candidate.confidence:.2f}) -> {request.target_zone}, '
                  f'pick at {[round(v, 3) for v in request.object_base_xyz_m]}')
            if placement.needs_operator_confirmation and real:
                answer = input(f'Is the {candidate.class_name} on its TAUGHT pick spot, alone? Type "yes": ')
                if answer.strip() != 'yes':
                    print('placement not confirmed; nothing sent')
                    gate.reset()
                    continue
            state = run_task(machine, request, real=real, advance=advance)
            if state != 'COMPLETED':
                print('stopping the loop: the operator decides what to do (no automatic recovery)')
                code = 2
                break
            machine.acknowledge(task_id)
            sorted_count += 1
            gate.reset()
            if sorted_count >= args.cycles:
                break
    finally:
        if real:
            executor.close()
    print(f'sorted {sorted_count} object(s)')
    return code if sorted_count or code else 1


def sample_depth(xyz, candidate, workspace):
    """Camera XYZ at the box centre with the team's depth checks (same frame), or None."""
    from waste_sorting.localization import sample_surface
    sample = sample_surface(xyz, list(candidate.center_pixel), dict(workspace, camera_pose_revalidated=True))
    return sample['camera_xyz_m'] if sample['status'] == 'valid_camera_sample' else None


def show_and_check(view, workspace, gate):
    """Live view with the zones; V confirms the camera/table alignment, Q quits."""
    import cv2
    from waste_sorting.zones import overlay
    cv2.imshow(WINDOW, overlay(view, workspace))
    key = cv2.waitKey(1) & 0xFF
    if key == ord('v') and not gate.camera_confirmed:
        gate.confirm_camera_alignment()
        print('camera/table alignment confirmed for this session')
    return key not in (ord('q'), 27)


if __name__ == '__main__':
    raise SystemExit(main())
