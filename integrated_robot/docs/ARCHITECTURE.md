# Architecture

## Data flow

```
vision/detector.py detect_waste(image) ──JSON──► VisionGate.observe(raw, frame_id)
                                                     │ ready: one stable Candidate
                                                     ▼
                         FixedPlacement / MeasuredPlacement .locate(candidate)
                                                     │ object position, UR base frame
                                                     ▼
                         make_vision_request(...) ──► PickRequest(task_id, class, bin, xyz)
                                                     │
                         TaughtRoute.steps(request) ─┤ list of Step (phase, kind, target)
                                                     ▼
       PickPlaceMachine: submit ─► approve (plans every stage) ─► tick/confirm ─► COMPLETED
                                                     │ one command at a time
                                                     ▼
                     Executor: MockExecutor (simulation) | UR5RTDEExecutor (ur_rtde)
```

## Modules

| Module | Role |
|---|---|
| `vision_bridge.py` | `parse_detector_output`, `VisionGate` (policy confidence, category check, pick zone in pixels, single candidate, N stable frames, operator camera confirmation), `FixedPlacement`, `MeasuredPlacement` + `CameraToBase` (refuses an unvalidated transform), `make_vision_request` |
| `taught_route.py` | `load_taught_catalog` re-verifies the taught package (hash of every evidence file and of the catalog, joints equal to the snapshots, zero TCP, routes equal to the sorting policy). `TaughtRoute.steps(request)` builds the stages; `grasp_position(class)` is the fixed pick spot |
| `picking.py` | `picking_steps(grasp, lift, kind=...)`: OPEN, approach, grip steps (default CLOSE), LIFT |
| `steps.py` | `Step` (name, kind, target, action, held_object_m, reference_joints, max_path_deviation_m, phase), `PHASES` |
| `pick_place_machine.py` | `PickPlaceMachine` (+ `GeometricRoute`, `build_steps` for a cell described by poses) |
| `motion_planner.py` | `plan` (to a TCP pose), `plan_to_joints`, `plan_linear`, `joint_path_deviation`, `state_problem` |
| `collision.py` | capsules of the UR5 links and tool, table plane, boxes, self-collision |
| `ur5_kinematics.py` | DH model, `forward`, `inverse` (8 solutions) |
| `adapters/ur5_rtde_executor.py` | real robot: `moveJ`/`moveL`/`setToolDigitalOut`, interlocks, completion at rest on target |
| `mock_executor.py` | simulated robot with the same contract |
| `journal.py` | strict JSON-lines journal, task history (duplicate / retry rules) |
| `runner.py` | `open_executor`, `run_task` (operator prompts, Ctrl+C = cancel) |

## Executor contract

```python
submit(task_id, action, *, pose=None, joints=None) -> command_id   # Action.MOVE_JOINTS / MOVE_POSE / OPEN / CLOSE
poll(command_id) -> CommandEvent(status='running' | 'completed' | 'failed', ...)
cancel(command_id) -> CommandEvent        # controlled stop, not an E-stop
current_joint_positions() -> [6 floats]   # rad
current_tcp_pose() -> [x, y, z, rx, ry, rz]
simulation_only: bool
```

Another robot library only needs a sibling class with these methods.

## State machine

```
IDLE ─submit─► WAIT_APPROVAL ─approve─► <stage> ... <stage> ─► COMPLETED
                                │            │
                                │            └─(step mode / confirm_phases)─► WAIT_CONFIRM ─confirm─► <stage>
                                └─ plan rejected / failure / timeout / cancel ─► FAILED | TIMED_OUT | CANCELLED
terminal ─acknowledge─► IDLE
```

* `approve` plans the whole sequence from the actual joints; any infeasible stage fails the task
  with no command sent.
* Each stage re-plans from the actual joints, which must be within 0.02 rad of the planned start.
* Timeouts scale with the slowest expected pendant speed slider (`min_speed_fraction`).
* Terminal states latch: no retry, gripper action or recovery motion. A new attempt of the same
  task needs `submit(request, retry=True)`; a process restart marks unfinished tasks `ABANDONED`.

## Extension points

* `pick_strategy(request, poses) -> [Step]`: a different grip (gripper / short linear steps; must
  close the gripper), passed to `TaughtRoute` or `GeometricRoute`.
* `route`: any object with `steps(request)`.
* `on_complete(PickPlaceResult)`: called after a completed task.
* Measured positions: `taught_route.max_pick_offset_m > 0` + `MeasuredPlacement`.

## Configuration

| File | Key settings |
|---|---|
| `configs/sorting_policy.json` | `min_confidence`, `class_to_category`, `category_to_zone` |
| `configs/gripper_io.json` | tool output channel and open / close levels |
| `configs/workspace_zones.json` | `PICK_ZONE` and bins as camera-pixel polygons |
| `configs/real_cell/cell_config.json` | `taught_route` (catalog, `max_pick_offset_m`, `max_path_deviation_m`, `incident_reviewed`), `observe`, `gripper.settle_s`, per-class held object length |
| `configs/real_cell/motion_config.json` | `tcp_pose`, joint limits, speeds, collision radii, `tool_capsule_flange_m`, `table_z_m`, `boxes`, planner parameters, validation flags |
