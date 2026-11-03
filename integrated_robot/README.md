# UR5 waste sorting

A UR5 arm sorts waste: an overhead ZED camera and a YOLO detector recognise the object in the pick
zone, the robot reaches it, grasps it with an OnRobot RG6 gripper and drops it in the bin of its
category (recyclable → `LEFT_CELL`, residual → `RIGHT_CELL`).

```
ZED image ─► YOLO (vision/) ─► vision gate ─► pick request ─► motion planning ─► UR5 + RG6
             class, box        1 object,       class, bin       every stage checked    via RTDE
                               stable, ≥ 0.60                    before the first move
```

Every motion is planned and collision-checked from the robot's actual joints before anything is
sent, with published algorithms (UR5 closed-form IK, RRT-Connect, conservative segment checks).
The robot runs in step mode until the cell has been validated, and never retries or moves on its
own after a failure.

**Status (2 November 2026).** The full cycle (reach → grasp → drop in the right bin → back to the
observe pose) runs on the real UR5 for every object class, driven by recorded detections. Next
milestone: the [automatic pipeline](#roadmap) — live camera, no operator step between objects.

RO11 project, École polytechnique, 2026.

---

## Contents

1. [Quick start](#quick-start)
2. [Testing on the real robot (recorded detections)](#testing-on-the-real-robot-recorded-detections)
3. [Sorting with the live camera](#sorting-with-the-live-camera)
4. [Hardware and network](#hardware-and-network)
5. [Safety](#safety)
6. [How a cycle runs](#how-a-cycle-runs)
7. [Commands](#commands)
8. [Teaching the cell](#teaching-the-cell)
9. [Validation status](#validation-status)
10. [Roadmap](#roadmap)
11. [Repository layout](#repository-layout) · [Algorithms](#algorithms)

---

## Quick start

Python 3.10 or newer. The planner, state machine and simulation need nothing but the standard
library.

**1. Install**

```bash
pip install -r requirements.txt            # robot connection (ur_rtde)
pip install -r requirements-vision.txt     # camera PC only: YOLO + OpenCV (+ ZED SDK, see below)
```

**2. Check everything works, without robot or camera**

```bash
python -m unittest discover -s tests
python run_sorting.py --replay fixtures/vision/sponge_stable.jsonl
```

The second command replays recorded detections of a sponge and runs a full sorting cycle on a
simulated robot. It should end with `RESULT: COMPLETED` and `sorted 1 object(s)`.

**3. Check the robot connection (read-only, nothing moves)**

```bash
python tools/check_robot.py --ip 147.250.35.40
```

Expected: `robot mode 7 (RUNNING)`, a position error below 3 mm (≈ 2 mm on this robot),
`collision model at current configuration: valid`. If the robot cannot be reached, see
[Hardware and network](#hardware-and-network).

---

## Testing on the real robot (recorded detections)

This is how the cell was validated on hardware. The camera is replaced by a **recorded detector
output** (`fixtures/vision/*.jsonl`): the program believes YOLO saw the object and runs the real
robot through the whole cycle. No camera, GPU or YOLO install is needed — any laptop with
`ur_rtde` works. In the current fixed-placement mode only the detected **class** matters, so a
recording is valid for any session as long as the object sits on its taught spot.

### Procedure

1. Pendant: robot on, brakes released, no program running, TCP = 0, **speed slider at 10–20 %**.
   One person keeps a hand on the emergency stop.
2. Arm **above the table** (freedrive if needed). The planner refuses to start from a posture
   that is already too close to the table — nothing is sent in that case.
3. Put **one** object on its taught spot (marked with tape on the table).
4. Run, from the repository folder:

```powershell
python run_sorting.py --replay fixtures/vision/plastic_bottle_stable.jsonl --robot-ip 147.250.35.40 --allow-unmeasured --confirm-before-picking
```

| Object | Recording | Spot | Bin |
|---|---|---|---|
| plastic bottle | `fixtures/vision/plastic_bottle_stable.jsonl` | bottle spot | `LEFT_CELL` (recyclable) |
| aluminium can | `fixtures/vision/aluminum_can_stable.jsonl` | **bottle spot** (handled exactly like the bottle) | `LEFT_CELL` |
| paper cup | `fixtures/vision/paper_cup_stable.jsonl` | cup spot | `LEFT_CELL` |
| sponge | `fixtures/vision/sponge_stable.jsonl` | sponge spot | `RIGHT_CELL` (residual) |
| two objects | `fixtures/vision/two_objects_ambiguous.jsonl` | — | must be **refused**, nothing moves |

### What happens

```
frame 5: ready plastic_bottle 0.77
TASK sort-…: plastic_bottle (0.77) -> LEFT_CELL, pick at [0.2, 0.108, 0.66]
Is the plastic_bottle on its TAUGHT pick spot, alone? Type "yes": yes
Approve this task? Type "yes": yes
```

1. The program connects, checks the controller TCP and the kinematic model (≈ 2 mm), then plans
   and checks **the whole cycle** from the actual joints before the first move.
2. It asks for two `yes`: the object is on its spot, and the task is approved.
3. With `--allow-unmeasured`, every stage waits for **Enter**: `TRANSFER_TO_PICK`,
   `OPEN_GRIPPER`, `DESCEND_TO_GRASP`, `CLOSE_GRIPPER`, `LIFT`, `TRANSFER_TO_DROP`,
   `DESCEND_TO_RELEASE`, `RELEASE`, `RETREAT`, `RETURN_TO_OBSERVE`. Look where the arm is going
   before each Enter. Each gripper stage takes ~5 s (the RG6 needs ~4.1 s to open or close).
4. Success ends with `RESULT: COMPLETED` and `sorted 1 object(s)`. **Ctrl+C** at any time is a
   controlled stop.

Every run is logged in `logs/robot_sorting.jsonl` (one JSON event per line: planned targets,
commands, controller feedback, reason of any refusal).

### Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `RESULT: FAILED`, journal says `Start configuration invalid … too close to the table` | the arm was left low (e.g. after teaching). Move it up in freedrive and run again. Nothing was sent. |
| the gripper receives the command but the fingers do not move | RG6 lock-up (probably its safety switch after a shock): pendant *I/O* tab, tool voltage 0 V then 24 V. |
| the arm goes to the right posture but misses the object | the object is not on its taught spot (fixed placement: a few cm are enough to miss). The journal shows the commanded joints, identical at every run. |
| the grasp posture itself is wrong (too high, too far) | re-teach that class with `tools/teach_grasp.py`, see [Teaching the cell](#teaching-the-cell). |
| `REFUSED, nothing sent: Duplicate task ID` | two runs started in the same second; run again. |

---

## Sorting with the live camera

```bash
python run_sorting.py --zed                                                                  # vision alone, simulated robot
python run_sorting.py --zed --robot-ip 147.250.35.40 --allow-unmeasured --confirm-before-picking --cycles 4
```

1. A window shows the camera image with the zones. Check they match the table, press **V**.
2. Put **one** object on its taught spot. After 5 stable frames the terminal shows the task
   (class, destination bin). Type `yes` twice, then follow the stages as above.
3. After each cycle the arm parks at the observe pose, out of the camera view: put the next object.

`--zed` needs the **ZED SDK**, which requires an **NVIDIA GPU** (CUDA): run it on the lab's camera
PC, not on a laptop with an AMD/Intel GPU. YOLO weights are in `vision/weights/best.pt`. Not run
live yet with the robot — first item of the [roadmap](#roadmap).

---

## Hardware and network

| Item | Value |
|---|---|
| Robot | Universal Robots UR5, CB3 controller, software 3.15.5 |
| Robot IP | `147.250.35.40`, mask `255.255.255.0`, gateway `147.250.35.245` (pendant: *Setup Robot → Network*) |
| Gripper | OnRobot RG6 on the wrist connector, 24 V, driven by **tool digital output 0** (ON = close, OFF = open); open/close ≈ 4.1 s |
| Camera | Stereolabs ZED above the table, HD720, left image + point cloud |
| Mounting | the robot base is on a rolling stand, about 0.55 m **below** the table top |

**Connecting the PC.** The PC must be on the robot's wired network (Ethernet port or USB-Ethernet
adapter); the school Wi-Fi does not reach the robot. Two ways:

* on the same wired network as the robot (wall socket / lab switch), adapter in automatic (DHCP)
  mode: the PC should get a `147.250.35.x` address;
* cable straight to the controller box (what we use), PC set to the fixed address
  `147.250.35.200`, mask `255.255.255.0`, no gateway (Windows: *Settings → Network → Ethernet →
  IP assignment → Manual*, or `New-NetIPAddress` in an administrator PowerShell). Only on a direct
  cable, and plug the robot's own cable back afterwards.

Test with `ping 147.250.35.40`, then `tools/check_robot.py` (disconnect any VPN). Do **not** switch
the robot to DHCP: its address would change. Powering the arm from the pendant: *Setup Robot →
Initialize Robot → ON → START*.

**Before running.** On the pendant: robot powered on, brakes released, no program running, no
protective stop. The active TCP must equal `tcp_pose` in `configs/real_cell/motion_config.json`
(0, as when the references were taught); the program refuses to start otherwise.

**ZED camera.** Install the ZED SDK, then its Python API (`get_python_api.py` in the SDK folder).
On Windows the SDK DLL folder is found automatically; set `ZED_DLL_DIRS` to override it.

---

## Safety

* Someone stays at the pendant with a hand on the **emergency stop** whenever the robot moves.
  Never test motions alone.
* Before the first motion the **whole cycle** is planned from the actual joints (inverse
  kinematics, collisions, straightness of approach moves). If any stage is impossible, nothing is
  sent.
* One command at a time; each stage starts only once the controller reports the robot at rest on
  its target, and is re-planned from the actual joints.
* After a failure, timeout or cancel the program **stops and does nothing on its own**: no
  re-opening of the gripper, no return home. The operator decides.
* Collisions are only checked against what is modelled (robot, gripper, held object, a virtual
  floor 0.35 m above the base, declared boxes). **The table top is not modelled** (team decision):
  table clearance relies on the taught postures, step mode and the E-stop. People and unmodelled
  objects are not avoided: the pendant safety configuration and the E-stop remain the real
  protection.
* The gripper's feedback signals are not used yet: "closed" means the command was executed and
  the settle time elapsed, not that the object is held (see [Roadmap](#roadmap)).

---

## How a cycle runs

| Phase | Stages | What happens |
|---|---|---|
| reaching | `TRANSFER_TO_PICK` | free motion (direct, or RRT-Connect around obstacles) to the pre-grasp pose, a safe distance from the object |
| picking | `OPEN_GRIPPER`, `DESCEND_TO_GRASP`, `CLOSE_GRIPPER`, `LIFT` | automatic grasp (`sorting_fsm/picking.py`); with `--confirm-before-picking` the robot waits for one Enter at the pre-grasp pose |
| transport | `TRANSFER_TO_DROP` | free motion with the object (included in the collision model) |
| placing | `DESCEND_TO_RELEASE`, `RELEASE`, `RETREAT` | release above the bin of the category |
| return | `RETURN_TO_OBSERVE` | park the arm at the taught observe pose, out of the camera view |

**Where the poses come from.** The vision team taught, by hand, 6 reference postures per object
class (approach, grasp, lift, drop approach, release, retreat), each object placed at a fixed
spot: `data/planning_inputs_four_objects_tcp_zero/`. The program verifies this package (file
hashes, joints, TCP) and replays those postures with planned, checked motions. The short
approach/lift/release/retreat moves are joint moves (`moveJ`) whose tool path must stay within
10 mm of a straight line. Joint moves are used rather than Cartesian lines because several
references are close to the elbow singularity, where a Cartesian line makes joint speeds explode.

**Taught grasps of our own.** A class's pick can be replaced by a grasp taught by hand with
`tools/teach_grasp.py` (e.g. the sponge taken with the gripper tilted ~22° below horizontal). Only
the grasp posture is taught; the approach (back along the tool axis) and the lift (vertical, or
back along the tool axis when a vertical lift is out of reach) are computed with the IK and
checked like the team's references. Enabled per class by `taught_route.grasp_overrides` in the
cell config; the team's package is never edited. Tilts up to ~40° keep the same arm posture;
beyond, the arm would have to flip, and a vertical (top-down) grasp is out of reach. All four
classes currently use grasps re-taught on site (`data/grasp_overrides/`).

**Route aliases.** `taught_route.route_aliases` makes a class use the whole taught route (pick and
drop) of another class with the same bin. Real cell: `{"aluminum_can": "plastic_bottle"}` — the
can is put on the bottle's spot and handled exactly like the bottle, so none of the can's own
references (and the protective stop recorded while teaching them) are used. No chains, same
target zone only.

**What the vision decides.** `sorting_fsm/vision_bridge.py` keeps a detection only if the detector
marks it OK, its confidence is at least the sorting policy's 0.60, its category matches the
policy, its box centre is inside the pick zone, it is the **only** object there and it stays
stable for 5 frames. The bin comes from `configs/sorting_policy.json`, never from the detector.

**Refused, nothing sent:** two objects in the pick zone; a class with no taught references
(toothbrush — never replaced by another class's plan unless explicitly aliased); a class whose
own teaching had an incident, until it is reviewed and added to `taught_route.incident_reviewed`.

---

## Commands

| Command | Purpose |
|---|---|
| `python run_sorting.py --replay FILE.jsonl` | sorting loop on recorded detections, simulated robot |
| `python run_sorting.py --replay FILE.jsonl --robot-ip IP --allow-unmeasured` | recorded detections, **real robot** (hardware validation) |
| `python run_sorting.py --image PHOTO.jpg` | YOLO on one image, simulated robot |
| `python run_sorting.py --zed` | live camera, simulated robot (check the vision alone) |
| `python run_sorting.py --zed --robot-ip IP ...` | live camera, real robot |
| `python tools/check_robot.py --ip IP [--watch]` | read-only: robot state, model vs controller, collision check |
| `python tools/teach_cell.py --ip IP` | read-only: record the observe pose and obstacles (arm moved in freedrive) |
| `python tools/teach_grasp.py --ip IP --class CLASS` | read-only: teach a grasp for a class; approach and lift are computed and checked |
| `python data/planning_inputs_four_objects_tcp_zero/verify_inputs.py` | integrity check of the taught references |

Options of `run_sorting.py`: `--cycles N` (objects to sort before stopping),
`--confirm-before-picking` (one Enter for the whole grasp), `--allow-unmeasured` (supervised runs
before validation; forces step mode), `--no-step` (no stage confirmation, only once the cell is
validated), `--frames-required N`, `--journal FILE`. Every run is logged as JSON lines in `logs/`.

---

## Teaching the cell

Both tools only **read** the robot; the arm is moved by hand (freedrive button on the pendant).
The position is read **when the key is typed**, and echoed as `recorded at joints …`.

**A grasp** (`tools/teach_grasp.py --ip 147.250.35.40 --class sponge`): gripper open (tool output
0 OFF), place the fingers around the object on its spot, type `g`, wait for
`all checks passed: type s to save`, then `s`. Only the **last** `g` is saved. Then move the arm
up before running the cycle.

**The observe pose** (`tools/teach_cell.py --ip 147.250.35.40`): arm folded out of the camera view,
`o`, then `s`. Do not leave an old instance of the tool open: it saves the config it loaded at
start and would overwrite newer changes. With TCP = 0 the recorded point is the **flange**, not the
fingertips, which is why `b` (obstacle boxes) must not be used on the table top without adding the
gripper length.

Files are backed up as `*.bak`; review the diff before running the robot.

---

## Validation status

| Item | Status | Where |
|---|---|---|
| UR5 kinematic model | checked on 24 real postures and on the robot (≈ 2 mm) | `tests/test_taught_route.py`, `tools/check_robot.py` |
| Controller TCP | 0 on the pendant, confirmed on site | `motion_config.json` `tcp_pose` |
| Gripper output | tool output 0, ON = close, confirmed on site | `configs/gripper_io.json` |
| Full cycle, every class | **passed on hardware** (step mode, recorded detections), 2 Nov 2026 | `logs/robot_sorting.jsonl` |
| Ctrl+C, two objects refused, chained grasp | **passed on hardware**, 2 Nov 2026 | — |
| Observe pose | taught on site | `cell_config.json` `observe` |
| Table | **not modelled** (team decision); virtual floor 0.35 m above the base | `collision.table_z_m` |
| RG6 collision geometry | assumed in line, 200 mm, not measured | `collision.tool_capsule_flange_m` |
| Live camera + robot | not run yet | [Roadmap](#roadmap) |
| Validation flags | `validated_on_hardware` and `collision_geometry_measured` still `false` → step mode forced | `motion_config.json` |

Progress log and to-do list (French): [`docs/AVANCEMENT_FR.md`](docs/AVANCEMENT_FR.md). On-site
checklist (French): [`docs/ROBOT_CHECKLIST_FR.md`](docs/ROBOT_CHECKLIST_FR.md).

---

## Roadmap

### 1. Automatic pipeline (next milestone)

Goal: one command, and the robot chains **camera → YOLO → bin choice → grasp → drop → observe
pose**, object after object, with no operator input between objects. The chain already exists in
`run_sorting.py --zed`; what remains:

**1.1 Run the live camera with the robot.**
*On the camera PC (NVIDIA)*: `git pull`, install both requirement files and the ZED SDK, connect
to the robot (fixed address as above), then `--zed` without and with `--robot-ip`. Check that the
zones match the table (**V**), that each class is recognised (the paper cup has never been seen
live), and that the arm at the observe pose does not hide the pick zone.
*On a laptop without NVIDIA GPU* (not implemented): read the ZED as a plain USB webcam (UVC, no SDK)
with `cv2.VideoCapture`, keep the left half of the side-by-side image at 1280×720, and add it as a
frame source next to `zed_frames()` in `run_sorting.py` (e.g. `--zed-webcam`). YOLO runs on CPU at
a few frames per second, enough for a 5-frame stability gate. The image is not rectified as the
SDK's is: check the zone overlay before pressing **V**. Depth is not needed in fixed placement.

**1.2 Check every grasp with the gripper signals.** Unattended runs need it: nobody watches the
fingers anymore. Measured on site: tool digital input 1 = 1 when the RG6 is ready (0 while
moving), tool digital input 0 = 1 when an object is gripped, analog input 0 ≈ 10 V open / ≈ 6.5 V
gripping. In `sorting_fsm/adapters/ur5_rtde_executor.py`, replace the fixed `settle_s` wait by
"wait for input 1, with a timeout", and after `CLOSE_GRIPPER` fail the cycle (gripper stays
closed, arm stays put, operator decides) if input 0 is not set. Map the inputs in
`configs/gripper_io.json`, add tests with the fake RTDE robot, then confirm on a few real grasps
(including an empty grasp, which must be refused before `LIFT`).

**1.3 Remove the manual confirmations.** Once 1.1 and 1.2 pass on hardware:
* the two `yes` per object (`run_sorting.py`, "on its TAUGHT pick spot"; `sorting_fsm/runner.py`,
  "Approve this task") become an option, e.g. `--auto-approve`, allowed only with a real robot
  when the validation flags are `true`;
* step mode is removed with `--no-step`, which requires `validated_on_hardware` and
  `collision_geometry_measured` set to `true` in `configs/real_cell/motion_config.json`. As the
  table is not modelled, this is a **team decision**: check the pendant safety planes first
  (*Installation → Safety → Planes*) and keep the speed slider low.

**1.4 Demo.**
`python run_sorting.py --zed --robot-ip 147.250.35.40 --no-step --cycles 4`, objects put one at a
time on their spot while the arm is at the observe pose.

### 2. Measured object positions (option B)

Implemented but disabled: the camera gives the object's position and the taught pick is shifted to
it (same orientation, heights and arm configuration), the drop part being unchanged. To enable it:
a **validated** camera → base calibration (marker on the gripper, 10–15 poses, OpenCV
`calibrateRobotWorldHandEye`, error < ~1 cm on an object at a measured position) saved as
`{"schema_version": 1, "T_base_from_camera": 4x4, "validated": true}`, the measured table and pick
zone, a TCP at the fingertips, then `taught_route.max_pick_offset_m > 0` (e.g. 0.05) and
`--placement measured --camera-to-base FILE.json`. The program refuses an unvalidated transform.
Top-down grasps are kinematically impossible in this cell (the table is too high above the base):
the object may move by a few centimetres, not anywhere.

### Out of scope

Toothbrush: no taught grasp or drop, refused.

---

## Repository layout

```
run_sorting.py              main program: camera/replay -> vision gate -> pick-and-place
sorting_fsm/                robot side (standard library only)
  ur5_kinematics.py           UR5 forward + closed-form inverse kinematics
  collision.py                capsule model of arm + gripper vs table, boxes, itself
  motion_planner.py           joint / linear planning, RRT-Connect, validation, timing
  pick_place_machine.py       non-blocking state machine: approval, stages, failure latching
  picking.py                  automatic picking phase
  taught_route.py             verified taught references -> stages (fixed or measured position), aliases
  grasp_override.py           grasps taught on site (teach_grasp.py) replacing a class's pick
  vision_bridge.py            detector JSON -> one stable candidate -> pick request
  runner.py                   operator loop shared by the scripts
  adapters/ur5_rtde_executor.py  real robot over ur_rtde, with interlocks
  mock_executor.py            simulated robot
vision/                     YOLO detector and ZED camera code (vision team)
  detector.py, weights/best.pt, waste_sorting/{camera,zones,localization}.py
configs/
  sorting_policy.json         class -> category -> bin, confidence threshold
  gripper_io.json             gripper output mapping
  workspace_zones.json        pick zone and bins in camera pixels
  real_cell/                  the lab cell (used by run_sorting.py and the tools)
  sim_cell/                   a fictional cell used by the unit tests
data/planning_inputs_four_objects_tcp_zero/   the 24 taught reference postures + evidence
data/grasp_overrides/       grasps re-taught on site, one file per class
fixtures/vision/            recorded detector outputs (one per class + ambiguous case)
tools/                      read-only robot tools (check, teach cell, teach grasp)
tests/                      101 unit tests (simulation, fake RTDE robot)
docs/                       architecture, on-site checklist (FR), progress log (FR), changelog
```

---

## Algorithms

| Problem | Method |
|---|---|
| Joint angles for a pose | closed-form UR5 IK, K. P. Hawkins (2013), as in ROS-Industrial `ur_kinematics` |
| Free motion | straight joint segment (`movej`), else RRT-Connect (Kuffner & LaValle, ICRA 2000, the OMPL/MoveIt default) + shortcutting (Geraerts & Overmars, 2007) |
| Cartesian moves | interpolated line with IK continuity / jump threshold, as MoveIt's `computeCartesianPath` |
| Collision checking | capsules; segments checked so that no point of the arm moves more than one check step between samples |
| Timing | trapezoidal profile, the one `movej(q, a, v)` uses; stop at each waypoint (no blending) |

Kinematic model checked on real data: on the 24 taught postures our forward kinematics matches the
controller within ~2 mm. Interfaces: [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).
