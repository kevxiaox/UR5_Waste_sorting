# Changelog

## Unreleased

* Docs: how to connect the PC to the robot (USB-Ethernet adapter, DHCP on the lab network or
  fixed address on a direct cable).
* Tools: `check_robot.py` and `teach_cell.py` print a clear checklist instead of a traceback when
  the robot cannot be reached.
* Real cell: gripper settle time 0.8 s -> 5.0 s. Measured on the RG6 (2026-11-02): a close or
  open takes about 4.1 s; with 0.8 s the arm could lift or descend while the fingers were moving.
* New `tools/teach_grasp.py` + `sorting_fsm/grasp_override.py`: teach a grasp by hand (e.g. tilted
  sponge grasp); approach and lift are derived and checked; enabled per class with
  `taught_route.grasp_overrides`. 9 new tests.
* Real cell (2026-11-02): grasps re-taught on site for every class; observe pose taught (cycle ends
  with `RETURN_TO_OBSERVE`); table box not kept (team decision, virtual floor kept).
* `taught_route.route_aliases`: a class can use the whole route of another class with the same
  bin; real cell `aluminum_can -> plastic_bottle`. 3 new tests.
* Fixtures for the paper cup and the can: every class can be tested on the real robot without the
  camera (`--replay ... --robot-ip ...`).
* Fix: `teach_cell.py` / `teach_grasp.py` read the arm when the key is typed, not when the prompt
  is shown (a pose moved while the prompt waited was ignored).
* Hardware validation (2026-11-02, step mode, recorded detections): full cycle for every class,
  Ctrl+C, two objects refused, chained grasp — all passed.
* README: hardware test procedure, troubleshooting, teaching, roadmap to the automatic pipeline.

## 0.1.0 — 2026-10-30

First version of the integrated system.

* Motion: UR5 forward / closed-form inverse kinematics, capsule collision model, joint and
  Cartesian planning with RRT-Connect and conservative validation, trapezoidal timing.
* `PickPlaceMachine`: whole-cycle feasibility check before any command, step mode, phase
  confirmation (`--confirm-before-picking`), latched failure / timeout / cancel, JSON-lines journal.
* Automatic picking phase (`picking.py`): open, approach, close, lift.
* Taught references: verified loading of the 24 reference postures (4 classes x 6) and replay
  with planned, checked motions; measured-position variant prepared (disabled).
* Vision bridge: YOLO detector output -> one stable candidate in the pick zone -> pick request;
  fixed placement (active) and camera-measured placement (requires a validated calibration).
* Real robot: `ur_rtde` executor with interlocks; read-only `check_robot.py` and `teach_cell.py`.
* Real-cell profile (TCP 0, radii validated against the 24 physically reached postures) and a
  simulation cell for tests; 89 unit tests, CI on Python 3.10 and 3.12.
