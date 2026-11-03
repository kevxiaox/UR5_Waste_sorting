# Four-object fixed-placement reference package — 2026-10-26

This package supersedes the earlier **two-object data package**. It does not replace the offline state-machine source assignment.

| Class | Destination | Explicit manual endpoints |
|---|---|---|
| sponge | RIGHT_CELL | 6 |
| plastic_bottle | LEFT_CELL | 6 |
| aluminum_can | LEFT_CELL | 6 |
| paper_cup | LEFT_CELL | 6 |

All 24 source snapshots recorded zero active TCP (tool flange origin). Joint positions are controller actual_q order in radians, preserved without angle wrapping. Each class has its own approach, grasp, lift, drop approach, release and retreat references. Do not share pickup/release heights just because classes share a destination.

## Portable use

Copy this whole folder to the Linux planning computer. Standard-library-only integrity check:

```text
python3 verify_inputs.py
```

`catalog.json` contains all four sets; each `<class>_requests.json` contains one object. `evidence/` stores original pose/session copies; `portable_sources.json` maps Windows provenance paths to package-relative evidence. Absolute Windows paths do not need to exist on the Linux computer. The verifier checks consistency, not physical readiness.

## Important retained incident

The can's first retreat capture was blocked by protective stop (safety mode 3). A later capture passed normal-mode checks, but the root cause was not documented. Its operator event remains in copied evidence and `incident_evidence`; `incident_review_required=true` is carried on every can planning candidate. Later normal status does not establish that the path or stop cause is resolved. Onsite cause review is required before future physical execution.

## Planner/adapter expectations

- These are **plan-only candidates**, not trajectories or an executable queue. Keep all execution-disabled flags.
- Configure UR5/CB3, not UR5e; confirm ROS/MoveIt/driver versions and joint-name prefix/order with the environment owner.
- Use calibrated robot kinematics and actual RG6/adapter/finger collision geometry, table/obstacles and carried-object attachments. Zero TCP does not imply no gripper geometry.
- Obtain a fresh actual start state. Reference predecessor joints are historical, not current measurements. Interpolation, velocities and timing are intentionally absent.
- Approach/lowering/release/retreat constraints must be planned deliberately. Unconstrained joint planning need not produce a straight approach.
- These are **fixed-placement** demonstrations. A matching vision label or confidence >=0.60 does not prove an object occupies the taught pickup pose. Placement registration is not verified; require an explicit placement decision before considering movement.
- Toothbrush has a vision class mapping but no manual endpoint plan. Never fall back to another class's plan.
- Tool digital output 0 OFF=open, ON=close is the onsite mapping; do not confuse tool output 0 with standard output 0 or guess a driver's service pin mapping.
- The temporary mean TCP is not used. Do not reinterpret these Cartesian recordings under another TCP.

State-machine work remains offline with its existing SIM_ONLY plans. Bridge through a separate reviewed adapter, maintaining task/attempt/command IDs and failure/cancel/timeout handling. Acknowledgement is not motion completion or physical grasp confirmation. Do not open the gripper or move home automatically on errors.

Next planning milestone: report environment versions/launch commands, load UR5 plus a static collision scene, and demonstrate plan-only goal handling. No real-robot connection or execution is authorized by this package.
