"""Real UR5 executor over RTDE, using the ur_rtde library (SDU Robotics: `pip install ur_rtde`).

Implements the Executor contract (submit / poll / cancel / current_joint_positions /
current_tcp_pose) so the pick-and-place machine drives the real robot exactly like the mock.
If the equipment PC uses another library, write a sibling class with the same methods.

Interlocks, all checked before any motion is possible:
  * the robot is powered and running, not protective- or emergency-stopped;
  * the controller TCP equals motion_config tcp_pose, AND our UR5 forward kinematics applied to
    the actual joints reproduces the controller's actual TCP pose (validates DH, TCP and base
    frame in one test; refuses on mismatch);
  * every target is checked by the controller's own safety limits
    (isJointsWithinSafetyLimits / isPoseWithinSafetyLimits) before it is sent;
  * speeds come from motion_config (no per-command override); keep the pendant speed slider
    low for first runs.
Completion = robot at rest AND actual joints/pose within tolerance of the target. A protective
stop, an emergency stop or a dead control script reports `failed`. cancel() asks for a
controlled deceleration (stopJ/stopL): it is NOT an emergency stop; the pendant E-stop is.
The gripper is the tool digital output from configs/gripper_io.json; there is no grasp sensor,
so OPEN/CLOSE complete after `settle_s` and never claim a verified grasp.
"""
import math
import time

from ..contracts import Action, CommandEvent, validate_joints, validate_pose
from ..ur5_kinematics import forward, pose_to_matrix

ROBOT_MODE_RUNNING = 7  # UR robot mode "RUNNING" (RTDE robot_mode)


def _import_rtde():
    try:
        import rtde_control
        import rtde_io
        import rtde_receive
    except ImportError as error:
        raise RuntimeError('ur_rtde is not installed: pip install ur_rtde') from error
    return rtde_receive.RTDEReceiveInterface, rtde_control.RTDEControlInterface, rtde_io.RTDEIOInterface


def pose_gap(pose_a, pose_b):
    """(position error m, orientation error as max rotation-matrix entry difference)."""
    a, b = pose_to_matrix(pose_a), pose_to_matrix(pose_b)
    return (math.dist([a[i][3] for i in range(3)], [b[i][3] for i in range(3)]),
            max(abs(a[i][j] - b[i][j]) for i in range(3) for j in range(3)))


class UR5RTDEExecutor:
    simulation_only = False

    def __init__(self, robot_ip, *, motion_config, gripper_config, settle_s, clock=time.monotonic,
                 interfaces=None, joint_tolerance_rad=0.005, position_tolerance_m=0.002,
                 orientation_tolerance=0.01, calibration_tolerance_m=0.003,
                 at_rest_speed_rad_s=0.005):
        """`interfaces`: optional (receive, control, io) factories, for tests; default ur_rtde."""
        if gripper_config.get('control_mode') != 'tool_digital_output':
            raise ValueError('Only tool_digital_output grippers are supported')
        self.tcp_pose = validate_pose(motion_config['tcp_pose'])
        self.v, self.a = motion_config['max_joint_speed_rad_s'], motion_config['max_joint_acceleration_rad_s2']
        self.v_lin, self.a_lin = motion_config['max_linear_speed_m_s'], motion_config['max_linear_acceleration_m_s2']
        self.channel = gripper_config['tool_output_channel']
        self.levels = {Action.OPEN: gripper_config['open_level'], Action.CLOSE: gripper_config['close_level']}
        self.settle_s, self.clock = settle_s, clock
        self.joint_tol, self.pos_tol, self.rot_tol = joint_tolerance_rad, position_tolerance_m, orientation_tolerance
        self.calibration_tol, self.rest_speed = calibration_tolerance_m, at_rest_speed_rad_s
        receive_cls, control_cls, io_cls = interfaces or _import_rtde()

        # Read-only connection first: refuse before taking control if anything is off.
        self.receive = receive_cls(robot_ip)
        self._check_robot_state()
        self.calibration = self.check_calibration()
        self.control = control_cls(robot_ip)
        controller_tcp = list(self.control.getTCPOffset())
        if max(abs(x - y) for x, y in zip(controller_tcp, self.tcp_pose)) > 1e-4:
            self.control.stopScript()
            raise RuntimeError(f'Controller TCP {controller_tcp} differs from motion_config tcp_pose '
                               f'{self.tcp_pose}; fix the config or the pendant TCP, nothing moved')
        self.io = io_cls(robot_ip)
        self.commands = []
        self._records, self._active = {}, None

    # ------------------------------------------------------------------ checks
    def _check_robot_state(self):
        if self.receive.isEmergencyStopped():
            raise RuntimeError('Robot is emergency-stopped')
        if self.receive.isProtectiveStopped():
            raise RuntimeError('Robot is protective-stopped')
        mode = self.receive.getRobotMode()
        if mode != ROBOT_MODE_RUNNING:
            raise RuntimeError(f'Robot mode {mode} is not RUNNING (7): power on and release brakes')

    def check_calibration(self):
        """Our FK(actual joints) must match the controller's actual TCP pose."""
        q = list(self.receive.getActualQ())
        actual = list(self.receive.getActualTCPPose())
        predicted = forward(q, self.tcp_pose)
        position_error, orientation_error = pose_gap(predicted, actual)
        if position_error > self.calibration_tol or orientation_error > 10 * self.rot_tol:
            raise RuntimeError(
                f'Model/robot mismatch: FK predicts {predicted}, controller reports {actual} '
                f'(position error {position_error * 1000:.1f} mm). Check TCP and base frame; nothing moved')
        return {'position_error_m': position_error, 'orientation_error': orientation_error}

    def _robot_problem(self):
        if self.receive.isEmergencyStopped():
            return 'emergency stop'
        if self.receive.isProtectiveStopped():
            return 'protective stop'
        if not self.control.isProgramRunning():
            return 'RTDE control script not running (stopped from the pendant?)'
        return None

    # ------------------------------------------------------------------ Executor contract
    def submit(self, task_id, action, waypoint=None, *, pose=None, joints=None):
        action = Action(action)
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValueError('Task ID required')
        if self._active is not None:
            raise RuntimeError('Only one active command at a time')
        if waypoint is not None or action is Action.MOVE:
            raise ValueError('Symbolic waypoints are not executable on the real robot')
        problem = self._robot_problem()
        if problem is not None:
            raise RuntimeError(f'Robot not ready: {problem}')
        target = None
        if action is Action.MOVE_JOINTS:
            target = validate_joints(joints)
            if pose is not None or not self.control.isJointsWithinSafetyLimits(target):
                raise ValueError('Joint target rejected by the controller safety limits')
            if not self.control.moveJ(target, self.v, self.a, True):
                raise RuntimeError('moveJ refused by the controller')
        elif action is Action.MOVE_POSE:
            target = validate_pose(pose)
            if joints is not None or not self.control.isPoseWithinSafetyLimits(target):
                raise ValueError('Pose target rejected by the controller safety limits')
            if not self.control.moveL(target, self.v_lin, self.a_lin, True):
                raise RuntimeError('moveL refused by the controller')
        else:  # OPEN / CLOSE
            if pose is not None or joints is not None:
                raise ValueError('Gripper commands take no target')
            if not self.io.setToolDigitalOut(self.channel, self.levels[action]):
                raise RuntimeError('setToolDigitalOut refused')
        command_id = f'ur5-{len(self.commands) + 1}'
        record = {'command_id': command_id, 'task_id': task_id, 'action': action, 'target': target,
                  'started_at': self.clock(), 'terminal': None}
        self.commands.append({k: (v.value if isinstance(v, Action) else v) for k, v in record.items()})
        self._records[command_id] = record
        self._active = command_id
        return command_id

    def poll(self, command_id):
        r = self._records[command_id]
        if r['terminal'] is not None:
            return r['terminal']
        problem = self._robot_problem()
        if problem is not None:
            return self._terminate(r, 'failed', problem)
        if r['action'] in (Action.OPEN, Action.CLOSE):
            if self.clock() - r['started_at'] < self.settle_s:
                return CommandEvent(command_id, r['task_id'], 'running', simulation_only=False)
            return self._terminate(r, 'completed', 'tool output set and settle time elapsed; grasp not sensed')
        if max(abs(v) for v in self.receive.getActualQd()) > self.rest_speed:
            return CommandEvent(command_id, r['task_id'], 'running', simulation_only=False)
        if self._reached(r):
            return self._terminate(r, 'completed', 'robot at rest at the target (controller feedback)')
        return CommandEvent(command_id, r['task_id'], 'running', simulation_only=False)

    def _reached(self, r):
        if r['action'] is Action.MOVE_JOINTS:
            q = self.receive.getActualQ()
            return max(abs(a - b) for a, b in zip(q, r['target'])) <= self.joint_tol
        position_error, orientation_error = pose_gap(self.receive.getActualTCPPose(), r['target'])
        return position_error <= self.pos_tol and orientation_error <= self.rot_tol

    def _terminate(self, r, status, reason):
        event = CommandEvent(r['command_id'], r['task_id'], status, reason, simulation_only=False)
        r['terminal'] = event
        if self._active == r['command_id']:
            self._active = None
        return event

    def cancel(self, command_id):
        r = self._records[command_id]
        if r['terminal'] is not None:
            return r['terminal']
        if r['action'] is Action.MOVE_JOINTS:
            self.control.stopJ(2.0)
        elif r['action'] is Action.MOVE_POSE:
            self.control.stopL(1.0)
        return self._terminate(r, 'cancelled', 'controlled stop requested (stopJ/stopL); not an emergency stop')

    def current_joint_positions(self):
        return list(self.receive.getActualQ())

    def current_tcp_pose(self):
        return list(self.receive.getActualTCPPose())

    def close(self):
        """Stop the RTDE control script and disconnect. Does not touch the gripper output."""
        for interface in (getattr(self, 'control', None),):
            if interface is not None:
                interface.stopScript()
                interface.disconnect()
        if getattr(self, 'receive', None) is not None:
            self.receive.disconnect()
