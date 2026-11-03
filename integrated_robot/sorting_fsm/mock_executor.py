"""Deterministic mock actions. No sockets, SDK imports, movement or I/O writes."""
import math
import time
from .contracts import Action, CommandEvent, validate_joints, validate_pose
from .ur5_kinematics import forward, inverse

BEHAVIORS = ('success', 'failure', 'never_complete')
DEFAULT_START_POSE = (0.30, -0.30, 0.15, 3.14159, 0.0, 0.0)  # fictional, not a taught pose
DEFAULT_START_JOINTS = (0.5, -1.2, 1.5, -1.9, -1.57, 0.0)    # fictional, not a taught pose


class MockExecutor:
    simulation_only = True

    def __init__(self, *, clock=time.monotonic, delay_s=0.1, behavior='success',
                 script=None, start_pose=DEFAULT_START_POSE, start_joints=DEFAULT_START_JOINTS,
                 tcp_pose=None):
        """`script` optionally maps the 1-based command index to a behavior for that command.

        With `tcp_pose` the mock is kinematically consistent (UR5 model): joint moves update the
        TCP pose and linear moves update the joints. Without it, pose and joints are independent.
        """
        if type(delay_s) not in (float, int) or not math.isfinite(delay_s) or delay_s < 0:
            raise ValueError('Invalid mock delay')
        if behavior not in BEHAVIORS:
            raise ValueError('Unsupported mock behavior')
        script = dict(script or {})
        if any(type(k) is not int or k < 1 or v not in BEHAVIORS for k, v in script.items()):
            raise ValueError('Unsupported mock script')
        self.clock, self.delay_s, self.behavior, self.script = clock, delay_s, behavior, script
        self.commands = []
        self._records = {}
        self._active = None
        self._pose = validate_pose(start_pose)
        self._joints = validate_joints(start_joints)
        self.tcp_pose = None if tcp_pose is None else validate_pose(tcp_pose)
        if self.tcp_pose is not None:
            self._pose = forward(self._joints, self.tcp_pose)

    def submit(self, task_id, action, waypoint=None, *, pose=None, joints=None):
        action = Action(action)
        if not isinstance(task_id, str) or not task_id.strip():
            raise ValueError('Task ID required')
        if self._active is not None:
            raise RuntimeError('Only one active mock command at a time')
        if action is Action.MOVE:
            if not isinstance(waypoint, str) or not waypoint.startswith('SIM_ONLY_'):
                raise ValueError('Mock movement requires a SIM_ONLY waypoint')
        elif waypoint is not None:
            raise ValueError('Only waypoint moves take a waypoint')
        if action is Action.MOVE_POSE:
            pose = validate_pose(pose)
        elif pose is not None:
            raise ValueError('Only pose moves take a pose')
        if action is Action.MOVE_JOINTS:
            joints = validate_joints(joints)
        elif joints is not None:
            raise ValueError('Only joint moves take joints')
        index = len(self.commands) + 1
        command_id = f'mock-{index}'
        record = {'command_id': command_id, 'task_id': task_id, 'action': action.value,
                  'waypoint': waypoint, 'pose': pose, 'joints': joints, 'started_at': self.clock(), 'terminal': None,
                  'behavior': self.script.get(index, self.behavior)}
        self.commands.append(dict(record))
        self._records[command_id] = record
        self._active = command_id
        return command_id

    def poll(self, command_id):
        r = self._records[command_id]
        if r['terminal'] is not None:
            return r['terminal']
        if r['behavior'] == 'never_complete' or self.clock()-r['started_at'] < self.delay_s:
            return CommandEvent(command_id, r['task_id'], 'running')
        status = 'completed' if r['behavior'] == 'success' else 'failed'
        event = CommandEvent(command_id, r['task_id'], status, 'mock result only')
        if status == 'completed' and r['pose'] is not None:
            self._pose = list(r['pose'])
            if self.tcp_pose is not None:
                self._joints = self._nearest_ik(r['pose'])
        if status == 'completed' and r['joints'] is not None:
            self._joints = list(r['joints'])
            if self.tcp_pose is not None:
                self._pose = forward(self._joints, self.tcp_pose)
        r['terminal'] = event
        self._active = None
        return event

    def cancel(self, command_id):
        r = self._records[command_id]
        if r['terminal'] is not None:
            return r['terminal']
        event = CommandEvent(command_id, r['task_id'], 'cancelled', 'mock cancellation; no physical stop claim')
        r['terminal'] = event
        if self._active == command_id:
            self._active = None
        return event

    def current_tcp_pose(self):
        """Simulated TCP pose; a cancelled or failed move leaves the last completed pose."""
        return list(self._pose)

    def current_joint_positions(self):
        """Simulated joint angles; only completed joint moves change them (pose moves do not)."""
        return list(self._joints)

    def _nearest_ik(self, pose):
        """Joints a linear move ends in: the IK branch continuous with the current joints."""
        best = None
        for q in inverse(pose, self.tcp_pose, q6_singular=self._joints[5]):
            q = [v + 2 * math.pi * round((ref - v) / (2 * math.pi)) for v, ref in zip(q, self._joints)]
            if best is None or math.dist(q, self._joints) < math.dist(best, self._joints):
                best = q
        if best is None:
            raise RuntimeError('mock: linear target unreachable')
        return best
