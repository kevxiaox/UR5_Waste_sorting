"""Nonblocking pick-and-place machine: current joints -> object -> drop zone, on any executor.

IDLE -> WAIT_APPROVAL -> TRANSFER_TO_PICK                                    (reaching)
     -> OPEN_GRIPPER -> DESCEND_TO_GRASP -> <grip steps> -> LIFT              (picking, picking.py)
     -> TRANSFER_TO_DROP -> DESCEND_TO_RELEASE -> RELEASE -> RETREAT          (transport, placing)
     -> [RETURN_TO_OBSERVE] -> COMPLETED
Grip steps default to CLOSE_GRIPPER; a picking variant can supply its own (pick_strategy).
Where the poses come from is a `route` (request -> Steps): GeometricRoute (default, poses from
cell_config + object position) or taught_route.TaughtRoute (the team's taught joint references).

Safety rules:
  * approve() first plans the WHOLE sequence from the robot's actual joints (IK, collisions,
    linear-path continuity). If any stage is infeasible, nothing is sent.
  * Each motion stage is replanned from the actual joints, which must match the expected ones.
  * One command at a time; the next only after a matching `completed` event.
  * step_mode: the operator confirms every stage before it starts (first robot sessions);
    confirm_phases=('picking',): one confirmation when a phase starts (robot waiting at the
    pre-grasp pose), then the whole phase runs automatically.
  * Failure / timeout / cancel latch a terminal state: no retry, no gripper action, no recovery
    or home move. The operator decides (acknowledge(), then a new explicit task).
"""
from dataclasses import asdict, dataclass, replace
import math
import re

from .cell import PickRequest, task_poses, validate_cell
from .contracts import Action, validate_joints, validate_timeout
from .journal import Journal, utc_now
from .motion_planner import MotionPlanner, MotionPlanningError
from .picking import picking_steps
from .steps import PHASES, STEP_KINDS, Step, default_pick_strategy  # noqa: F401  (re-exported)

TASK_ID_PATTERN = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]{0,63}')
TERMINAL_STATES = ('COMPLETED', 'FAILED', 'TIMED_OUT', 'CANCELLED')
RETRYABLE_OUTCOMES = ('FAILED', 'TIMED_OUT', 'CANCELLED', 'ABANDONED')
START_TOLERANCE_RAD = 0.02


def build_steps(cell, request, poses, pick_strategy=default_pick_strategy):
    steps = [Step('TRANSFER_TO_PICK', 'joint_pose', poses.pre_grasp, phase='reaching'),
             *picking_steps(poses.grasp, poses.pre_grasp, kind='linear',
                            grip_steps=pick_strategy(request, poses)),
             Step('TRANSFER_TO_DROP', 'joint_pose', poses.drop_approach,
                  held_object_m=poses.held_object_length_m, phase='transport'),
             Step('DESCEND_TO_RELEASE', 'linear', poses.release, phase='placing'),
             Step('RELEASE', 'gripper', action=Action.OPEN, phase='placing'),
             Step('RETREAT', 'linear', poses.drop_approach, phase='placing')]
    if cell['observe'].get('joints') is not None:
        steps.append(Step('RETURN_TO_OBSERVE', 'joint_q', list(cell['observe']['joints']), phase='return'))
    return steps


class GeometricRoute:
    """Poses from cell_config: object position from the request, top-down grasp."""
    def __init__(self, cell, pick_strategy=default_pick_strategy):
        self.cell, self.pick_strategy = cell, pick_strategy

    def steps(self, request):
        return build_steps(self.cell, request, task_poses(self.cell, request), self.pick_strategy)


@dataclass(frozen=True)
class PickPlaceResult:
    task_id: str
    attempt_id: str
    class_name: str
    target_zone: str
    object_base_xyz_m: list
    simulation_only: bool
    physical_grasp_verified: bool = False
    physical_release_verified: bool = False


class PickPlaceMachine:
    def __init__(self, executor, *, cell, clock, journal_path, planner=None, pick_strategy=None,
                 step_mode=False, command_timeout_s=10.0, min_speed_fraction=0.1,
                 on_complete=None, wall_clock=utc_now, require_simulation=True, route=None,
                 confirm_phases=()):
        """`min_speed_fraction`: lowest pendant speed slider expected (timeouts scale with it).
        `route`: object with steps(request) -> [Step]; default GeometricRoute(cell, pick_strategy)."""
        if require_simulation and getattr(executor, 'simulation_only', None) is not True:
            raise ValueError('require_simulation=True accepts simulation executors only')
        validate_cell(cell)
        validate_timeout(command_timeout_s)
        if type(min_speed_fraction) not in (int, float) or not 0 < min_speed_fraction <= 1:
            raise ValueError('min_speed_fraction must be in (0, 1]')
        if on_complete is not None and not callable(on_complete):
            raise ValueError('on_complete must be callable')
        confirm_phases = tuple(confirm_phases)
        if not all(p in PHASES for p in confirm_phases):
            raise ValueError(f'confirm_phases must be among {PHASES}')
        if route is not None and pick_strategy is not None:
            raise ValueError('Give the pick_strategy to the route, not to the machine')
        if route is not None and not callable(getattr(route, 'steps', None)):
            raise ValueError('route needs a steps(request) method')
        self.executor, self.cell, self.clock = executor, cell, clock
        self.planner = planner or MotionPlanner()
        self.pick_strategy = pick_strategy or default_pick_strategy
        self.route = route or GeometricRoute(cell, self.pick_strategy)
        self.step_mode, self.on_complete = step_mode, on_complete
        self.confirm_phases = confirm_phases
        self.command_timeout_s, self.min_speed_fraction = command_timeout_s, min_speed_fraction
        self.journal = Journal(journal_path, clock=clock, wall_clock=wall_clock)
        self.state, self.active, self.result = 'IDLE', None, None
        self._history = self.journal.task_history()
        for task_id, entry in self._history.items():
            if entry['last_outcome'] is None:
                entry['last_outcome'] = 'ABANDONED'
                self.journal.write(event='startup_unfinished_attempt_not_resumed', task_id=task_id,
                                   attempt_id=f"{task_id}#{entry['attempts']}", outcome='ABANDONED',
                                   reason='process restarted; robot state unknown; operator must re-submit',
                                   simulation_only=self._sim())
        self.journal.write(event='startup_idle', to_state='IDLE', simulation_only=self._sim(),
                           reason='fresh process starts idle; no gripper or motion command')

    # ------------------------------------------------------------------ operator API
    def submit(self, request, *, retry=False):
        """Register one PickRequest. Sends no command. Returns the attempt ID."""
        task_id = getattr(request, 'task_id', None)
        logged = task_id if isinstance(task_id, str) and TASK_ID_PATTERN.fullmatch(task_id) else None
        try:
            if not isinstance(request, PickRequest):
                raise ValueError('Expected a PickRequest')
            if logged is None:
                raise ValueError('Task ID must match [A-Za-z0-9][A-Za-z0-9._-]{0,63}')
            if self.state != 'IDLE':
                raise ValueError(f'Busy: single active task policy (state {self.state})')
            steps = list(self.route.steps(request))
            if not steps or not all(isinstance(s, Step) and s.kind in STEP_KINDS for s in steps):
                raise ValueError('route must return a non-empty list of Steps')
            known = self._history.get(task_id)
            if known is not None:
                if not retry:
                    raise ValueError('Duplicate task ID; pass retry=True for an explicit new attempt')
                if known['last_outcome'] not in RETRYABLE_OUTCOMES:
                    raise ValueError(f"Task already ended as {known['last_outcome']}; not retryable")
            elif retry:
                raise ValueError('Retry requested for an unknown task ID')
        except (ValueError, KeyError, TypeError) as error:
            self.journal.write(event='submit_rejected', task_id=logged, from_state=self.state,
                               to_state=self.state, reason=str(error), outcome='REJECTED',
                               simulation_only=self._sim())
            raise ValueError(str(error)) from None
        number = 1 if known is None else known['attempts'] + 1
        self._history[task_id] = {'attempts': number, 'last_outcome': None}
        self.active = {'request': request, 'task_id': task_id, 'attempt_id': f'{task_id}#{number}',
                       'steps': steps, 'expected_start': None, 'step': None, 'commands': [],
                       'command_id': None, 'deadline': None}
        self.result = None
        self._transition('WAIT_APPROVAL', 'task_accepted',
                         details={'class_name': request.class_name, 'target_zone': request.target_zone,
                                  'object_base_xyz_m': list(request.object_base_xyz_m),
                                  'steps': [s.name for s in steps], 'retry': retry})
        return self.active['attempt_id']

    def approve(self, task_id):
        """Operator approval: plan the whole sequence from the actual joints, then start."""
        if self.state != 'WAIT_APPROVAL' or self.active['task_id'] != task_id:
            self._reject_call('approve_rejected', task_id, 'No pending task with this ID')
        try:
            start = validate_joints(self.executor.current_joint_positions())
            expected = self._feasibility(start)
        except (MotionPlanningError, ValueError, TypeError) as error:
            self._finish('FAILED', 'plan_rejected', reason=f'{error}; no command sent')
            return self.state
        self.active['expected_start'] = expected
        self.journal.write(event='sequence_feasible', task_id=task_id, attempt_id=self.active['attempt_id'],
                           from_state=self.state, to_state=self.state, simulation_only=self._sim(),
                           reason='every stage planned from the actual joints (model-based check)',
                           details={'start_joints': start})
        self._enter_step(0)
        return self.state

    def confirm(self, task_id):
        """step_mode only: start the stage the machine is waiting on."""
        if self.state != 'WAIT_CONFIRM' or self.active['task_id'] != task_id:
            self._reject_call('confirm_rejected', task_id, 'No stage waiting for confirmation')
        self._start_step(self.active['step'])
        return self.state

    def pending_step(self):
        a = self.active
        return None if a is None or a['step'] is None else a['steps'][a['step']]

    def tick(self):
        """Poll the pending command once; never blocks."""
        a = self.active
        if a is None or a['command_id'] is None or self.state in TERMINAL_STATES + ('WAIT_CONFIRM',):
            return self.state
        if self.clock() >= a['deadline']:
            command_id = a['command_id']
            self._finish('TIMED_OUT', 'command_timeout', command_id=command_id,
                         reason='no completion before the deadline')
            self._request_executor_cancel(command_id)
            return self.state
        event = self.executor.poll(a['command_id'])
        if event.command_id != a['command_id'] or event.task_id != a['task_id']:
            self.journal.write(event='event_ignored', task_id=a['task_id'], attempt_id=a['attempt_id'],
                               from_state=self.state, to_state=self.state, command_id=event.command_id,
                               reason='command/task identity mismatch', simulation_only=self._sim())
            return self.state
        if event.status == 'running':
            return self.state
        if event.status != 'completed':
            self._finish('FAILED', 'command_failed', command_id=event.command_id,
                         reason=f'{event.reason or event.status}; no automatic recovery move or gripper action')
            return self.state
        self.journal.write(event='command_completed', task_id=a['task_id'], attempt_id=a['attempt_id'],
                           from_state=self.state, to_state=self.state, command_id=event.command_id,
                           reason=event.reason, simulation_only=event.simulation_only,
                           details={'physical_grasp_verified': event.physical_grasp_verified,
                                    'physical_release_verified': event.physical_release_verified})
        a['command_id'] = None
        if a['commands']:
            self._send_next()
        elif a['step'] + 1 < len(a['steps']):
            self._enter_step(a['step'] + 1)
        else:
            self._complete()
        return self.state

    def cancel(self, task_id):
        """Latch a logical cancel, then ask the executor for a controlled stop. Not an E-stop."""
        if self.active is None or self.active['task_id'] != task_id or self.state in TERMINAL_STATES:
            self._reject_call('cancel_rejected', task_id, 'No cancellable task with this ID')
        command_id = self.active['command_id']
        self._finish('CANCELLED', 'logical_cancel', command_id=command_id,
                     reason='operator cancel latched; no further sequence command will be issued')
        if command_id is not None:
            self._request_executor_cancel(command_id)
        return self.state

    def acknowledge(self, task_id):
        if self.state not in TERMINAL_STATES or self.active['task_id'] != task_id:
            self._reject_call('acknowledge_rejected', task_id, 'No terminal task with this ID')
        self._transition('IDLE', 'operator_acknowledged', reason='ready for a new task')
        self.active = None
        return self.state

    # ------------------------------------------------------------------ internals
    def _sim(self):
        return getattr(self.executor, 'simulation_only', None) is True

    def _plan_step(self, step, q):
        """(commands [(action, kwargs, expected_s)], end joints) for one stage from joints q."""
        if step.kind == 'gripper':
            return [(step.action, {}, self.cell['gripper']['settle_s'])], q
        if step.kind == 'linear':
            plan = self.planner.plan_linear(q, step.target)
            return [(Action.MOVE_POSE, {'pose': plan.target_pose}, plan.duration_s)], plan.end_joints
        if step.kind == 'joint_pose':
            plan = self.planner.plan(q, step.target, held_object_m=step.held_object_m,
                                     reference_joints=step.reference_joints)
        else:
            plan = self.planner.plan_to_joints(q, step.target, held_object_m=step.held_object_m)
            if step.max_path_deviation_m is not None:
                if len(plan.waypoints) > 2:
                    raise MotionPlanningError('needs a detour (RRT) where a near-straight move is required')
                deviation = self.planner.joint_path_deviation(plan.waypoints[0], plan.waypoints[-1])
                if deviation > step.max_path_deviation_m:
                    raise MotionPlanningError(f'joint move deviates {deviation * 1000:.1f} mm from a straight '
                                              f'line (limit {step.max_path_deviation_m * 1000:.1f} mm)')
        commands = [(Action.MOVE_JOINTS, {'joints': w}, d)
                    for w, d in zip(plan.waypoints[1:], plan.segment_durations_s)]
        return commands, plan.waypoints[-1]

    def _feasibility(self, start):
        expected, q = [], start
        for step in self.active['steps']:
            expected.append(q)
            try:
                _, q = self._plan_step(step, q)
            except MotionPlanningError as error:
                raise MotionPlanningError(f'stage {step.name} infeasible: {error}') from None
        return expected

    def _enter_step(self, index):
        self.active['step'] = index
        steps = self.active['steps']
        step = steps[index]
        phase_start = step.phase in self.confirm_phases and (index == 0 or steps[index - 1].phase != step.phase)
        if self.step_mode or phase_start:
            why = f'step mode: confirm stage {step.name}' if self.step_mode else \
                f'confirm the start of the {step.phase} phase ({step.name})'
            self._transition('WAIT_CONFIRM', 'waiting_operator_confirmation', reason=why,
                             details={'next_stage': step.name, 'phase': step.phase, 'target': step.target})
        else:
            self._start_step(index)

    def _start_step(self, index):
        a, step = self.active, self.active['steps'][index]
        try:
            actual = validate_joints(self.executor.current_joint_positions())
            gap = max(abs(x - y) for x, y in zip(actual, a['expected_start'][index]))
            if step.kind != 'gripper' and gap > START_TOLERANCE_RAD:
                raise MotionPlanningError(f'robot is {gap:.3f} rad away from the expected start of {step.name}')
            commands, _ = self._plan_step(step, actual)
        except (MotionPlanningError, ValueError, TypeError) as error:
            self._finish('FAILED', 'stage_plan_failed', reason=f'{step.name}: {error}; nothing sent')
            return
        self._transition(step.name, 'stage_started',
                         details={'commands': len(commands), 'phase': step.phase, 'target': step.target})
        a['commands'] = commands
        if commands:
            self._send_next()
        elif index + 1 < len(a['steps']):
            self._enter_step(index + 1)
        else:
            self._complete()

    def _send_next(self):
        a = self.active
        action, kwargs, expected_s = a['commands'].pop(0)
        try:
            command_id = self.executor.submit(a['task_id'], action, **kwargs)
        except (ValueError, RuntimeError, TypeError) as error:
            self._finish('FAILED', 'submit_failed', reason=f'{error}; no automatic recovery')
            return
        timeout = expected_s / self.min_speed_fraction + self.command_timeout_s
        a['command_id'], a['deadline'] = command_id, self.clock() + timeout
        self.journal.write(event='command_submitted', task_id=a['task_id'], attempt_id=a['attempt_id'],
                           from_state=self.state, to_state=self.state, command_id=command_id,
                           reason='submission is not completion', simulation_only=self._sim(),
                           details={'action': action.value, **kwargs, 'timeout_s': timeout})

    def _complete(self):
        a, r = self.active, self.active['request']
        self.result = PickPlaceResult(a['task_id'], a['attempt_id'], r.class_name, r.target_zone,
                                      list(r.object_base_xyz_m), self._sim())
        self._finish('COMPLETED', 'sequence_completed', details=asdict(self.result),
                     reason='all stages reported complete; grasp/release not physically sensed')
        if self.on_complete is not None:
            try:
                self.on_complete(self.result)
            except Exception as error:  # external code must not corrupt the machine state
                self.journal.write(event='on_complete_failed', task_id=a['task_id'],
                                   attempt_id=a['attempt_id'], from_state=self.state, to_state=self.state,
                                   reason=f'{type(error).__name__}: {error}', simulation_only=self._sim())

    def _request_executor_cancel(self, command_id):
        a = self.active
        try:
            ack = self.executor.cancel(command_id)
            status, reason = ack.status, ack.reason
        except Exception as error:
            status, reason = 'cancel_error', f'{type(error).__name__}: {error}'
        self.journal.write(event='executor_cancel_ack', task_id=a['task_id'], attempt_id=a['attempt_id'],
                           from_state=self.state, to_state=self.state, command_id=command_id,
                           reason=f'{reason} (executor acknowledgement, not proof the robot stopped)',
                           simulation_only=self._sim(), details={'executor_status': status})

    def _transition(self, to_state, event, *, command_id=None, reason='', outcome=None, details=None):
        a = self.active or {}
        self.journal.write(event=event, task_id=a.get('task_id'), attempt_id=a.get('attempt_id'),
                           from_state=self.state, to_state=to_state, command_id=command_id,
                           reason=reason, outcome=outcome, simulation_only=self._sim(),
                           details=_json_safe(details))
        self.state = to_state

    def _finish(self, state, event, *, command_id=None, reason='', details=None):
        self._history[self.active['task_id']]['last_outcome'] = state
        self.active['command_id'] = None
        self.active['commands'] = []
        self._transition(state, event, command_id=command_id, reason=reason, outcome=state, details=details)

    def _reject_call(self, event, task_id, reason):
        logged = task_id if isinstance(task_id, str) and TASK_ID_PATTERN.fullmatch(task_id) else None
        self.journal.write(event=event, task_id=logged, from_state=self.state, to_state=self.state,
                           reason=reason, outcome='REJECTED', simulation_only=self._sim())
        raise ValueError(reason)


def _json_safe(details):
    if details is None:
        return None
    return {k: (list(v) if isinstance(v, tuple) else v) for k, v in details.items()
            if not (isinstance(v, float) and not math.isfinite(v))}
