"""Shared operator loop of run_cell.py and run_sorting.py: executor choice and one task run."""
import time

from .mock_executor import MockExecutor
from .pick_place_machine import TERMINAL_STATES


def open_executor(robot_ip, *, motion, gripper, cell, sim_start_joints):
    """(executor, clock, advance). Real UR5 over ur_rtde when robot_ip is given, else the mock
    on a fake clock that `advance()` moves forward."""
    if robot_ip is not None:
        from .adapters.ur5_rtde_executor import UR5RTDEExecutor
        motion = dict(motion, simulation_only=False)
        executor = UR5RTDEExecutor(robot_ip, motion_config=motion, gripper_config=gripper,
                                   settle_s=cell['gripper']['settle_s'], clock=time.monotonic)
        print(f'connected; calibration check passed: {executor.calibration}')
        return executor, time.monotonic, lambda: time.sleep(0.02)
    now = [0.0]

    def advance():
        now[0] += 0.05
    executor = MockExecutor(clock=lambda: now[0], delay_s=0.2, start_joints=sim_start_joints,
                            tcp_pose=motion['tcp_pose'])
    return executor, (lambda: now[0]), advance


def run_task(machine, request, *, real, advance, ask=input):
    """Submit, approve (operator types yes on the real robot), run to a terminal state.
    Returns the final state; Ctrl+C = controlled cancel. Never closes the executor."""
    try:
        machine.submit(request)
    except ValueError as error:
        print(f'REFUSED, nothing sent: {error}')
        return 'REJECTED'
    try:
        approved = not real or ask('Approve this task? Type "yes": ').strip() == 'yes'
    except (KeyboardInterrupt, EOFError):
        approved = False
    if not approved:
        machine.cancel(request.task_id)
        print('not approved; nothing sent')
        return machine.state
    try:
        machine.approve(request.task_id)
        last = None
        while machine.state not in TERMINAL_STATES:
            if machine.state != last:
                print(f'  -> {machine.state}')
                last = machine.state
            if machine.state == 'WAIT_CONFIRM':
                step = machine.pending_step()
                what = f'the {step.phase} phase' if step.phase in machine.confirm_phases else step.name
                ask(f'  [Enter] to start {what} (Ctrl+C to cancel) ')
                machine.confirm(request.task_id)
                continue
            machine.tick()
            advance()
    except (KeyboardInterrupt, EOFError):  # Ctrl+C, or no operator input available
        if machine.state not in TERMINAL_STATES:
            machine.cancel(request.task_id)
        print('\nCANCELLED by operator (controlled stop requested; use the pendant E-stop if needed)')
    print(f'RESULT: {machine.state} (see journal for the reason)')
    return machine.state
