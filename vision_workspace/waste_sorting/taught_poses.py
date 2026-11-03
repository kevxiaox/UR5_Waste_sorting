"""Capture a manually chosen posture for review. No motion or I/O output writes."""
import math

from .observations import flange_from_state
from .rtde_readonly import read_robot_state,read_tool_io_state
from .zones import timestamp

LABEL_PURPOSES = {
    'HOME': '人工指定的 HOME 用途位置',
    'PICK_APPROACH': '抓取前的上方接近位置',
    'LEFT_DROP_APPROACH': '物理左侧分类区上方接近位置',
    'RIGHT_DROP_APPROACH': '物理右侧分类区上方接近位置',
    'PICK_GRASP': '两指对准物体两侧的待夹持位置',
    'PICK_LIFT': '夹持后短距离抬升、物体离桌的候选位置，不表示夹持或路径已验证',
    'LEFT_DROP_RELEASE': '物理左侧分类区的候选释放位置',
    'RIGHT_DROP_RELEASE': '物理右侧分类区的候选释放位置',
    'LEFT_DROP_RETREAT': '左侧释放后空载撤离位置，不表示路径已验证',
    'RIGHT_DROP_RETREAT': '右侧释放后空载撤离位置，不表示路径已验证',
    'TCP_CHECK_GRASP': 'TCP几何核对用待夹持位置，不限定物体所在区域，不表示TCP已验证',
}
LABELS = tuple(LABEL_PURPOSES)


def build_pose_record(label,before,pose,after):
    if label not in LABELS:
        raise ValueError('Unknown posture label')
    states = [before,pose,after]
    if len({(s['robot_ip'],s['controller_version']) for s in states}) != 1:
        raise ValueError('Robot identity changed during capture')
    for field in ('controller_timestamp_s','host_sample_monotonic_s'):
        times = [s[field] for s in states]
        if not all(math.isfinite(t) for t in times) or not times[0]<times[1]<times[2]:
            raise ValueError('Snapshot timing invalid')
    if after['host_sample_monotonic_s']-before['host_sample_monotonic_s']>10:
        raise ValueError('Snapshot bracket exceeded ten seconds')
    for endpoint in (before,after):
        velocity = endpoint['actual_qd_rad_s']
        if (endpoint['runtime_state'] != 1 or endpoint['robot_status_bits'] & 2 or
                endpoint['safety_mode'] not in (1,2)):
            raise ValueError('Robot program must be stopped and safety mode NORMAL/REDUCED')
        if len(velocity)!=6 or any(not math.isfinite(v) or abs(v)>0.01 for v in velocity):
            raise ValueError('Robot joints are moving')
    joints = [s['actual_q_rad'] for s in states]
    if any(len(q)!=6 or any(not math.isfinite(x) for x in q) for q in joints):
        raise ValueError('Invalid joint snapshot')
    spread = max(max(q[i] for q in joints)-min(q[i] for q in joints) for i in range(6))
    if math.degrees(spread)>0.1:
        raise ValueError('Robot posture changed during capture')
    return {'type':'manually_positioned_posture_for_review','timestamp_utc':timestamp(),'label':label,
            'label_purpose':LABEL_PURPOSES[label],
            'label_scope':'operator_intent_only_not_physical_verification',
            'actual_q_rad':pose['actual_q_rad'],
            'actual_q_deg':[math.degrees(x) for x in pose['actual_q_rad']],
            'configured_tcp_pose_base_m_rotvec_rad':pose['actual_tcp_pose_base_m_rotvec_rad'],
            'configured_tcp_offset_flange_m_rotvec_rad':pose['active_tcp_offset_flange_m_rotvec_rad'],
            'T_base_from_flange':flange_from_state(pose).tolist(),
            'bracket_joint_spread_deg':math.degrees(spread),'snapshots':states,
            'robot_motion_commands_enabled':False,'sorting_execution_enabled':False,
            'tcp_verified':False,'path_validated':False,'automatic_replay_enabled':False,
            'note':'Label is operator intent only; this record does not certify posture safety or a route. Endpoint checks do not observe continuous motion.'}


def capture_pose(label,host='147.250.35.40'):
    if label not in LABELS:
        raise ValueError('Unknown posture label')
    before = read_tool_io_state(host)
    pose = read_robot_state(host)
    after = read_tool_io_state(host)
    return build_pose_record(label,before,pose,after)
