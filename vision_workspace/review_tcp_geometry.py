"""Offline geometry hypotheses only. No hardware connection or TCP changes."""
import hashlib
import json
from pathlib import Path

import numpy as np

from waste_sorting.candidate_preview import load_candidate, candidate_provenance
from waste_sorting.observations import flange_from_state
from waste_sorting.point_calibration import analyze_observations
from waste_sorting.taught_poses import build_pose_record

ROOT = Path(__file__).resolve().parent


def compare_points(base_from_flange, base_from_camera, logo_offset, configured_offset, axial_m):
    """Transform reference points; axial candidate assumes unmeasured X=Y=0."""
    for transform in (base_from_flange, base_from_camera):
        if (np.shape(transform) != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], [0, 0, 0, 1])
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
                or not np.isclose(np.linalg.det(transform[:3, :3]), 1)):
            raise ValueError('Expected proper rigid transforms')
    if not np.isfinite(axial_m) or not 0 < axial_m < 1:
        raise ValueError('Invalid axial distance')
    offsets = {'logo': logo_offset, 'configured_tcp': configured_offset,
               'sponge_grip_hypothesis': [0, 0, axial_m]}
    points = {}
    camera_from_base = np.linalg.inv(base_from_camera)
    for name, offset in offsets.items():
        offset = np.asarray(offset, dtype=float)
        if offset.shape != (3,) or not np.isfinite(offset).all():
            raise ValueError('Expected finite three-component offset')
        base = base_from_flange @ np.r_[offset, 1]
        camera = camera_from_base @ base
        points[name] = {'flange_xyz_m': offset.tolist(),
                        'calculated_base_xyz_m': base[:3].tolist(),
                        'candidate_camera_xyz_m': camera[:3].tolist()}
    delta = (np.array(points['sponge_grip_hypothesis']['calculated_base_xyz_m'])
             - points['configured_tcp']['calculated_base_xyz_m'])
    return {'points': points, 'configured_to_hypothesis_base_delta_mm': (1000*delta).tolist(),
            'configured_to_hypothesis_distance_mm': float(1000*np.linalg.norm(delta)),
            'grasp_pose': None, 'trajectory': None, 'tcp_verified': False,
            'robot_execution_enabled': False, 'automatic_replay_enabled': False,
            'assumptions': ['sponge grip X=Y=0 is assumed, not measured',
                            '195mm is a coarse ruler reading for one jaw state',
                            'camera extrinsic and logo offset remain candidate fits',
                            'all points use one historical flange pose, not live state']}


def main():
    folder = ROOT/'data/tcp_checks'
    measurement_path = folder/'sponge_gripped_geometry_20261023_124503_486720.json'
    repeat_path = folder/'sponge_gripped_repeat_20261023_124731_606133.json'
    measurement = json.loads(measurement_path.read_text(encoding='utf-8'))
    repeat = json.loads(repeat_path.read_text(encoding='utf-8'))
    pose_path = Path(measurement['preclosure_pose_path'])
    if (hashlib.sha256(pose_path.read_bytes()).hexdigest() != measurement['preclosure_pose_sha256']
            or Path(repeat['prior_measurement_path']).resolve() != measurement_path.resolve()
            or hashlib.sha256(measurement_path.read_bytes()).hexdigest() != repeat['prior_measurement_sha256']
            or repeat['operator_reported_axial_distance_mm'] != measurement['operator_reported_axial_distance_mm']):
        raise ValueError('Measurement evidence differs from saved references')
    pose = json.loads(pose_path.read_text(encoding='utf-8'))
    checked_pose = build_pose_record(pose['label'], *pose['snapshots'])
    for key in ('actual_q_rad', 'actual_q_deg', 'configured_tcp_pose_base_m_rotvec_rad',
                'configured_tcp_offset_flange_m_rotvec_rad'):
        if pose[key] != checked_pose[key]:
            raise ValueError('Pose summary differs from snapshots: '+key)
    report_path = ROOT/'data/calibration_analysis/candidate_report.json'
    manifest_path = ROOT/'data/calibration_analysis/training_manifest.json'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
    workspace_path = ROOT/'config/workspace_zones.json'
    candidate = load_candidate(report_path, manifest_path,
                               hashlib.sha256(workspace_path.read_bytes()).hexdigest())
    reproduced = analyze_observations(report['training_files']+report['held_out_files'], manifest)
    logo = reproduced['candidate_feature_offset_flange_m']
    if not np.allclose(logo, report['candidate_feature_offset_flange_m'], rtol=0, atol=1e-10):
        raise ValueError('Logo offset differs from reproduced fit')
    result = compare_points(flange_from_state(pose['snapshots'][1]), candidate['transform'],
                            logo, pose['configured_tcp_offset_flange_m_rotvec_rad'][:3],
                            measurement['operator_reported_axial_distance_mm']/1000)
    result['type'] = 'offline_historical_tcp_geometry_comparison'
    result['candidate_provenance'] = candidate_provenance(candidate)
    result['sources'] = [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()}
                         for p in (pose_path, measurement_path, repeat_path, report_path,
                                   manifest_path, workspace_path)]
    output = folder/'historical_geometry_comparison.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# 已有logo标定与海绵TCP候选的离线几何对照', '',
             '复核冻结训练/留出证据，重现外参和logo偏移；不改写旧报告，不连接硬件。', '',
             '以下坐标均在历史124213姿态下计算，不代表机器人现在的位置，也不是执行目标。', '',
             '| 参考点 | 法兰坐标 XYZ（mm） |', '| --- | --- |']
    for name, point in result['points'].items():
        labels = {'logo': '固定机身logo（候选拟合）', 'configured_tcp': '当前配置工具点',
                  'sponge_grip_hypothesis': '海绵抓取假设点（X/Y假设为0）'}
        xyz = ', '.join(f'{v*1000:.3f}' for v in point['flange_xyz_m'])
        lines.append(f'| {labels[name]} | {xyz} |')
    lines += ['', '配置点与抓取假设点距离：'+f"{result['configured_to_hypothesis_distance_mm']:.3f} mm。",
              '在Base下的差向量：'+str(result['configured_to_hypothesis_base_delta_mm'])+' mm。', '',
              '不能把45mm直接加到Base Z或相机Z上：偏移应随法兰朝向旋转。', '',
              '下一验证应检验独立物理参考与抓取假设的对应；不能把用同一候选外参正反变换得到的一致性称为独立验证。',
              '195mm只适用于本次海绵夹持状态的粗量假设。TCP方向、X/Y偏移、开口依赖及独立物理精度未验证。',
              '所有执行及回放继续禁用。', '']
    output.with_suffix('.md').write_text('\n'.join(lines), encoding='utf-8')
    print('OFFLINE ONLY. Frozen candidate reproduced; controller configuration unchanged.')
    print('Distance mm:', result['configured_to_hypothesis_distance_mm'])
    print('Base delta mm:', result['configured_to_hypothesis_base_delta_mm'])
    print('Saved:', output.with_suffix('.md'))


if __name__ == '__main__':
    main()
