"""Offline evidence inventory. Never selects a target or connects to hardware."""
import hashlib
import json
from pathlib import Path

from .taught_poses import build_pose_record, LABEL_PURPOSES


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_catalog(folder):
    folder = Path(folder).resolve()
    annotations = {}
    for path in sorted(folder.glob('*.json')):
        data = json.loads(path.read_text(encoding='utf-8'))
        if data.get('type') == 'operator_posture_review':
            for entry in data['entries']:
                annotations.setdefault(str(Path(entry['source_path']).resolve()), []).append(entry)
    rows = []
    for path in sorted(folder.glob('*/pose.json')):
        record = json.loads(path.read_text(encoding='utf-8'))
        notes = list(annotations.get(str(path.resolve()), []))
        role_path = path.parent / 'operator_role.json'
        if role_path.exists():
            notes.append(json.loads(role_path.read_text(encoding='utf-8')))
        sha = digest(path)
        errors = []
        for note in notes:
            if (Path(note['source_path']).resolve() != path.resolve()
                    or note['source_sha256'] != sha):
                errors.append('annotation_source_mismatch')
        try:
            checked = build_pose_record(record['label'], *record['snapshots'])
            # Summary fields must still match the original snapshots.
            for key in ('actual_q_rad', 'actual_q_deg',
                        'configured_tcp_pose_base_m_rotvec_rad',
                        'configured_tcp_offset_flange_m_rotvec_rad'):
                if record[key] != checked[key]:
                    errors.append('snapshot_summary_mismatch:' + key)
        except (KeyError, TypeError, ValueError, RuntimeError) as error:
            errors.append('snapshot_check_failed:' + str(error))
        rejected = any(n.get('review_status') == 'rejected_table_contact_do_not_reuse'
                       for n in notes)
        rows.append({
            'record_id': path.parent.name, 'source_path': str(path),
            'source_sha256': sha, 'label': record.get('label'),
            # Older labels had mixed uses; do not retroactively reinterpret them.
            'label_purpose': (LABEL_PURPOSES.get(record.get('label'))
                              if record.get('label_scope') == 'operator_intent_only_not_physical_verification'
                              else None),
            'intended_roles': [n['intended_role'] for n in notes if 'intended_role' in n],
            'annotations': notes,
            'review_status': ('rejected_table_contact_do_not_reuse' if rejected else
                              'evidence_error' if errors else 'record_only_unvalidated'),
            'evidence_errors': errors,
            'actual_q_deg': record.get('actual_q_deg'),
            'configured_tcp_pose_base_m_rotvec_rad': record.get('configured_tcp_pose_base_m_rotvec_rad'),
            'automatic_replay_enabled': False,
        })
    groups = {}
    for row in rows:
        groups.setdefault(row['label'], []).append(row['record_id'])
    return {'type': 'offline_pose_evidence_catalog', 'records': rows,
            'duplicate_labels': {k: v for k, v in groups.items() if len(v) > 1},
            'target_selection_performed': False, 'automatic_replay_enabled': False,
            'sorting_execution_enabled': False,
            'note': 'Evidence inventory only; no posture or route is approved. No latest-record selection.'}


def render_markdown(catalog):
    roles = {
        'sponge_overhead_approach_open_jaws': '海绵上方接近（张爪）',
        'sponge_trial_right_drop_overhead_approach_empty_open_jaws': '右侧上方接近（空载张爪）',
        'sponge_right_cell_candidate_release_empty_open_jaws': '右侧候选释放（空载判断）',
    }
    lines = ['# 人工姿态证据清单', '',
             '仅供审阅，不选择运动目标，不连接机器人。所有记录均未获准自动回放。', '',
             '| 记录 | 原标签 | 用途/状态 | 证据检查 |', '| --- | --- | --- | --- |']
    for row in catalog['records']:
        role = ('；'.join(roles.get(x, x) for x in row['intended_roles'])
                or row.get('label_purpose') or '原标签记录，用途未细分')
        if row['review_status'] == 'rejected_table_contact_do_not_reuse':
            role = '曾压桌面，不可复用'
        elif any(n.get('review_status') == 'operator_reported_clearance_record_only'
                 for n in row['annotations']):
            role = '海绵两侧待夹持（用户报告有间隙）'
        check = '异常，需检查' if row['evidence_errors'] else '快照及已有说明哈希匹配'
        lines.append(f"| {row['record_id']} | {row['label']} | {role} | {check} |")
    lines += ['', '同名标签保留全部记录，不能按最新时间自动选择。原始关节角不做整圈归一化。', '',
              '证据检查通过不代表无碰撞、TCP准确或路径安全。候选释放姿态未被证明等于实际带载释放姿态。', '',
              '现场海绵试验包括一次夹角滑落，以及调整到中部后人工搬运至右侧并释放的反馈；实际试验路径和抓取/释放端点未完整记录。', '']
    return '\n'.join(lines)
