"""Offline task-review replay of saved visuals. No camera, robot, or inference."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

from waste_sorting.task_review import build_task_review

ROOT = Path(__file__).resolve().parent
DEFAULT_CASES = ('20261023_131447_362569', '20261023_131844_792003',
                 '20261023_132223_349833', '20261023_132345_297757', '20261023_132625_639336')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--saved', nargs='+', type=Path, help='Explicit saved preview.json paths')
    args = parser.parse_args()
    paths = args.saved or [ROOT/'data/sorting_previews'/case/'preview.json' for case in DEFAULT_CASES]
    rows = []
    for path in paths:
        raw = path.read_bytes()
        data = json.loads(raw)
        task = build_task_review(data['result'], data['workspace_snapshot'])
        rows.append({'source_path': str(path.resolve()), 'source_sha256': hashlib.sha256(raw).hexdigest(),
                     'task_review': task})
        print(path.parent.name, task['status'])
    policy = ROOT/'config/sorting_policy.json'
    report = {'type': 'offline_sorting_task_review_replay', 'sources': rows,
              'policy_path': str(policy), 'policy_sha256': hashlib.sha256(policy.read_bytes()).hexdigest(),
              'robot_execution_enabled': False, 'trajectory': None,
              'note': 'Historical data replay only; current policy is reapplied without changing old records. '
                      'Saved flags/summaries are not independent verification of original evidence.'}
    folder = ROOT/'data/task_reviews'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder.mkdir(parents=True, exist_ok=False)
    (folder/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# 分拣任务只读预览', '', '历史样本按当前全局策略复核；不运行推理、不连接硬件，不选择执行目标。', '',
             '| 样本 | 类别 | 分类区 | 视觉复核 | 视觉阻塞项 |', '| --- | --- | --- | --- | --- |']
    for row in rows:
        for item in row['task_review']['items']:
            lines.append('| '+Path(row['source_path']).parent.name+' | '+str(item['class_name'])+' | '
                         +str(item['target_zone'])+' | '+('通过' if item['vision_review_passed'] else '未通过')
                         +' | '+(', '.join(item['vision_blockers']) or '无')+' |')
    lines += ['', '所有样本的物理执行均阻塞：外参独立验证、物理TCP、实际抓取参考点、路径及投放几何未完成；自动执行禁用。',
              '视觉通过只表示保存样本满足当前类别/门槛/区域/深度/连续帧摘要检查，不表示可抓取。',
              '此前海绵约59mm一致性差异保留为未解决问题；195mm不会被此工具应用。未知物体测试按用户要求跳过。', '']
    (folder/'report.md').write_text('\n'.join(lines), encoding='utf-8')
    print('Saved:', folder/'report.md')


if __name__ == '__main__':
    main()
