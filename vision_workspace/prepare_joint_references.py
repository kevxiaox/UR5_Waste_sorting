"""Export offline joint-goal references; no camera or robot connections."""
import argparse
import json
from pathlib import Path

from waste_sorting.endpoint_catalog import build_endpoint_catalog

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--index', type=Path, default=ROOT/'data/manual_sessions/20261026_two_route_endpoint_index.json')
    parser.add_argument('--policy', type=Path, default=ROOT/'config/sorting_policy.json')
    parser.add_argument('--output', type=Path, default=ROOT/'data/planning_inputs/20261026_tcp_zero')
    args = parser.parse_args()
    catalog = build_endpoint_catalog(args.index, args.policy)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output/'catalog.json').write_text(json.dumps(catalog, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# TCP=0：分物体关节参考与离线规划输入', '',
             '仅整理端点，不连接硬件，不生成或执行轨迹。完整来源哈希和原始关节弧度见catalog.json。', '',
             '| 物体 | 分类区 | 参考点数 |', '| --- | --- | --- |']
    for entry in catalog['objects']:
        lines.append(f"| {entry['object_class']} | {entry['target_zone']} | {len(entry['waypoints'])} |")
        path = args.output/(entry['object_class']+'_requests.json')
        path.write_text(json.dumps(entry, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
        print(entry['object_class'], entry['target_zone'], len(entry['waypoints']), 'joint references')
    lines += ['', '这是离线适配参考，不是MoveIt已规划结果；未设置速度、时间或插补方式。',
              '每段必须有实际起始状态、机器人/夹爪/障碍物模型及碰撞检查；不能仅凭类别匹配就前往固定抓取点。',
              '两物体拾取位置分别保存，未录制类别无兜底计划。零TCP是坐标约定，不代替夹爪碰撞几何。',
              '状态机交接包中的SIM_ONLY计划保持不变，后续由适配层对接这些具备来源的关节参考。', '']
    (args.output/'README.md').write_text('\n'.join(lines), encoding='utf-8')
    print('Saved:', args.output/'README.md')


if __name__ == '__main__':
    main()
