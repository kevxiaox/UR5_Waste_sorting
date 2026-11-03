"""Copy a minimal offline assignment snapshot and record source hashes."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PACK = ROOT/'handoff_fsm_fr'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_new_or_same(path, raw):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_bytes() != raw:
        raise RuntimeError(f'Refusing to overwrite changed handoff file: {path}')
    path.write_bytes(raw)


def main():
    copied = []
    pairs = [(f'config/{name}.json', f'config/{name}.json')
             for name in ('sorting_policy', 'workspace_zones', 'gripper_io')]
    pairs += [(f'waste_sorting/{name}.py', f'reference/waste_sorting/{name}.py')
              for name in ('__init__', 'interfaces', 'decision', 'gripper', 'zones', 'task_review')]
    pairs += [(name, f'reference/{name}')
              for name in ('HANDOFF_2026-10-23.md', 'TASK_PREVIEW_GUIDE.md')]
    pairs += [('tests/test_task_review.py', 'reference/tests/test_task_review.py')]
    cases = {'sponge': '20261023_162848_516853', 'bottle': '20261023_164647_730880'}
    for case, capture in cases.items():
        for name in ('preview.json', 'rgb.png'):
            pairs.append((f'data/sorting_previews/{capture}/{name}', f'fixtures/evidence/{case}/{name}'))
    pairs.append(('data/task_reviews/two_route_acceptance_20261023_60_percent.json',
                  'fixtures/evidence/two_route_acceptance.json'))
    for original, destination in pairs:
        source, target = ROOT/original, PACK/destination
        write_new_or_same(target, source.read_bytes())
        copied.append({'source_relative_to_project': original, 'destination': destination,
                       'sha256': digest(source), 'bytes': source.stat().st_size})

    for case, capture in cases.items():
        original = ROOT/f'data/sorting_previews/{capture}/preview.json'
        data = json.loads(original.read_text(encoding='utf-8'))
        result = data['result']
        item = result['items'][0]
        if result['task_review']['vision_pass_count'] != 1:
            raise ValueError('Expected one visually accepted saved observation')
        route = item['target_zone']
        task_id = f'sim-{case}-001'
        proposal = {
            'schema_version': 1, 'task_id': task_id,
            'class_name': item['detector_item']['class_name'],
            'confidence': item['detector_item']['confidence'],
            'target_zone': route, 'vision_review_passed': True,
            'stable_window_frames': result['temporal_observation']['valid_processed_frames'],
            'physical_ready': False, 'simulation_only': True, 'robot_execution_enabled': False,
            'source_fixture': f'fixtures/evidence/{case}/preview.json',
            'source_sha256': digest(original),
            'note': 'Derived visual proposal for simulation. Not a grasp pose, execution approval or real task queue.'}
        plan = {
            'schema_version': 1, 'task_id': task_id, 'target_zone': route,
            'simulation_only': True, 'robot_execution_enabled': False,
            'operator_approval_required': True,
            'waypoints': {
                'pick_approach': 'SIM_ONLY_PICK_APPROACH',
                'pick_grasp': 'SIM_ONLY_PICK_GRASP',
                'pick_lift': 'SIM_ONLY_PICK_LIFT',
                'drop_approach': f'SIM_ONLY_{route}_APPROACH',
                'drop_release': f'SIM_ONLY_{route}_RELEASE',
                'drop_retreat': f'SIM_ONLY_{route}_RETREAT',
            },
            'note': 'Fictional symbolic plan for offline development, not taught poses, approved clearance or real trajectory.'}
        for name, payload in (('proposals', proposal), ('plans', plan)):
            raw = (json.dumps(payload, ensure_ascii=False, indent=2)+'\n').encode('utf-8')
            write_new_or_same(PACK/f'fixtures/{name}/{case}.json', raw)

    files = []
    for path in sorted(PACK.rglob('*')):
        if (not path.is_file() or '__pycache__' in path.parts
                or path.name == 'MANIFEST.json' or path.suffix == '.pyc'):
            continue
        files.append({'path': path.relative_to(PACK).as_posix(), 'sha256': digest(path),
                      'bytes': path.stat().st_size})
    manifest = {'kind': 'offline_fsm_handoff_snapshot',
                'created_at_utc': datetime.now(timezone.utc).isoformat(),
                'source_project': str(ROOT), 'copied_files': copied, 'package_files': files,
                'note': 'Copied reference/config/evidence files are immutable context. Starter hashes describe the initial handoff, not a signature of future student work.',
                'robot_execution_enabled': False}
    (PACK/'MANIFEST.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('Package:', PACK)
    print('Copied source files:', len(copied), '| packaged text/image files:', len(files))


if __name__ == '__main__':
    main()
