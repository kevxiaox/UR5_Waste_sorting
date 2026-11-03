"""Portable integrity check only. Does not import ROS or connect to hardware."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def main():
    mappings = json.loads((ROOT/'portable_sources.json').read_text(encoding='utf-8'))
    lookup, source_hashes = {}, {}
    for item in mappings['sources']:
        source = (ROOT/item['relative_path']).resolve()
        if not source.is_relative_to(ROOT):
            raise ValueError('Evidence path outside package')
        raw = source.read_bytes()
        if hashlib.sha256(raw).hexdigest() != item['sha256']:
            raise ValueError('Changed evidence: '+item['relative_path'])
        lookup[item['original_source_path']] = json.loads(raw)
        source_hashes[item['original_source_path']] = item['sha256']
    catalog_raw = (ROOT/'catalog.json').read_bytes()
    if hashlib.sha256(catalog_raw).hexdigest() != mappings['catalog_sha256']:
        raise ValueError('Changed catalog')
    catalog = json.loads(catalog_raw)
    if catalog['robot_execution_enabled'] is not False or catalog['trajectory'] is not None:
        raise ValueError('Unexpected execution settings')
    count = 0
    for obj in catalog['objects']:
        exported = json.loads((ROOT/(obj['object_class']+'_requests.json')).read_text(encoding='utf-8'))
        if exported != obj:
            raise ValueError('Object request file differs from catalog')
        for waypoint in obj['waypoints'].values():
            record = lookup[waypoint['source_path']]
            if source_hashes[waypoint['source_path']] != waypoint['source_sha256']:
                raise ValueError('Evidence mapping differs from catalog source hash')
            if record['actual_q_rad'] != waypoint['joint_positions_rad']:
                raise ValueError('Reference joints differ from copied source')
            if record['configured_tcp_offset_flange_m_rotvec_rad'] != [0]*6:
                raise ValueError('Nonzero recorded TCP')
            count += 1
    print('Integrity OK:', count, 'joint references; zero recorded TCP; original angles preserved.')
    print('No ROS/hardware accessed. This is not trajectory, collision or physical validation.')


if __name__ == '__main__':
    main()
