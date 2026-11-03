"""Offline class-presence checks in PICK_ZONE; not bounding-box AP or grasp tests."""
import hashlib
import json
from pathlib import Path

from .decision import classify_detection
from .vision_pipeline import adapt_item
from .zones import zone_at_pixel, validate_workspace


def deduplicate_samples(entries):
    unique, duplicates, seen = [], [], {}
    for entry in entries:
        key = entry['image_sha256']
        if key in seen:
            original = seen[key]
            if sorted(original['expected_classes_in_pick_zone']) != sorted(entry['expected_classes_in_pick_zone']):
                raise ValueError('Identical image has conflicting reviewed labels')
            duplicates.append({'sample_id': entry['sample_id'], 'duplicate_of': original['sample_id']})
        else:
            seen[key] = entry
            unique.append(entry)
    return unique, duplicates


def verified_sample(entry):
    image = Path(entry['image_path'])
    is_capture = 'source_session_path' in entry
    source = Path(entry['source_session_path'] if is_capture else entry['source_preview_path'])
    source_hash = entry['source_session_sha256'] if is_capture else entry['source_preview_sha256']
    for path, expected in ((image, entry['image_sha256']), (source, source_hash)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise ValueError('Evaluation evidence changed: '+str(path))
    data = json.loads(source.read_text(encoding='utf-8'))
    if is_capture:
        if data.get('type') != 'unfiltered_rgb_evaluation_capture':
            raise ValueError('Unknown capture session format')
        matches = [a for a in data['attempts'] if a['status'] == 'saved'
                   and Path(a['image_path']).resolve() == image.resolve()
                   and a['image_sha256'] == entry['image_sha256']]
        if len(matches) != 1:
            raise ValueError('Image is not a saved attempt in referenced session')
        if entry.get('label_source') in (None, '', 'scenario_intent'):
            raise ValueError('Capture scenario must not become ground truth without review')
    workspace = data['workspace_snapshot']
    validate_workspace(workspace, require_complete=True)
    if entry['label_scope'] != 'class_presence_in_pick_zone_only':
        raise ValueError('Unsupported label scope')
    expected = entry['expected_classes_in_pick_zone']
    allowed = {'sponge', 'plastic_bottle', 'aluminum_can', 'paper_cup'}
    if not isinstance(expected, list) or any(x not in allowed for x in expected):
        raise ValueError('Invalid reviewed labels')
    return image, workspace


def score_predictions(items, expected, workspace, threshold):
    accepted, excluded = [], []
    for item in items:
        detection, error = adapt_item(item, workspace['image_size'])
        if error:
            excluded.append(error)
            continue
        route = classify_detection(detection, preview_min_confidence=threshold)
        if route['status'] != 'ready_for_localization':
            excluded.append(route['status'])
            continue
        if zone_at_pixel(workspace, detection['center_pixel']) != ['PICK_ZONE']:
            excluded.append('outside_or_overlapping_pick_zone')
            continue
        accepted.append({'class_name': detection['class_name'], 'confidence': detection['confidence'],
                         'center_pixel': detection['center_pixel'], 'target_zone': route['target_zone']})
    labels = [x['class_name'] for x in accepted]
    return {'threshold': threshold, 'accepted_in_pick_zone': accepted,
            'exact_class_list_match': sorted(labels) == sorted(expected),
            'missing_expected_classes': sorted(set(expected)-set(labels)),
            'unexpected_classes': sorted(set(labels)-set(expected)),
            'extra_same_class_outputs': sum(max(0, labels.count(x)-expected.count(x))
                                           for x in set(labels) & set(expected)),
            'excluded_reasons': excluded, 'robot_execution_enabled': False}
