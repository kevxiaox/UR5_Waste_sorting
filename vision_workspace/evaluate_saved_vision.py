"""Run local CPU detector on a reviewed image inventory; never opens hardware."""
import argparse
from datetime import datetime
import hashlib
import json
from pathlib import Path

import cv2

from waste_sorting.vision_evaluation import verified_sample, score_predictions, deduplicate_samples

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=ROOT/'data/vision_evaluation/manifest.json')
    args = parser.parse_args()
    manifest_bytes = args.manifest.read_bytes()
    manifest = json.loads(manifest_bytes)
    # Verify all reviewed evidence before loading weights or producing results.
    verified = [(entry, *verified_sample(entry)) for entry in manifest['samples']]
    unique, duplicates = deduplicate_samples(manifest['samples'])
    ids = {entry['sample_id'] for entry in unique}
    verified = [sample for sample in verified if sample[0]['sample_id'] in ids]
    from detector import detect_waste, detector_info
    info = detector_info()
    if info['weights_sha256'] != manifest['weights_sha256']:
        raise ValueError('Weights differ from reviewed inventory')
    output = ROOT/'data/vision_evaluation/runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for entry, path, workspace in verified:
        image = cv2.imread(str(path))
        if image is None or [image.shape[1], image.shape[0]] != workspace['image_size']:
            raise ValueError('Cannot read original-sized image: '+str(path))
        raw, marked = detect_waste(image)
        data = json.loads(raw)
        if data['image_resolution'] != workspace['image_size'] or data['total_detected'] != len(data['items']):
            raise ValueError('Detector JSON contract mismatch')
        row = {'sample_id': entry['sample_id'], 'expected_classes_in_pick_zone': entry['expected_classes_in_pick_zone'],
               'image_path': str(path), 'image_sha256': entry['image_sha256'],
               'detector_items': data['items'], 'threshold_checks': [
                   score_predictions(data['items'], entry['expected_classes_in_pick_zone'], workspace, t)
                   for t in (.45, .80)]}
        rows.append(row)
        (output/(entry['sample_id']+'_detector.json')).write_text(raw+'\n', encoding='utf-8')
        if not cv2.imwrite(str(output/(entry['sample_id']+'_marked.png')), marked):
            raise OSError('Failed to save detector image')
        print(entry['sample_id'], [check['exact_class_list_match'] for check in row['threshold_checks']], flush=True)
    totals = {str(t): {'exact_class_list_matches': sum(row['threshold_checks'][i]['exact_class_list_match'] for row in rows),
                       'reviewed_images': len(rows)} for i, t in enumerate((.45, .80))}
    result = {'type': 'selected_saved_images_roi_class_presence_review', 'manifest_path': str(args.manifest.resolve()),
              'manifest_sha256': hashlib.sha256(manifest_bytes).hexdigest(), 'detector': info,
              'samples': rows, 'descriptive_counts': totals, 'excluded_exact_duplicates': duplicates,
              'inventory_image_count': len(manifest['samples']), 'robot_execution_enabled': False,
              'limitations': manifest['limitations'] + [
                  'No human bounding boxes: exact class-list match is not localization accuracy or mAP.',
                  'Thresholds filter adapter output; underlying model prediction floor and NMS remain unchanged.',
                  'No camera session or depth validation; offline ROI membership only.',
                  'No confidence calibration or production threshold recommendation from this small selected set.']}
    (output/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    lines = ['# 已保存图片的离线识别检查', '',
             '仅检查PICK_ZONE中的类别输出。标签来自助手原图目视审阅，尚待用户复核；未使用模型预测生成标签。', '',
             '| 样本 | 实际待抓区类别 | 0.45输出 | 0.80输出 |', '| --- | --- | --- | --- |']
    for row in rows:
        outputs = []
        for check in row['threshold_checks']:
            labels = ', '.join(x['class_name']+f" {x['confidence']:.3f}" for x in check['accepted_in_pick_zone']) or '无'
            outputs.append(labels+('（一致）' if check['exact_class_list_match'] else '（不一致）'))
        lines.append('| '+row['sample_id']+' | '+(', '.join(row['expected_classes_in_pick_zone']) or '空')+' | '+' | '.join(outputs)+' |')
    lines += ['', '描述性计数：'+str(totals), '', '限制：']
    lines += ['- 已排除完全重复图像：'+str(duplicates)]
    lines += ['- '+x for x in result['limitations']]
    (output/'report.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print('Saved:', output/'report.md')
    print('Counts:', totals)


if __name__ == '__main__':
    main()
