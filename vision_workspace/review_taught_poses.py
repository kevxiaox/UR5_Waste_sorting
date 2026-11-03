"""Build an offline inventory of saved postures and annotations."""
import json
from pathlib import Path

from waste_sorting.pose_catalog import build_catalog, render_markdown


def main():
    root = Path(__file__).resolve().parent
    catalog = build_catalog(root / 'data/taught_poses')
    output = root / 'data/pose_review'
    output.mkdir(parents=True, exist_ok=True)
    (output / 'catalog.json').write_text(
        json.dumps(catalog, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (output / 'catalog.md').write_text(render_markdown(catalog), encoding='utf-8')
    errors = sum(bool(row['evidence_errors']) for row in catalog['records'])
    rejected = sum(row['review_status'] == 'rejected_table_contact_do_not_reuse'
                   for row in catalog['records'])
    print('OFFLINE ONLY. No hardware connection; automatic replay disabled.')
    print(f'Records: {len(catalog["records"])}; rejected: {rejected}; evidence errors: {errors}')
    print('Saved:', output / 'catalog.md')
    if errors:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
