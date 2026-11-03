"""Offline candidate fit/audit only. Does not connect to robot or camera."""
import argparse
import json
from pathlib import Path

from waste_sorting.point_calibration import analyze_observations, freeze_training_manifest

ROOT = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,default=ROOT/"data/tool_robot_observations")
    parser.add_argument("--report",type=Path,default=ROOT/"data/calibration_analysis/candidate_report.json")
    parser.add_argument("--training-manifest",type=Path,default=ROOT/"data/calibration_analysis/training_manifest.json")
    parser.add_argument("--freeze-training",action="store_true",help="Freeze current accepted distinct records; refuses overwrite")
    args = parser.parse_args()
    paths = sorted(args.input.glob("*/measurement.json"))
    if args.freeze_training:
        freeze_training_manifest(paths,args.training_manifest)
    manifest = json.loads(args.training_manifest.read_text(encoding="utf-8")) if args.training_manifest.exists() else None
    report = analyze_observations(paths,manifest)
    args.report.parent.mkdir(parents=True,exist_ok=True)
    args.report.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False))
    print("Offline report:",args.report)


if __name__ == "__main__":
    main()
