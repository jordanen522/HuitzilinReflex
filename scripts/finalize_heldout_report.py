#!/usr/bin/env python3
"""finalize_heldout_report.py — turn a per-scenario JSONL into the official
held-out artifact.

run_heldout_eval.sh restarts the detector process between scenarios (fixing
a cross-scenario background-map contamination bug, where scoring several
scenarios in one shared process let one scenario's background map leak into
the next) and has truth_score_heldout write one JSON line per scenario via
-p results_jsonl:=<path>. This script reads that
file back, checks it has exactly one line per id in the split (no duplicates,
nothing missing -- a partial file is refused rather than silently scored as
if it were complete), and writes the same report format truth_score_heldout
would have written for a single-process run.

Usage:
  python3 scripts/finalize_heldout_report.py \
      --results-jsonl /tmp/heldout_results.jsonl \
      --scenario-matrix src/huitzilin_perception/config/scenario_matrix.yaml \
      --split heldout --bag-dir /data/huitzilin_bags --min-matched 3 \
      --output-file /tmp/heldout_truth_scoring_final.txt
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

# Import the ROS-free formatter directly from its source tree -- this script
# is meant to run without a sourced ROS workspace (it does no ROS I/O at
# all), so it inserts the package's own src dir onto sys.path rather than
# relying on `ros2 pkg prefix` / an installed overlay.
_PKG_SRC = (Path(__file__).resolve().parent.parent
            / "src" / "huitzilin_perception")
sys.path.insert(0, str(_PKG_SRC))
from huitzilin_perception.heldout_report import build_report  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--results-jsonl", required=True, type=Path)
    ap.add_argument("--scenario-matrix", required=True, type=Path)
    ap.add_argument("--split", default="heldout")
    ap.add_argument("--bag-dir", default="/data/huitzilin_bags")
    ap.add_argument("--min-matched", type=int, default=3)
    ap.add_argument("--output-file", required=True, type=Path)
    args = ap.parse_args()

    with open(args.scenario_matrix) as f:
        matrix = yaml.safe_load(f)
    splits = matrix.get("split", {})
    if args.split not in splits:
        print(f"ERROR: split {args.split!r} not in {args.scenario_matrix}",
              file=sys.stderr)
        return 1
    expected_ids = list(splits[args.split])

    results_by_id: dict[str, dict] = {}
    with open(args.results_jsonl) as f:
        for lineno, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            rid = row.get("id")
            if rid in results_by_id:
                print(f"ERROR: duplicate result for {rid!r} at line {lineno} "
                      f"of {args.results_jsonl} — refusing to finalize on an "
                      f"ambiguous input rather than silently taking the last one",
                      file=sys.stderr)
                return 1
            results_by_id[rid] = row

    missing = [sid for sid in expected_ids if sid not in results_by_id]
    extra = [rid for rid in results_by_id if rid not in expected_ids]
    if missing or extra:
        print(f"ERROR: {args.results_jsonl} does not match split "
              f"{args.split!r} exactly.", file=sys.stderr)
        if missing:
            print(f"  missing: {missing}", file=sys.stderr)
        if extra:
            print(f"  unexpected (not in split): {extra}", file=sys.stderr)
        print("  Refusing to finalize a partial or contaminated run.",
              file=sys.stderr)
        return 1

    # Report in split-declared order, not JSONL arrival order, so a report
    # produced this way is byte-identical in row order to a single-process run.
    results = [results_by_id[sid] for sid in expected_ids]

    report = build_report(results, args.split, str(args.bag_dir),
                           args.min_matched)
    args.output_file.parent.mkdir(parents=True, exist_ok=True)
    args.output_file.write_text(report, encoding="utf-8")
    print(report)
    print(f"Report saved to: {args.output_file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
