"""heldout_report.py — format the held-out scoring artifact.

Pure python, no ROS imports, so it can be called both from
truth_score_heldout.py (a ROS node) and from scripts/finalize_heldout_report.py
(a plain script that only reads a JSONL results file and scenario_matrix.yaml).
Kept as one function so the two ways of producing a report -- one detector
process scoring the whole split, or one detector process per scenario via
run_heldout_eval.sh -- are formatted identically and therefore directly
comparable.
"""

from __future__ import annotations

__all__ = ["build_report"]


def build_report(results: list[dict], split: str, bag_dir: str,
                  min_matched: int) -> str:
    voids = [r for r in results if r.get("void")]
    errors = [r for r in results if r.get("error")]
    scored = [r for r in results
              if not r.get("void") and not r.get("error")]
    positives = [r for r in scored if r["label"] == "positive"]
    negatives = [r for r in scored if r["label"] == "negative"]

    n_pos_enum = sum(1 for r in results if r["label"] == "positive")
    n_neg_enum = sum(1 for r in results if r["label"] == "negative")

    recalled = sum(1 for r in positives if r["recalled"])
    fired = sum(1 for r in negatives if r["fired"])

    lines = [
        "",
        "=" * 70,
        "  HuitzilinReflex Held-out Perception Scoring — TRUTH-ATTRIBUTED",
        f"  Split: {split}   Bag dir: {bag_dir}   K={min_matched}",
        "=" * 70,
        "",
    ]
    for r in results:
        if r.get("void"):
            lines.append(f"  {r['id']:5s} {r['label']:8s} VOID — {r['void_reason']}")
        elif r.get("error"):
            lines.append(f"  {r['id']:5s} {r['label']:8s} ERROR — {r['error']}")
        elif r["label"] == "positive":
            errs = r["errors_m"]
            err_str = (f"errors_m=[{min(errs):.2f}..{max(errs):.2f}] "
                       f"mean={sum(errs)/len(errs):.2f}" if errs else "errors_m=[]")
            lines.append(
                f"  {r['id']:5s} positive matched={r['matched']:2d} "
                f"unmatched={r['unmatched']:2d} in_window={r['in_window']:2d} "
                f"K={r['min_matched']} RECALLED={r['recalled']} "
                f"win_rel=[{r['window_rel'][0]:.2f},{r['window_rel'][1]:.2f}] {err_str}"
            )
        else:
            lines.append(
                f"  {r['id']:5s} negative matched={r['matched']:2d} "
                f"unmatched={r['unmatched']:2d} in_window={r['in_window']:2d} "
                f"K={r['min_matched']} FIRED={r['fired']} "
                f"win_rel=[{r['window_rel'][0]:.2f},{r['window_rel'][1]:.2f}]"
            )

    recall = recalled / n_pos_enum if n_pos_enum else 0.0
    fp_rate = fired / n_neg_enum if n_neg_enum else 0.0

    lines += [
        "",
        "-" * 70,
        f"  Positives enumerated (denominator, per section 4.1): {n_pos_enum}",
        f"  Positives RECALLED (matched >= K): {recalled}/{n_pos_enum}"
        f"  = {recall*100:.1f}%",
        f"  Negatives enumerated (denominator, per section 4.2): {n_neg_enum}",
        f"  Negatives FIRED (unmatched >= K): {fired}/{n_neg_enum}"
        f"  = {fp_rate*100:.1f}%",
        f"  Voids: {len(voids)}  {[v['id'] for v in voids]}",
        f"  Errors: {len(errors)}  {[(e['id'], e['error']) for e in errors]}",
        "",
    ]
    if recalled == n_pos_enum and n_pos_enum > 0:
        lines.append(f'  CLAIM PER SECTION 5: "100 % recall on {n_pos_enum} '
                      f'held-out positive scenarios"')
    elif n_pos_enum - recalled == 1:
        lines.append(f"  CLAIM PER SECTION 5: \"{recall*100:.0f}% recall "
                      f"({recalled}/{n_pos_enum})\". THE 100% CLAIM IS DROPPED.")
    else:
        lines.append(f"  CLAIM PER SECTION 5: measured figure only "
                      f"({recalled}/{n_pos_enum}) — investigation required.")
    lines += ["=" * 70, ""]
    return "\n".join(lines)
