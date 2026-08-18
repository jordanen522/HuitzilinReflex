"""
null_attribution_mc.py — the Monte Carlo harness behind Task 5c's null table.

NOT a pytest module (no `test_` prefix, so pytest does not collect it).
test_score_bags_logic.py imports it and pins one published figure; run it
standalone to reproduce the whole table:

    PYTHONPATH=src/huitzilin_perception \
      python3 src/huitzilin_perception/test/null_attribution_mc.py

Shipped because two load-bearing claims rest on it (Task 5c fix round 1,
MEDIUM-5): "the reimplementation of the 5b rule reproduces the reviewer's
published table", and "at the measured in-window counts the 5c rule's noise
attribution rate is <interval>, so 0 of 12 is below the noise floor". Neither
could be re-run by anyone before this file existed -- and the first time
anyone did re-run the second one, in fix round 2, the shipped interval turned
out to be wrong (see MEASURED_N_WIN below). Running this file prints both.

THE NULL MODEL
--------------
A bag's in-window /threat/centroid stream, if it contains no ball and no
tracking whatsoever, is n detections whose ranges are independent draws with
no closing physics at all:

  - n detections, timestamps at the depth camera's DEPTH_CLOUD_HZ (15 Hz),
    consecutive and evenly spaced — the densest, most favourable-to-the-rule
    arrival pattern, since gaps only break runs.
  - ranges iid Uniform[roi_min_range_m, roi_max_range_m] = [0.30, 5.00] m,
    the ROI band the detector can report at all.
  - no correlation between successive ranges. This is the assumption that
    makes the model CONSERVATIVE in the direction that matters: real FP
    ranges are clustered (successive terrain patches at similar depths), and
    clustering produces MORE long monotone runs than independence, not
    fewer. So the real spurious-attribution rate should be at least this
    high, and the measured control is indeed worse than this null predicts.

TRIAL COUNT: 200_000 per (n, rule) point, the same count the Task 5b
reviewer used, seeded so the table is reproducible.

WHAT THE MODEL IS NOT
---------------------
It is not a model of the detector. It answers exactly one question — "how
often does this decision rule fire on structureless input?" — which is the
question a rule that claims to identify a closing ball has to answer before
its firing count means anything.

Note the 5b sign rule is SCALE- AND DISTRIBUTION-FREE: it reads only the
sign of successive differences, so its curve depends on n alone and not on
the range bounds. That is why its reproduction of the reviewer's published
figures is an exact check on this harness rather than a coincidence of
parameter choice. The 5c rate rule is not scale-free — it depends on the
range bounds, the sample rate and the band — so its column is only as good
as the three constants above.
"""

from __future__ import annotations

import random

from huitzilin_perception.score_bags_logic import (
    DEFAULT_MIN_CLOSING_RUN,
    DEPTH_CLOUD_HZ,
    LIBRARY_MAX_BALL_SPEED_MPS,
    attribute_closing_ball,
    longest_closing_run,
)

ROI_MIN_RANGE_M = 0.30   # detector.yaml
ROI_MAX_RANGE_M = 5.00   # detector.yaml
DEFAULT_TRIALS = 200_000
DEFAULT_SEED = 20260817

# ── the published figures, and the measurement they were computed FROM ──────
#
# Task 5c fix round 2, HIGH (F2-1). The three null-model figures quoted in
# detector.yaml were computed at ONE PARTICULAR n_win vector and then went
# stale when the fix round 1 window change moved that vector -- the previous
# round's (13, 3, 8, 11, 5, 4, 3, 7, 4, 16, 4, 5) became the one below, and
# nobody recomputed. The published interval's floor of 0.10 was in fact never
# right at EITHER vector; the true floor sits at n = 3 (S02 and S07).
#
# So the vector lives here, in code, next to the figures it produces, and
# test_score_bags_logic.py recomputes the figures from it and asserts they
# still match. A future window change now has to move all three together or
# fail a test -- which is what the earlier version of this block was supposed
# to guarantee and could not, being prose.
#
# PROVENANCE: n_win per original positive, S01..S12, from the corrected-window
# scoring run of 2026-08-17 (Dell artifacts week3_regression_20260817_163942
# train + _164822 test, tabulated in task-5c-fix1-report.md section 9). These
# are MEASURED counts, an input to the null model and not an output of it.
MEASURED_N_WIN = (10, 3, 8, 11, 5, 4, 3, 7, 4, 12, 4, 5)
MEASURED_N_WIN_IDS = ("S01", "S02", "S03", "S04", "S05", "S06",
                      "S07", "S08", "S09", "S10", "S11", "S12")

# What null_summary(MEASURED_N_WIN) returns at DEFAULT_TRIALS, and therefore
# what detector.yaml is allowed to say. Recompute, do not edit to taste.
PUBLISHED_RATE_MIN = 0.038      # at n = 3 (S02, S07)
PUBLISHED_RATE_MAX = 0.291      # at n = 12 (S10)
PUBLISHED_P_NONE = 0.160        # P(0 of 12 attributions | null)
PUBLISHED_EXPECTED = 1.64       # E[attributions | null]


def null_stream(
    n: int,
    rng: random.Random,
    hz: float = DEPTH_CLOUD_HZ,
    r_lo: float = ROI_MIN_RANGE_M,
    r_hi: float = ROI_MAX_RANGE_M,
) -> list[tuple[float, float]]:
    """One draw from the null model: n (t, range) pairs, no closing physics."""
    dt = 1.0 / hz
    return [(i * dt, rng.uniform(r_lo, r_hi)) for i in range(n)]


def fires_sign_rule(points: list[tuple[float, float]]) -> bool:
    """
    Task 5b's refuted rule: a run of >= 3 strictly decreasing ranges, sign
    only, with NO gap bound (5b had none). At 15 Hz every gap is 0.067 s, so
    passing float("inf") only makes the reproduction exact by construction
    rather than by luck.
    """
    return longest_closing_run(points, max_gap_s=float("inf")) >= \
        DEFAULT_MIN_CLOSING_RUN


def fires_rate_rule(
    points: list[tuple[float, float]],
    ball_speed_mps: float = LIBRARY_MAX_BALL_SPEED_MPS,
) -> bool:
    """
    Task 5c's replacement, run at the LIBRARY MAXIMUM band (3.49, 20.49] —
    the most permissive band any scenario gets, so the noise rate reported
    is an upper bound over the library rather than a best case.
    """
    return attribute_closing_ball(points, ball_speed_mps)["attributable"]


def p_fires(rule, n: int, trials: int = DEFAULT_TRIALS,
            seed: int = DEFAULT_SEED) -> float:
    """Fraction of `trials` null streams of length n on which `rule` fires."""
    rng = random.Random(seed + n)
    return sum(rule(null_stream(n, rng)) for _ in range(trials)) / trials


def table(ns=(5, 10, 15, 20, 30, 50), trials: int = DEFAULT_TRIALS) -> None:
    print(f"null model: iid U[{ROI_MIN_RANGE_M}, {ROI_MAX_RANGE_M}] m at "
          f"{DEPTH_CLOUD_HZ} Hz, {trials} trials/point")
    print(f"{'n':>4}  {'P(5b sign rule)':>16}  {'P(5c rate rule)':>16}")
    for n in ns:
        print(f"{n:>4}  {p_fires(fires_sign_rule, n, trials):>16.4f}  "
              f"{p_fires(fires_rate_rule, n, trials):>16.4f}")


def null_summary(n_win=MEASURED_N_WIN, trials: int = DEFAULT_TRIALS) -> dict:
    """
    The three figures detector.yaml quotes, computed from ONE n_win vector.

    Returns {"n_win", "rates", "rate_min", "rate_max", "p_none", "expected"}.

      rates    -- P(the 5c rate rule fires | null) per scenario, in n_win order
      p_none   -- P(zero of the len(n_win) positives attributes | null),
                  the scenarios treated as independent draws
      expected -- E[number of attributions | null] = sum(rates)

    p_none is the probability the OBSERVED 0-of-12 arises from pure noise, so
    it is the number that says how decisive 0/12 is. It is NOT a small number.
    """
    rates = [p_fires(fires_rate_rule, n, trials) for n in n_win]
    p_none = 1.0
    for r in rates:
        p_none *= (1.0 - r)
    return {
        "n_win": tuple(n_win),
        "rates": rates,
        "rate_min": min(rates),
        "rate_max": max(rates),
        "p_none": p_none,
        "expected": sum(rates),
    }


def summary_table(n_win=MEASURED_N_WIN, trials: int = DEFAULT_TRIALS) -> None:
    s = null_summary(n_win, trials)
    print(f"\nper-scenario null attribution rate at the MEASURED n_win, "
          f"{trials} trials/point")
    print("  n_win vector: " + ", ".join(
        f"{i}={n}" for i, n in zip(MEASURED_N_WIN_IDS, s["n_win"])))
    print("  rates sorted: " + " ".join(f"{r:.3f}"
                                        for r in sorted(s["rates"])))
    print(f"  range              : {s['rate_min']:.3f} - {s['rate_max']:.3f}")
    print(f"  P(0 of {len(s['n_win'])} | null) : {s['p_none']:.3f}")
    print(f"  E[attributions]    : {s['expected']:.2f}")
    print("  ^ these three are what detector.yaml quotes. If they no longer "
          "match it,\n    the n_win vector moved and the yaml is stale — "
          "recompute, do not edit.")


if __name__ == "__main__":
    table()
    summary_table()
