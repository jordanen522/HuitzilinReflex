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
attribution rate is 0.10-0.36, so 0 of 12 is below the noise floor". Neither
could be re-run by anyone before this file existed.

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


if __name__ == "__main__":
    table()
