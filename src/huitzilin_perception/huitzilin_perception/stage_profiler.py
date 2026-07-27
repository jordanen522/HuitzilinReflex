"""Per-stage wall-clock profiler for the detector cloud callback.

Why this exists (docs/JOURNAL.md, 2026-07-27): the dodge latency was traced to
detector *compute*, not transport. Clouds arrive every ~77 ms of wall time
(15 Hz at the measured RTF 0.864) and `_cloud_cb` was taking ~160 ms mean, so
the depth-5 queue backs up and every cloud is already stale on arrival. The
fix needs to know WHICH stage is hot; guessing which of ten numpy passes
dominates is exactly the kind of assumption that produced the wrong RTF and
the wrong cruise speed earlier in the week.

Wall time, deliberately. Sim time cannot measure this: the callback blocks the
single-threaded executor, so no `/clock` message is processed while it runs and
the sim clock reads the same value at entry and exit. See the comment in
`detector_node._cloud_cb`.

Totals, not means, drive the report ordering. A stage costing 5 ms on every
frame matters more than one costing 40 ms on the two frames per minute that
reach it, and the pipeline returns early at eight different points so the
per-stage call counts genuinely differ. The report carries both.
"""

from __future__ import annotations

import time
from contextlib import contextmanager


class StageProfiler:
    """Accumulate per-stage wall time, and emit a report every N frames.

    Intended use inside a callback that may return early::

        with prof.stage("deserialize"):
            pts = read(msg)
        ...
        prof.frame_done()          # only on the paths that complete
        if prof.due():
            log(prof.report())     # report() resets the accumulators

    ``frame_done`` is separate from ``stage`` because what counts as a frame is
    the caller's policy, not the profiler's. `detector_node` counts every
    arriving cloud including its early-out paths, since a single-threaded
    executor is delayed by a frame that bails just as much as by one that
    publishes; the resulting ms/frame is the mean cost per arriving cloud,
    which is the figure that has to fit inside the arrival period. The
    per-stage call counts then double as the pipeline's funnel: a stage with
    far fewer calls than there were frames is downstream of an early return.
    """

    def __init__(self, window_frames: int = 30) -> None:
        self._window = max(1, int(window_frames))
        # name -> [total_seconds, calls], insertion-ordered so a tie in total
        # time reports in pipeline order rather than arbitrarily.
        self._stages: dict[str, list[float]] = {}
        self._frames = 0
        self._wall0 = time.monotonic()

    # ── Recording ────────────────────────────────────────────────────────────

    @contextmanager
    def stage(self, name: str):
        t0 = time.monotonic()
        try:
            yield
        finally:
            # finally, not a plain trailing statement: an exception mid-stage
            # (the deque-mutation race cost a whole battery run to find) must
            # not silently drop the timing that would have localised it.
            dt = time.monotonic() - t0
            slot = self._stages.get(name)
            if slot is None:
                self._stages[name] = [dt, 1]
            else:
                slot[0] += dt
                slot[1] += 1

    def frame_done(self) -> None:
        self._frames += 1

    # ── Reporting ────────────────────────────────────────────────────────────

    def due(self) -> bool:
        return self._frames >= self._window

    def reset(self) -> None:
        self._stages = {}
        self._frames = 0
        self._wall0 = time.monotonic()

    def report(self) -> str:
        """One-line summary, hottest stage first. Resets the accumulators.

        The arrival-period budget is not baked in here — the caller knows it.
        """
        frames = self._frames
        elapsed = max(1e-9, time.monotonic() - self._wall0)
        measured_s = sum(v[0] for v in self._stages.values())
        ranked = sorted(self._stages.items(), key=lambda kv: -kv[1][0])
        parts = []
        for name, (total_s, calls) in ranked:
            share = 100.0 * total_s / measured_s if measured_s > 0 else 0.0
            per_call_ms = 1000.0 * total_s / calls if calls else 0.0
            parts.append(f"{name} {per_call_ms:.1f}ms x{calls} ({share:.0f}%)")
        per_frame_ms = 1000.0 * measured_s / frames if frames else 0.0
        head = (f"profile: frames={frames} in {elapsed:.1f}s wall, "
                f"{per_frame_ms:.1f} ms/frame measured")
        self.reset()
        return head + " | " + " | ".join(parts) if parts else head
