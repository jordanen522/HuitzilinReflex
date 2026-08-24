# Known issues

Characterised limitations of the rendered long-range sensor lane, with the evidence behind
each. None of these blocks a requirement in `requirements.md`; the measured envelope in
`RESULTS.md` §8 stands independently of them. They are documented rather than solved — the
rendered lane is left in the state described here, with the figures below as its measured
result.

---

## 1. Rendered-lane detector recall is unreliable

**Symptom.** On the `tune_rendered` split (RT01–RT10): 2/6 positive recall (RT04, RT05),
2/4 false-fire (RT09, RT10). RT01 lands one frame short of the `K=3` match threshold
(`matched=2`) carrying the tightest position error on the split
(`errors_m=[0.03..0.04]`); RT02, RT03 and RT06 are flat zeros.

**Repro.**
```bash
SPLIT=tune_rendered BAG_PREFIX=rt_ ./scripts/run_heldout_eval.sh
```
Score per-scenario-restart, not a whole split in one detector process — shared
background-map state leaks between scenarios and understates recall.

**What the evidence shows.** Ball-sized candidates do form on most frames even in the
failing scenarios (`debug_funnel:=true`), so this is not a missing-candidate problem. No
single-frame geometric feature separates the ball from a same-sized clutter fragment:
point count, extent, and density read alike in one frame, and two independent tie-break
designs (count, density) produced byte-identical results on this split. A temporal
ballistic-consistency confirm-gate traded recall for false-fire rather than improving
either, and was reverted.

RT01 and RT05 partially recall, so their lever looks like frame budget/volume rather than
discrimination. RT02, RT03 and RT06 failed under every geometric and temporal approach
tried; their debug-funnel traces put detections 18–20 m from the true ball position — a
different region of the scene, not a near miss.

**Status.** Characterised, not solved. One further diagnostic was scoped — projecting the
ball's ground-truth position into the RT02/RT03/RT06 point clouds, to establish whether
those bags contain a geometrically resolvable ball encounter at all — and was **not
pursued**. The figures above are the lane's measured state; anyone resuming this work
starts from that diagnostic, not from another pipeline-side tuning pass.

---

## 2. G01 fires far more often than the patrol baseline

**Symptom.** The fidelity gate's G01 row (14 m/s) fires 12/17 where the reference patrol
battery fired 0/17. Both save 0/17, so the divergence is confined to whether a doomed
command fires at all, not to whether anything is saved.

**What is established.** Every G01 row has `success=False`; each `dodged=True` row's note
reads "dodged but min_dist ≤ hit_radius" (0.064–0.177 m). `tca_s` on every fire is
0.06–0.15 s, far below the 0.798 s LD50, and 10/12 fires were latency-over-budget, up to
441 ms against a reference tail of 282 ms. Small-n is ruled out (n=17 rerun, Fisher exact
p ≈ 3e-5 against the reference). Duplicate `/clock` and depth-noise re-stamping are ruled
out by source inspection: `depth_noise_node.py` copies `src.header` verbatim, and only one
`clock_bridge` node exists in either launch graph.

**Status.** Characterised, not solved. The one remaining untested candidate is the
`depth_noise` hop's RELIABLE/depth=5 QoS causing bursty delivery under this box's degraded
RTF (~0.33 with depth rendering), letting the detector accumulate three track updates from
a late burst. The test that would settle it — pointing the detector directly at
`/oak/points_rendered`, bypassing `depth_noise`, and rerunning G01 — was **not pursued**.

**Consequence.** The 14 m/s finding itself is not in dispute — 0/17 saves in either lane.
What is not established is the R01–R04 plumbing, since a fire-rate gap this large also
raises a question about R04's clean-miss negative control.

---

## 3. The rendered dodge battery was never run to completion

`week7_rendered_battery.yaml` (G01/G02 fidelity gate, R01–R04 hover envelope at
8/14/20 m/s) has not been run to completion, for the reason in issues 1 and 2: a detector
that misses most tracked balls in its own tune split cannot produce a trustworthy dodge
number. No rendered-lane dodge figure exists, and none should be quoted.

---

## 4. Held-out bags H01, H02, H03, H16 are spent

These four were used diagnostically rather than scored as an official pass. Treat them as
spent if a fresh held-out evaluation is ever run, and capture clean replacements first —
see the ground-truth scoring section of `bag_capture_runbook.md`.
