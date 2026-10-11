# E16 no-cart OLD/FRESH handoff audit

## Finding

**Observed inconsistency, but reconciliation necessity is not established.**
Typical actual-boundary position residuals are about **1.6 cm**, while same-nominal-time
future disagreement is about **0.43 m RMSE**. Future longitudinal timing/progress
revision dominates lateral revision. These successful runs do not establish that
making consecutive predictions agree would improve navigation.

A large startup tangent outlier and a later 12.4 cm boundary residual deserve
inspection. They are diagnostic candidates, not confirmed hard cases or evidence
that a graph should force OLD and FRESH to agree. No correction is implemented.

## Source and integrity

- A: `outputs/nova-e16-hospital-lights-baseline-20261010-01`.
- B: `outputs/nova-e16-no-cart-15-repro-20261011-01`.
- Clean starting local/fetched origin `hardcase-probe`: `66e21b8f00a0444e029da4b74c8b0f0cc794edf2`.
- Fetched upstream/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
- Each has 48 finite native `(30,2)` predictions and 47 adjacent pairs C1→C2…C47→C48.
  C1 alone is bootstrap; **C1→C2 remains in all primary statistics**. Total94 pairs
  are two dependent within-run series, not94 independent experiments.
- Both original configs agree, max_v=1.5m/s, no added cart/human. Existing validators
  confirm continuous OLD-command retention, exact observation poses, original
  controller reconstruction, corner passage, goal-region entry and zero recorded
  nonfloor contacts. Both continue past the goal under the fixed48 budget; this
  is not an official stopping-success benchmark. Different historical/current
  host and reasoning/observation histories remain comparison limitations.
- All700 source files and their inventory remain SHA-256 identical. No source
  run, existing code, checkpoint, controller or environment was changed.

## Definitions and reuse

[Analyzer](../research/analyze_e16_handoff_audit.py) reuses `load_saved`,
`analyze_no_cart_speed.analyze_run` for original gates/controller verification,
`route_geometry.temporal_overlap`, `boundary_seam`, `interpolate`,
`nova_dynamic_geometry.remaining_curve`, and `analyze_handoffs.metrics`.
[Configuration](../configs/research/e16_handoff_audit.json) contains descriptive
phase/selection rules and the existing numerical tangent epsilon, no hard-case score.

Native points are forward/left metres from their **own observation robot origin**.
World has Z up; yaw is counterclockwise from world+X; saved quaternions are wxyz.
For column vectors, world projection is `pW = t_obs + R(yaw_obs) p_native`.
FRESH is never re-anchored to B. Actual measured history and actual SE(2) B stay fixed.
Native output has no yaw or timestamp channel. Timing is assigned from the
source-confirmed nominal +0.1…+3.0s target convention in
[ACTION_SEMANTICS](ACTION_SEMANTICS.md).

**Boundary:** position seam is `||B.xy - FRESH_world(t_switch)||`. The reused
piecewise-linear diagnostic adds observation XY at tau=0 **only as a separate
interpolation anchor**, never as a native waypoint. Outside0…3s, result is null;
no extrapolation. Robot-yaw seam and preceding executed-direction seam are separate
absolute wrapped differences to the right-hand FRESH segment at switch. Executed
direction uses the last0.1sim s, clipped to the FRESH observation time. Tangents are
**derived geometric tangents; not native TIC-VLA yaw**. Length <=1e-6m, absent
history or out-of-domain timing produce nulls with reasons. All94 boundaries and
all aligned segments in these runs are numerically valid; this numerical floor
is not a physical heading-confidence or hard-case criterion. Short-scale zigzags
remain sensitive geometric descriptors, with lengths exported explicitly.

**Future:** let native knot time `T_kj = t_obs,k + 0.1j`, j=1…30. Full overlap is
`[max(T_O1,T_F1), min(T_O30,T_F30)]`. Existing temporal alignment uses FRESH
knots plus exact common endpoints, with linear interpolation of both source curves.
Post-switch **editable overlap** starts at `max(full_start,t_switch)`; an interior
switch endpoint is inserted by interpolating original source curves. No samples
outside native horizons, spatial nearest-neighbor matching or smoothing are used.
RMSE/max use Euclidean world differences; signed and absolute lateral/forward
components use `R_F.T (pF_world-pO_world)` in the FRESH observation frame.
Derived segment tangent differences are wrapped to[-pi,pi]. Statistics weight
sample points/segments equally, not continuous time or path length.

C1→C2 has a1.800000094sim s observation gap and12 common samples; all other pairs
have a0.500000026sim s gap and25 samples. Do not round away actual saved clocks.
Post-switch sample counts are12/25, except B C47→C48 has24. Full vs editable metrics
are close because switches mostly occur around the first FRESH target time.

**Commands:** `delta_v/w = new - previously applied OLD`. CSV row at the switch
physics tick still records OLD; the request application snapshot records the
subsequent target replacement at the same physical pose. The next physics row
carries FRESH. That exact order is checked. Separate measured forward/yaw-rate
changes from switch to next tick (~1/60sim s) are exported; they are not instantaneous
command jumps or isolated causal responses.

**Clocks:** exact observation/action-start/model-ready/detection/application wall
and monotonic clocks are exported, alongside simulation observation/switch times.
Action latency is ready−action-start. Observation-to-switch includes capture/IPC/
main-thread work and must not be called model inference latency. Exact model-ready
simulator pose/time is unavailable and remains null. Transport is measured FRESH
observation→application XY distance and wrapped yaw change, not velocity×wall time.

## Within-run distributions

Every cell below is **median / P95 / max**; each primary metric has47 valid pairs
per run. P95 is NumPy linear interpolation, descriptive only. Complete signed and
absolute metrics and valid counts, including every phase, are in
`phase_statistics.csv` and `summary.json` in the output directory.

| Metric | A historical | B reproduced |
|---|---:|---:|
| Boundary position [m] | 0.0159 / 0.0552 / 0.0742 | 0.0162 / 0.0686 / 0.1243 |
| Boundary robot yaw / FRESH tangent [deg] | 1.7225 / 5.4243 / 125.2338 | 2.7134 / 7.5863 / 127.2307 |
| Executed direction / FRESH tangent [deg] | 2.0692 / 5.2612 / 125.1654 | 2.8088 / 7.4010 / 126.8814 |
| Full overlap RMSE [m] | 0.4367 / 0.5910 / 1.1397 | 0.4250 / 0.6304 / 1.1649 |
| Post-switch overlap RMSE [m] | 0.4367 / 0.5910 / 1.1406 | 0.4250 / 0.6304 / 1.1666 |
| Full maximum displacement [m] | 0.5569 / 0.8487 / 1.3171 | 0.5308 / 0.9100 / 1.3588 |
| Full mean absolute forward [m] | 0.4302 / 0.5687 / 1.1272 | 0.4170 / 0.6160 / 1.1514 |
| Full mean absolute lateral [m] | 0.0227 / 0.0961 / 0.1094 | 0.0221 / 0.0970 / 0.1647 |
| Full maximum absolute lateral [m] | 0.0634 / 0.2202 / 0.2926 | 0.0575 / 0.2874 / 0.4207 |
| Full mean derived tangent difference [deg] | 37.9300 / 55.6201 / 60.8575 | 38.1882 / 54.5738 / 63.4415 |
| Full maximum derived tangent difference [deg] | 165.1853 / 178.1416 / 179.9373 | 163.5111 / 175.8455 / 179.2684 |
| Signed command delta_v [m/s] | 0.0000 / 0.0000 / 0.0000 | 0.0000 / 0.0000 / 0.0000 |
| Signed command delta_w [rad/s] | -0.0029 / 0.0627 / 0.1264 | -0.0018 / 0.0572 / 0.1964 |
| Absolute command delta_w [rad/s] | 0.0152 / 0.0956 / 0.1379 | 0.0229 / 0.0925 / 0.1964 |
| Action-call wall time [s] | 0.0538 / 0.0984 / 0.1126 | 0.0553 / 0.1144 / 0.2408 |
| Observation→switch wall time [s] | 0.5286 / 0.5580 / 0.5641 | 0.5352 / 0.5733 / 0.6777 |
| Observation→switch simulation time [s] | 0.1000 / 0.1450 / 0.1500 | 0.1000 / 0.1500 / 0.2833 |
| Pending measured XY transport [m] | 0.1501 / 0.2176 / 0.2252 | 0.1501 / 0.2251 / 0.4254 |
| Pending absolute measured yaw change [deg] | 0.3862 / 1.6988 / 1.8941 | 0.4171 / 2.0994 / 2.4560 |

Full and post-switch medians agree to the shown precision. Editable maximum RMSE
is1.1406/1.1666m; full maximum is1.1397/1.1649m. No extrapolation was used to
create a boundary or overlap. Physical motion occurs during pending inference
in these **continuous** runs; this audit does not use the earlier frozen8-chunk run.

### Startup and tangent interpretation

Both largest heading differences occur at **C1→C2**:125.234/127.231deg,
while position seams are only0.009118/0.002664m and command delta_w is
−0.001426/+0.006597rad/s. FRESH native point1→2 displacement is
A`[-0.009765625,-0.013824463]`m and B`[-0.009277344,-0.012207031]`m.
Segment lengths0.016926/0.015332m are well above the numerical degeneracy floor;
this is a resolved local backward segment, not division by zero. It does not show
the robot executing a125deg turn. The controller selects a farther lookahead and
does not track every timed waypoint or native tangent.

The same C1→C2 pair has maximum overlap RMSE in each run. OLD was observed at0s,
bootstrap applied around1.8s, and FRESH was observed at1.8s after almost stationary
startup. Comparing the OLD late horizon with FRESH early horizon gives a large
nominal progress disagreement despite similar spatial route direction. This is a
startup/timing diagnostic, not clean evidence of a route-shape failure.

For transparency, the supplemental **excluding bootstrap OLD** group has46 pairs
without discarding any primary data. Robot-heading median/P95/max is
A1.6966/4.7095/6.8654deg and B2.5618/5.4975/9.0720deg. RMSE is
A0.4252/0.5790/0.6387m and B0.4237/0.5966/0.7288m.

Large future tangent statistics also need care: of1,392 native segments per run,
A143 and B125 have negative **native forward** displacement; none is numerically
zero. Minimum lengths are0.000488/0.001465m. This local zigzag/backtracking makes
segment angles large even when the lateral shape difference is centimetric.
Negative native-forward steps do not individually establish an invalid world
route, and native BF16 outputs do not define reliable robot yaw targets.
`*_tangent_diagnostics.json` preserves all step lengths and first-segment values.

## Physical-progress phases

Original gate: corridor center Y[9.457836,11.741243]m, exit X<=2.552104m,
robot disk radius0.606983m. Original dense corner-entry and crossing-plus-one
application rules are reused unchanged. A entry/completion:7.616667/10.883334s;
B:7.533334/12.883334s. B can leave the Y band while turning, so the phase remains
stateful until gate completion. It is a geographic turn phase, not a claim that
every instant has positive yaw rate.

The original success gate does not prescribe initial/approach subphases. This audit
uses a disclosed **descriptive** approach band extending one nominal horizon
(`30*.1*max_v=4.5m`) south of the corner-entry lower Y: initial below4.957836m,
approach above it before entry. After corner completion, current XY distance<the
existing2m goal radius means near-goal; otherwise hallway. After leaving the goal
region, samples return to hallway. Hallway can contain gentle steering; it is not
an angular-threshold definition. No new navigation or hard-case threshold is set.
All phase assignment uses actual FRESH **switch** position/progress, not C index.

Each cell remains **median / P95 / max**. n is the valid pair count for every shown metric.

| Run / phase | n | Position [m] | Robot-heading [deg] | Full RMSE [m] | Max lateral [m] |
|---|---:|---:|---:|---:|---:|
| A / initial travel | 5 | 0.0192 / 0.0568 / 0.0598 | 1.1772 / 100.7415 / 125.2338 | 0.5648 / 1.0395 / 1.1397 | 0.0840 / 0.2363 / 0.2736 |
| A / corner approach | 7 | 0.0234 / 0.0693 / 0.0742 | 1.3432 / 6.2577 / 6.8654 | 0.4049 / 0.4928 / 0.5062 | 0.0837 / 0.2406 / 0.2926 |
| A / left turn | 7 | 0.0201 / 0.0244 / 0.0250 | 1.1535 / 2.8616 / 2.9656 | 0.4367 / 0.5528 / 0.5941 | 0.1360 / 0.2202 / 0.2234 |
| A / hallway straight | 23 | 0.0145 / 0.0202 / 0.0485 | 3.0880 / 4.3058 / 5.6750 | 0.4137 / 0.5449 / 0.5473 | 0.0419 / 0.1200 / 0.1403 |
| A / near goal | 5 | 0.0146 / 0.0178 / 0.0187 | 1.7587 / 3.1951 / 3.3847 | 0.5140 / 0.5792 / 0.5838 | 0.0612 / 0.0829 / 0.0877 |
| B / initial travel | 5 | 0.0203 / 0.0768 / 0.0804 | 4.4132 / 103.4630 / 127.2307 | 0.5440 / 1.0601 / 1.1649 | 0.0611 / 0.1234 / 0.1348 |
| B / corner approach | 7 | 0.0256 / 0.0686 / 0.0693 | 1.4983 / 3.2576 / 3.2788 | 0.3803 / 0.4261 / 0.4281 | 0.0802 / 0.2784 / 0.3400 |
| B / left turn | 11 | 0.0188 / 0.0326 / 0.0373 | 0.7579 / 2.9953 / 3.2772 | 0.4310 / 0.5448 / 0.5619 | 0.1568 / 0.3617 / 0.4207 |
| B / hallway straight | 19 | 0.0144 / 0.0363 / 0.1243 | 3.6280 / 6.0429 / 9.0720 | 0.3935 / 0.5859 / 0.7288 | 0.0366 / 0.0704 / 0.0886 |
| B / near goal | 5 | 0.0144 / 0.0160 / 0.0164 | 1.4839 / 2.9413 / 3.1202 | 0.4966 / 0.5893 / 0.6055 | 0.0512 / 0.0644 / 0.0659 |

| Run / phase | n | Absolute delta_w [rad/s] | Obs→switch wall [s] | Pending XY [m] |
|---|---:|---:|---:|---:|
| A / initial travel | 5 | 0.0126 / 0.1071 / 0.1264 | 0.5317 / 0.5496 / 0.5505 | 0.1501 / 0.1951 / 0.2001 |
| A / corner approach | 7 | 0.0341 / 0.0680 / 0.0714 | 0.5296 / 0.5614 / 0.5627 | 0.1501 / 0.2251 / 0.2251 |
| A / left turn | 7 | 0.0360 / 0.0896 / 0.1059 | 0.5166 / 0.5340 / 0.5363 | 0.1500 / 0.1501 / 0.1501 |
| A / hallway straight | 23 | 0.0119 / 0.0384 / 0.0541 | 0.5285 / 0.5521 / 0.5528 | 0.1501 / 0.1502 / 0.2252 |
| A / near goal | 5 | 0.0195 / 0.1231 / 0.1379 | 0.5555 / 0.5627 / 0.5641 | 0.1501 / 0.1501 / 0.1501 |
| B / initial travel | 5 | 0.0375 / 0.0603 / 0.0616 | 0.5352 / 0.5516 / 0.5516 | 0.1501 / 0.2201 / 0.2251 |
| B / corner approach | 7 | 0.0125 / 0.0534 / 0.0536 | 0.5395 / 0.6468 / 0.6777 | 0.1501 / 0.2251 / 0.2251 |
| B / left turn | 11 | 0.0558 / 0.1559 / 0.1964 | 0.5327 / 0.5496 / 0.5543 | 0.1501 / 0.1625 / 0.1750 |
| B / hallway straight | 19 | 0.0159 / 0.0436 / 0.0578 | 0.5366 / 0.5748 / 0.6775 | 0.1501 / 0.2001 / 0.4254 |
| B / near goal | 5 | 0.0229 / 0.0777 / 0.0899 | 0.5338 / 0.5667 / 0.5698 | 0.1501 / 0.1501 / 0.1501 |

Both runs show more lateral revision and larger typical angular-command changes
in the turn than hallway. Turn/hallway median maximum lateral revision is
A0.1360/0.0419m and B0.1568/0.0366m; absolute delta_w is
A0.0360/0.0119 and B0.0558/0.0159rad/s. Turn position medians remain
0.0201/0.0188m; turn heading medians are actually smaller than hallway medians.
Full RMSE medians differ only modestly between turn and hallway. Thus turn revision
is clearer in lateral shape and command than in boundary discontinuity.
The different7/11 turn samples are evidence that equal C indices are not generally
physical matches. Shared selected IDs below emerged independently from each run.

## Representative observed pairs

Within each run select maxima for position and robot-heading; largest full RMSE
among both position and heading at/below their respective medians; largest lateral
revision in the turn; smallest full RMSE among that same lower-half boundary subset.
These relative median conditions choose readable examples, not hard/non-hard labels.
No composite score. Actual ranks and pool counts are saved in `selected_cases.json`.
The third criterion includes both boundary quantities; startup cannot masquerade
as a small-heading example merely because its position seam is small.

| Run | Reason | Pair | Position [m] | Heading [deg] | Full RMSE [m] | Max lateral [m] | delta_w [rad/s] |
|---|---|---|---:|---:|---:|---:|---:|
| A | largest position | C9→C10 | 0.0742 | 6.8654 | 0.3183 | 0.0821 | 0.0599 |
| A | largest robot heading | C1→C2 | 0.0091 | 125.2338 | 1.1397 | 0.0840 | -0.0014 |
| A | future with lower half boundary | C13→C14 | 0.0120 | 0.2892 | 0.5941 | 0.1824 | 0.0291 |
| A | turn lateral revision | C15→C16 | 0.0231 | 2.6188 | 0.2662 | 0.2234 | 0.0045 |
| A | small boundary and future | C25→C26 | 0.0104 | 0.6708 | 0.3593 | 0.0772 | -0.0541 |
| B | largest position | C47→C48 | 0.1243 | 9.0720 | 0.3873 | 0.0485 | 0.0043 |
| B | largest robot heading | C1→C2 | 0.0027 | 127.2307 | 1.1649 | 0.0781 | 0.0066 |
| B | future with lower half boundary | C41→C42 | 0.0124 | 2.2315 | 0.5362 | 0.0334 | -0.0080 |
| B | turn lateral revision | C15→C16 | 0.0270 | 3.2772 | 0.5619 | 0.4207 | 0.1964 |
| B | small boundary and future | C25→C26 | 0.0136 | 0.3873 | 0.2952 | 0.0327 | -0.0264 |

B **C47→C48** is the largest position residual0.124294m, with9.072deg heading,
0.387301m full RMSE, and only+0.004275rad/s command change. Observation→switch
is0.283333sim s/0.677499wall s, action-call0.240831wall s, and measured transport
0.425435m. It is a useful temporal-boundary inspection candidate. A C9→C10 is
0.074159m with0.150000sim s pending/0.225117m transport. Association with longer
pending intervals does not establish a counterfactual latency-induced failure.

B **C15→C16** has the largest turn lateral revision0.420728m and angular jump
+0.196413rad/s, but position seam0.027009m and heading3.277deg. It is useful
for studying intent preservation, not established evidence that the revision
should be corrected. A's corresponding per-run lateral maximum is0.223351m,
with a much smaller angular jump+0.004525rad/s.

The small-boundary/future examples are relative controls, not zero-error cases:
A C25→C26 still has0.359269m RMSE; B C25→C26 has0.295187m. There is no
near-zero overlap example under an externally validated noise floor, because
no such calibration or threshold exists here.

## Research judgement and limitations

- **Observed:** centimetric typical boundary position residuals, a few larger
  residuals, startup local tangent reversal, time-aligned future disagreements,
  measured pending transport and separate real command changes.
- **Compatible with ordinary replanning:** most future variation, especially
  longitudinal progress and turn lateral intent with small boundary residuals.
  Success alone does not prove each revision optimal or harmless, but this audit
  does not demonstrate a navigation cost attributable to these differences.
- **Candidates for later evaluation:** B C47→C48 temporal boundary case; B C15→C16
  turn intent-preservation case; A/B C1→C2 as explicitly startup-specific tangent/
  timing diagnostics; small-boundary controls must accompany any evaluation.
- **Not determined:** that SE(2) graph reconciliation is needed, that forcing
  agreement helps, which future prediction is correct, a robust yaw-construction
  rule, or a causal link between latency, revision and navigation quality.

Two successful no-cart runs in one scene,48 predictions each, overlapping future
windows and sequential dependency, nominal timing rather than native timestamps,
unsmoothed geometric tangents, different host/history, no obstacle intervention,
no exact ready simulator pose and no counterfactual correction evaluation limit
inference. No p-values, spatial matching, invented yaw, graph, controller tuning,
trajectory smoothing or new simulation/inference were performed.

**Next single research step (not implemented):** on these same saved runs,
measure each OLD/FRESH nominal prediction's deviation from the subsequent measured
robot path over observable horizons, retaining the current small-boundary controls.
Treat that as descriptive follow-through, not ground-truth optimality (the executed
path is itself influenced by later commands). Establish an actual execution-related
cost before selecting a reconciliation objective or implementing a graph.

## Reproduction, figures and validation

Final output: `outputs/e16-handoff-audit-20261011-04`.
Preliminary-01/-02/-03 are preserved: -02 adds longitudinal/startup diagnostics and
moves legends outside data; -03 requires both position and heading in the small-
boundary examples. Required numerical distributions do not change between these
versions. -04 also handles undefined metrics through case selection and plotting;
actual-run metric values are identical to -03. All are saved-only analyses, not simulation retries.

```bash
MPLCONFIGDIR=/tmp/e16-audit-mpl python3 research/analyze_e16_handoff_audit.py --historical-run outputs/nova-e16-hospital-lights-baseline-20261010-01 --reproduction-run outputs/nova-e16-no-cart-15-repro-20261011-01 --output-dir outputs/e16-handoff-audit-NEW
python3 -m unittest tests.test_e16_handoff_audit tests.test_chunk_geometry tests.test_route_switch -v
```

Fresh destination mandatory. Outputs:47-pair CSV/JSON for each run; full/editable
world geometry and exact transforms/clocks; dense measured path exports; summary,
phase statistics, case ranks and native tangent diagnostics. `metadata.json`
records source hashes, config/script hashes, local/upstream SHA, dirty identity,
Python/NumPy versions and command. Generated outputs are ignored and not committed.

Figures (PNG) in the final `figures/` directory:

- [Both actual paths and all C observation markers](../outputs/e16-handoff-audit-20261011-04/figures/world_paths.png).
- [Boundary distributions](../outputs/e16-handoff-audit-20261011-04/figures/boundary_distributions.png).
- [Full/editable future distributions](../outputs/e16-handoff-audit-20261011-04/figures/future_distributions.png).
- [Command and measured response distributions](../outputs/e16-handoff-audit-20261011-04/figures/command_distributions.png).
- [Boundary versus future](../outputs/e16-handoff-audit-20261011-04/figures/boundary_vs_future.png).
- [Physical phase comparison](../outputs/e16-handoff-audit-20261011-04/figures/phase_comparison.png).
- Ten `historical_pair_*.png` / `reproduction_pair_*.png` selected-pair figures,
  including [B C47→C48](../outputs/e16-handoff-audit-20261011-04/figures/reproduction_pair_47_48.png)
  and [B C15→C16](../outputs/e16-handoff-audit-20261011-04/figures/reproduction_pair_15_16.png).
  Each separates projected native30-point OLD/FRESH, derived remaining OLD and
  interpolated switch sample, actual OLD/FRESH observation poses and actual B,
  plus executed history (last3sim s in the local view) and a boundary close-up.
  Axes are equal world metres. Full executed histories are in the overview/export.

**27 focused tests PASS** (including10 new): two48/47 contracts, projection/frame
invariance, lateral shift recovery, exact switch pose/time, no extrapolation,
wrapping/degenerate null reasons, command identity/delta, progress phases and
source/destination immutability. Independent recomputation checks all94 pairs'
projection, full/editable RMSE/lateral, exact B, boundary position/heading and
commands. All700 original source files and inventory are unchanged. Figure
coordinates and visual layout reviewed. Verification receipt is in
`outputs/e16-handoff-audit-setup-20261011-01`.

Only new analyzer/plotter/config/tests/report and append-only WORK_LOG are committed.
Normal `git push origin hardcase-probe`; final SHA and push receipt are recorded in
that setup directory after commit. No existing code changes or force push.
