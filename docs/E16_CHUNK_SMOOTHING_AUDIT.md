# E16 saved-only waypoint fitting and smoothed handoff audit

**Future positional disagreement persists; tangent disagreement is strongly estimator-dependent.** On all 94 pairs, full-overlap XY RMSE has median **0.426549 m RAW / 0.424013 m spline03**. The median pairwise retention is **99.63%**. In contrast, the median per-pair mean future heading difference is **39.12° with adjacent raw derivatives, 4.63° with the existing windowed chord rule, and 3.79° with spline03 derivatives**. The existing graph already uses windowed chords: the large adjacent-point heading number must not be attributed directly to its reference headings.

At the actual switch, position seams remain centimetres and typical heading seams remain a few degrees; smoothing is not consistently an improvement. The remaining same-time XY differences are predominantly longitudinal, consistent with nominal progress/timing revision plus normal replanning. This diagnostic cannot uniquely separate those contributions from changed spatial intent or establish an error needing reconciliation.

No model, simulator, graph optimization, controller or navigation was run. Existing graph code, factors, weights and outputs remain unchanged.

## Sources and frozen protocol

| Run | Exact saved source | Native chunks | Adjacent pairs |
|---|---|---:|---:|
| A | `outputs/nova-e16-hospital-lights-baseline-20261010-01` | 48 | 47 |
| B | `outputs/nova-e16-no-cart-15-repro-20261011-01` | 48 | 47 |

These are **96 actual finite `(30,2)` predictions and 94 temporally dependent pairs from TWO runs**, not independent navigation trials. All 384 fits (three splines and one polynomial per chunk) completed without fitting warnings/failures. All 94 pairs have five method rows, including RAW: **470 rows**. Invalid derivatives are retained with null heading/projection values and explicit counts; numerical fit success does not imply reliable heading everywhere.

Starting clean local/fetched `origin/hardcase-probe`: **`c0938073624db04403f35e3918101be01b6a5d42`**. Upstream: **`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`**. Source run Git/checkpoint identities remain those recorded in [the graph report](E16_SE2_GRAPH_OFFLINE.md): A `1013ccd872994f4e6cca1a35c4ca62b33a863cb6`, B `cb18a80affa8c026b3a101ce1fa3059309fa8aac`, checkpoint SHA256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`. Original dirty status/runtime/config records are hashed, not relabeled as clean experiments. Analysis runtime: Python 3.12.3, NumPy 1.26.4, SciPy 1.11.4; no installations.

`outputs/e16-chunk-smoothing-setup-20261011-01/policy-freeze.json` predates all fitting/handoff evaluation and records config plus prior-source/output hashes. `implementation-freeze.json` predates the full analysis. Config SHA256: **`e5380ddb4e00e1e663b4747e1255dc3c5a01594ae1c0ad0d158ffd22a83bfb4e`**. No settings were changed after observing results. The selected pairs are exactly A13→14, B15→16, A25→26, B47→48; previously saved Graph short-segment cases are an additional fixed diagnostic group.

## Fitting, frames and timing

Each native row is a cumulative `[forward,left]` offset in metres from its own observation robot pose. No cumsum is applied. Fitting uses the 30 original points at derived nominal times `tau=0.1,...,3.0 s`, with **no observation origin added to the fitting data**. Native tensors have no timestamp, yaw or derivative channels.

Each coordinate is fitted independently with SciPy `UnivariateSpline(k=3,w=ones(30),s=30*b^2,ext=2,check_finite=True)`. Budgets `b=0.01,0.03,0.05 m` correspond to `s=0.003,0.027,0.075 m²` **per coordinate**. They are prespecified probe budgets, not calibrated noise levels. The API uses a residual-sum-of-squares budget with a nominal 0.1% fitting tolerance; an unconstrained cubic polynomial may already satisfy the budget, leaving actual residual below it. No per-chunk budget search is performed; the library's adaptive knot selection is part of the fixed algorithm. See the [SciPy 1.11.4 API convention](https://docs.scipy.org/doc/scipy-1.11.4/reference/generated/scipy.interpolate.UnivariateSpline.html).

The diagnostic polynomial is ordinary unweighted degree-3 least squares in the same local XY and nominal time. It is not a second calibrated noise model. Spline03 is the prespecified central probe; all three strengths and the polynomial are retained, including unfavorable cases. Coordinatewise budgets are defined in physical observation-local forward/left axes: rotating the *fitting axes* could change adaptive fits. Global world-frame changes preserve the original fitting axes and correctly rotate all positions/derivatives/residual projections.

For world comparison, with column vectors,

\[
\hat p^W(\tau)=t_{obs}+R(\theta_{obs})\hat p^{obs}(\tau),\qquad
\hat p^{W\prime}(\tau)=R(\theta_{obs})\hat p^{obs\prime}(\tau).
\]

World XY is in metres, Z up; yaw is CCW from world +X toward +Y. Saved robot quaternions are wxyz, but this diagnostic uses the validated saved SE(2) yaw directly and introduces no quaternion conversion. FRESH is never reanchored at ready or switch. `B` is the exact measured FRESH application pose. Service-ready wall/monotonic time is not assigned an invented simulator pose. No later measured robot position is a fitting or comparison target; committed measured execution is displayed separately.

All evaluations reject temporal extrapolation. Within `[0,0.1)` only, a separately labeled straight bridge joins the derived observation origin to the fitted first point for boundary sampling. This is neither a spline extension nor a native waypoint. At native endpoints, fitted derivatives are one-sided; at `0.1` the fit's right derivative is used, so the bridge is not assumed differentiable across the join. RAW uses its right-hand adjacent segment, and the final knot uses the last segment. Analytic first/second derivatives are exported; RAW second derivative at knots is undefined. Derivatives have units m per **nominal** second, not measured robot velocity/acceleration.

A fitted tangent is accepted only if its derivative is finite and norm >=**0.1 m/nominal s**. This prespecified numerical floor does not certify uncertainty or eliminate directional sensitivity. No heading is substituted for invalid derivatives. Dense curve samples every 0.005 nominal seconds expose inter-knot near-stationarity, and approximate geometric arc length; they do not prove continuous-time reliability between samples.

## Residuals and geometry change

At original native times, `r=p_native-p_fit`, `T=p_fit'/||p_fit'||`, `N=[-T_y,T_x]`. Reported **forward/longitudinal** residual is `r·T`, **lateral/normal** residual is `r·N`. These are local fitted-curve directions, not necessarily the robot's original body axes. RMSE and `1.4826*median(|r-median(r)|)` use only reliable projected samples. XY residuals use all 30 points. Counts, signed per-time residuals, maximum absolute projections, maximum XY displacement, endpoint movement, arc change, segment-heading and turn-shape changes are exported per chunk.

**These are fitting residual scales or high-frequency variation proxies, not independent waypoint noise standard deviations.** They include model mismatch, real curvature/short-scale intent, nonmonotonic nominal progress, quantized/native local variation, and fitting bias. Sample points and adjacent chunks are correlated; no significance test is performed.

Across 96 chunks, cells are **median / p95 / max** in **millimetres**:

| Fit | Longitudinal RMSE | Normal RMSE | Longitudinal MAD scale | Normal MAD scale |
|---|---|---|---|---|
| Spline 1 cm | 10.094 / 10.892 / 11.713 | 9.042 / 10.524 / 11.326 | 11.139 / 13.411 / 14.279 | 9.507 / 13.263 / 14.982 |
| Spline 3 cm | 26.343 / 30.504 / 31.712 | 9.398 / 17.316 / 27.839 | 24.398 / 33.573 / 39.110 | 9.746 / 17.453 / 27.880 |
| Spline 5 cm | 26.778 / 42.548 / 51.170 | 9.329 / 17.316 / 27.839 | 25.214 / 40.074 / 54.588 | 9.801 / 17.679 / 27.880 |
| Cubic polynomial, diagnostic | 26.778 / 42.548 / 74.379 | 9.329 / 17.355 / 27.839 | 25.214 / 46.092 / 83.191 | 9.801 / 17.468 / 27.880 |

The 1 cm residual reflects a tight imposed budget and must not be read as a discovered 1 cm noise standard deviation. At 3/5 cm, **63/96 and 93/96 chunks** respectively already use a global cubic in both coordinates. Thus similar 3 cm, 5 cm and polynomial results often reflect budget saturation, not three independent confirmations.

Observation-phase bins reuse the original audit's `phase_at` rule. Straight combines `initial_travel` and `hallway_straight`; approach and near-goal are kept separately. Phases label where the observation occurred, not that all future curve points belong to that phase. At spline03:

| Observation context | Chunks | Median longitudinal RMSE | Median normal RMSE | Median longitudinal / normal MAD | Median arc change |
|---|---:|---:|---:|---:|---:|
| Straight | 56 | 24.314 mm | 8.485 mm | 23.180 / 8.887 mm | −5.052% |
| Left turn | 18 | 29.170 mm | 11.274 mm | 28.860 / 11.563 mm | −12.735% |

The other 22 chunks remain in approach/near-goal bins and in overall statistics. Per-run, per-phase and nonbootstrap distributions are in `statistics.csv`; there is no claim of a causal turn effect.

Smoothing changes geometry, not just headings. Spline03 native-to-fit XY RMSE median is **30.093 mm**, maximum waypoint displacement **95.767 mm**, and maximum endpoint displacement **44.241 mm**. Its arc change has median **−9.374%** and minimum **−30.700%**; spline05 reaches **−40.793%**, and the unconstrained cubic polynomial **−55.540%** with a maximum waypoint movement of **218.740 mm**. Spline01 can *increase* dense arc length by **12.068%**, indicating retained/interpolated oscillation. These are warnings of shape sensitivity and possible over-smoothing/oscillation, not proof that removed details were noise or intentional motion.

## Handoff comparison definitions

OLD and FRESH are independently fitted and projected with their own observation poses. Full overlap reuses the audit's FRESH native target knots plus exact common support endpoints. The new **editable overlap retains only queries strictly after actual switch+1e-8 s**, following this task. The previous audit included an interpolated switch endpoint; that old value is retained in `raw_legacy_boundary_inclusive_editable_rmse_m`. This documented endpoint-count difference must not be mistaken for a new experimental effect. Every method uses identical full/editable times; no nearest-neighbor, ICP, DTW or joint registration.

Boundary position is `||B.xy-FRESH_fit(t_switch)||`. Boundary heading is separately compared to (a) measured B yaw, and (b) a valid preceding executed-motion direction using the existing graph's 0.1 s/0.03 m/forward-speed/30° checks. The original audit's weaker numerical execution-direction check is retained as separate context. Startup execution direction is not a main comparison.

For revision decomposition, the **same RAW FRESH ±0.2 s chord tangent and left normal** is used for every method at each overlap time. It uses the existing graph's 0.03 m chord and 0.25 coherence checks, clipped to full native support. This fixed basis prevents each smoothing policy from rotating away its own lateral discrepancy. Invalid basis samples have null projections. Euclidean XY remains reportable. `body_forward`/`body_left` columns separately retain the original FRESH observation-frame basis for exact prior-audit comparability. Residual T/N and revision T/N therefore have explicitly different reference definitions.

Boundary raw chord angles are the original saved graph `F_0` heading, whose window is clipped at the remaining-future start. The supplementary full/editable OLD/FRESH chord comparison applies the same fixed rule to each original full native curve at shared queries. RAW adjacent, windowed chord and fitted derivative angles all remain derived geometric quantities, not robot yaw. Estimators use their respective explicit validity masks; per-pair mean heading differences have 94 valid pair summaries but not necessarily identical within-pair point masks.

## Boundary results

All 94 pairs, position seam median / p95 / maximum [m]:

| Method | Median | p95 | Maximum |
|---|---:|---:|---:|
| RAW | 0.016058 | 0.063851 | 0.124294 |
| Spline01 | 0.017626 | 0.074416 | 0.130444 |
| Spline03 | 0.020313 | 0.061219 | 0.128454 |
| Spline05 | 0.014861 | 0.061219 | 0.128454 |
| Polynomial | 0.016252 | 0.061219 | 0.128454 |

Spline03 reduces position seam in 37 pairs and increases it in 57; smoothing does not impose boundary compatibility. A's RAW/spline03 medians are **0.015904/0.017990 m**; B's **0.016212/0.024308 m**. Excluding both startup pairs gives overall RAW/spline03 **0.016267/0.022418 m**.

The following comparison uses the **same 91 nonbootstrap pairs** valid for all three splines (B C43→44 fails the spline01 derivative floor). Angles are median [deg]:

| Estimator | Versus measured B yaw | Versus valid executed direction |
|---|---:|---:|
| RAW adjacent | 2.2253 | 2.1481 |
| Existing RAW chord | 2.5225 | separately exported |
| Spline01 derivative | 3.6847 | 3.6459 |
| Spline03 derivative | 2.4279 | 2.3593 |
| Spline05 derivative | 2.4459 | 2.5490 |
| Polynomial derivative | 2.4279 | 2.4530 |

There is no broad median heading-seam reduction. Spline01 has a nonbootstrap maximum **177.570°** versus B yaw; a small position budget does not guarantee stable endpoint derivatives. Across the three spline strengths the boundary-heading spread median is **2.160°**, maximum **176.176°**, with missing cases retained. RAW's original startup outliers are still recorded (125.234°/127.231°); spline03/05 derivatives at those switches are unreliable, so their exclusion must not be described as smoothing those angles to zero.

## Future disagreement results

Across all 94 pairs, table entries are medians. Revision normal uses the common fixed RAW chord basis; body-left uses the FRESH observation frame.

| Method | Full XY RMSE [m] | Strict editable XY RMSE [m] | Full mean abs normal revision [m] | Full mean abs body-left revision [m] | Full mean derivative-heading difference [deg] |
|---|---:|---:|---:|---:|---:|
| RAW | 0.426549 | 0.430872 | 0.039520 | 0.022601 | 39.115 |
| Spline01 | 0.426222 | 0.430459 | 0.037852 | 0.020021 | 14.736 |
| Spline03 | 0.424013 | 0.428252 | 0.036167 | 0.017250 | 3.794 |
| Spline05 | 0.423968 | 0.428116 | 0.035979 | 0.017205 | 3.410 |
| Polynomial | 0.423968 | 0.428116 | 0.036373 | 0.017205 | 3.347 |

Existing raw chord mean-heading medians are **4.629° full / 4.600° editable**, much closer to spline03 than to adjacent derivatives. At B15→16, raw chord full disagreement is **13.624°**, versus spline03 **12.858°**. Windowing already explains much of the apparent high-frequency angular disagreement; this predates graph optimization.

Full XY median by run: A **0.436717→0.435167 m**, B **0.425018→0.422390 m** (RAW→spline03). For spline03, pairwise full-RMSE retention ranges **98.418–100.289%**, median **99.631%**. Across all spline strengths, the lowest retention is **97.978%**; none of these probes removes most of the XY discrepancy. This is descriptive scale comparison, not a significance or hard-case threshold.

Full mean absolute longitudinal revision in the fixed chord basis is **0.410029 m RAW / 0.410299 m spline03**; in FRESH body-forward axes it is **0.418429/0.418518 m**. It dominates lateral revision. The result is compatible with changes in nominal progress along a broadly similar route, while the selected turn still has a persistent lateral component. No retiming, matched spatial arc-length analysis or causal decomposition is performed, so “progress/timing revision” remains an interpretation, not an isolated measured cause.

For descriptive normalization, define each chunk's fitted residual magnitude as `sqrt(RMSE_T²+RMSE_N²)` over reliable tangent samples; require at least 24/30 reliable samples and denominator >1e-8 m. Boundary ratios use the FRESH magnitude; future ratios use the quadrature of OLD and FRESH magnitudes. This is **not a standard error or independence assumption**. At spline03, median RAW/smoothed boundary ratios are **0.585/0.805**, and RAW/smoothed full-overlap ratios **10.779/10.714**. All 94 denominators pass. Angular values are never divided by positional scales; angular uncertainty is described by estimator/budget sensitivity instead. A ratio above one alone proves neither significance nor reconciliation need.

## Prescribed real cases and graph diagnostics

Cells below are RAW → spline03; heading is geometric tangent versus measured B yaw. All four requested cases remain included.

| Case | Position seam [m] | Boundary heading [deg] | Full XY RMSE [m] | Mean abs normal revision [m] | Mean abs body-left revision [m] |
|---|---|---|---|---|---|
| A C13→14 | 0.012019→0.006194 | 0.289→3.728 | 0.594075→0.593415 | 0.138655→0.139356 | 0.075022→0.075362 |
| B C15→16 | 0.027009→0.035867 | 3.277→2.999 | 0.561907→0.562304 | 0.061156→0.063021 | 0.164682→0.164773 |
| A C25→26 | 0.010414→0.012109 | 0.671→1.408 | 0.359269→0.356109 | 0.017406→0.016317 | 0.030854→0.029096 |
| B C47→48 | 0.124294→0.128454 | 9.072→1.286 | 0.387301→0.385377 | 0.030962→0.029818 | 0.022880→0.021084 |

**A13→14:** the previously unsupported graph node 27 (FRESH tau=2.8 s) has finite derivatives for all splines. Local fitted headings there are **4.908°, 12.549°, 17.213°**, speeds **1.358, 1.134, 0.832 m/nominal s**. However, maximum cross-strength heading spread over the whole remaining reference is **179.990°**; boundary spread is **23.572°**. Every sampled node passing a speed floor does not make the heading field stable. Its persistent ~0.594 m disagreement and substantial normal revision survive; it is **not retroactively a successful Graph case**.

**B15→16:** the turn remains a meaningful geometric-revision example: ~0.562 m full RMSE and ~0.165 m body-left revision persist. Window-chord and spline03 future heading disagreement are similar (~13.6°/12.9°). Boundary derivative is sensitive to the tight 1 cm fit (13.897° versus 2.999° at 3/5 cm), whereas the original boundary chord error was only **0.276°**. Do not conflate future turn revision, raw adjacent heading, existing graph reference heading and actual controller performance.

**A25→26:** small boundary seam persists, mild lateral variation is reduced, but longitudinal future disagreement remains ~0.356 m. **B47→48:** apparent large boundary angle becomes ~1.3° across all splines, yet the ~0.128 m position seam and ~0.385 m future disagreement remain. This is after goal-region entry, not a corner failure or an avoidance case.

Additional examples were selected using the frozen descriptive rule, not substituted for the four cases: B47→48 largest spline03 heading decrease; A14→15 largest increase (**1.058°→6.831°**); A43→44 closest-to-unchanged full RMSE among upper-half RAW disagreement cases. No arbitrary threshold labels a discontinuity “gone”; reduced angle and persistent XY are reported separately.

### Prior unsupported and short-segment cases

Among the previous **26 unsupported Graph pairs**, all reference-node derivatives pass the new speed gate in **5/26 at spline01, 5/26 at spline03, 1/26 at spline05**, and **only 1/26 across all three strengths** (A13→14, itself directionally sensitive). Overall all-reference-node derivative coverage is 54/94, 71/94, 67/94 respectively. These are different derivative tests from graph chord/coherence validity, not a re-evaluation of graph eligibility. Near-stationarity/backtracking at later nominal times continues to obstruct confident heading interpretation. Dense invalid derivative samples total 1439/1073/1546 across the 96 spline01/03/05 curves.

The six nonbootstrap Graph collapse cases had original RAW first segments **16.767–22.969 mm**, all above the previous 10 mm evaluation floor. Independent spline03 segments at exactly the same switch/next-target times remain **14.073–55.218 mm**. Saved Graph segments are **0.009066–9.038 mm**. Thus those collapsed output segments are not already-collapsed native input segments; they arise in the prior optimization's fixed-boundary compromise. This does not prove input variation has no effect on that optimization. B startup is distinct: RAW 10.222 mm, spline03 6.645 mm, saved Graph 9.931 mm; startup remains separate. Per-case lengths are in `descriptive-comparisons.json` and the figure.

On the **same 60 cases** used in the prior valid connection comparison, sample exactly the original switch and next future node, apply the same 10 mm length check, and compare to preceding execution. Median first-segment errors [deg]:

| Uncorrected RAW | Independent spline01 | Independent spline03 | Independent spline05 | Saved RIGID | Saved GRAPH |
|---:|---:|---:|---:|---:|---:|
| 2.123793 | 2.018891 | 2.003509 | 2.003509 | 4.712048 | 0.168288 |

This input-only smoothing diagnostic does not reproduce most of the previously measured Graph connection reduction. Graph explicitly optimizes a boundary-direction objective and changes geometry; the spline is not constrained to B. It would be wrong to infer Graph necessity or overall superiority from this unequal-objective comparison. Prior shape distortion, unsupported coverage, first-segment collapse and weak correspondence contribution remain unresolved; nothing in this task changes those results.

## Interpretation and next single task

- **A. PERSISTENT GEOMETRIC DISAGREEMENT:** same-nominal-time XY separation and the selected turn's lateral revision survive all frozen fits. “Disagreement” is descriptive; it may be desirable replanning.
- **B. SMOOTHING-SENSITIVE DISAGREEMENT:** adjacent derivative angles and some boundary angles change substantially. Existing windowed chords already suppress much of the future angular variation; tight spline fitting can introduce/retain poor derivative behavior.
- **C. SMALL BOUNDARY / LARGE FUTURE REVISION:** typical ~1.6–2.0 cm position seams coexist with ~0.42 m future RMSE. They are different questions and scales, not interchangeable hard-case evidence.
- **D. UNRELIABLE HEADING OR FIT:** zero numerical fitting failures, but numerous near-stationary derivatives, budget-sensitive directions and substantial shape changes. A13 derivative availability is not heading stability; prior 26 exclusions are not simply solved.
- **E. INSUFFICIENT EVIDENCE:** true independent waypoint noise, a unique progress-versus-spatial-change decomposition, measurement uncertainty magnitude, semantic-intent preservation, and reconciliation necessity are unestablished. Exact saved ticks verify data association, not a zero-error physical sensor model.

The evidence supports persistent **nominal-time positional revision with a strong longitudinal component**, alongside local high-frequency angular variation. It does not show that smoothing improves navigation, that all disagreement is error, or that Graph is either unnecessary or necessary.

**Next single recommended task, not implemented:** a controlled saved-only ablation of the **unchanged graph factors/weights** using original reference geometry versus independently spline03-fitted reference geometry. Retain the same 94 cases, derivative-invalid cases, fixed B and original-native distortion metrics. This directly tests preprocessing's contribution to graph coverage, collapse and continuity without changing the graph objective or claiming an executable controller improvement.

## Validation, artifacts and reproduction

**13 focused tests PASS**, no skips. They cover exact 96 native files/hashes, nominal times/budgets, observation-world transforms, independent fits, tangent/normal residual projection, global frame consistency, exact straight-line and known smooth-turn fixtures/endpoint derivatives, no reanchoring, invalid derivatives, no extrapolation, original-audit comparison, and raw immutability. The first test run failed only on exact equality between `30*s*s` and `30*s**2` at binary floating-point precision; the assertion was corrected to 15 decimal places without changing fitting code or policy. Both test logs are preserved. Synthetic fixtures are tests only.

An independent verifier reconstructs splines from saved knots/coefficients using `BSpline` and reconstructs polynomial coefficients, without calling fitting or graph code. **384 fitted chunk outputs and 470 pair-method outputs** pass residual, derivative, budget, native/world, boundary, overlap and projection checks. The original RAW full RMSE/body-left metrics and position seams match the prior audit. **700 original run files, 1156 protected source/prior-output files and 123 pre-existing code files** remain hash-identical; original run and prior-output inventories are checked. New analysis modules do not modify `se2_graph.py`, its config, inference, controller, simulator, DynaNav or earlier experiment sources.

| Deliverable | Exact directory/file |
|---|---|
| Frozen policy, implementation hashes, test/process logs | `outputs/e16-chunk-smoothing-setup-20261011-01/` |
| Primary numeric analysis | `outputs/e16-chunk-smoothing-audit-20261011-01/` |
| Chunk residual CSV/JSON | primary `chunk_metrics.csv/json` |
| All 94 x 5 method handoffs | primary `pair_metrics.csv/json` |
| Strength sensitivity and run/phase distributions | primary `smoothing_sensitivity.csv/json`, `statistics.csv`, `summary.json` |
| Plot coordinates, original/native references, derivatives | primary `chunks/{A|B}_Cxx.json`, `pairs/{A|B}_Cxx_Cyy.json` |
| Final comparison PNG/PDFs | `outputs/e16-chunk-smoothing-review-20261011-02/figures/` |
| Primary 15-page figure book | `outputs/e16-chunk-smoothing-review-20261011-02/comparison_figures.pdf` |
| Independent validation, common-case comparisons, raw chord comparison | final review `independent-validation.json`, `descriptive-comparisons.json`, `future_chord_comparison.json` |

Original numeric output and the first review are preserved. The final review changes figure layout and adds the requested future-window chord comparison from saved coordinates; **zero refits and zero graph solves**. Its earlier review script version is archived in setup for hash provenance. All 15 final pages were rasterized and reviewed; the 14 previously inspected figures are byte-identical PNGs and the additional chord comparison is separately inspected. A broad 0–180° view retains outliers; a labeled 0–10° detail makes typical boundary angles readable. Path panels use equal world/local XY aspect and distinguish measured committed execution, original predictions and fitted curves. Outputs/figures remain ignored by Git.

Reproduce with fresh output names; existing destinations are rejected:

```bash
python3 -m unittest tests.test_e16_chunk_smoothing -v
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/analyze_e16_chunk_smoothing.py --freeze outputs/e16-chunk-smoothing-setup-20261011-01/policy-freeze.json --output-dir outputs/e16-chunk-smoothing-audit-NEW
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/review_e16_chunk_smoothing.py --analysis-dir outputs/e16-chunk-smoothing-audit-NEW --output-dir outputs/e16-chunk-smoothing-review-NEW
```

The archived receipt requires the unchanged source runs, prior graph evidence, code and fitting config. A changed fitting policy needs a new freeze and separate study. Final source/inventory/figure verification and normal commit/push receipt are in setup `final-verification.json` and `git-finalization.json`. No generated data or figures are committed.
