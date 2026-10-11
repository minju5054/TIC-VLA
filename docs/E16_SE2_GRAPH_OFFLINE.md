# E16 offline SE(2) graph reconciliation

The minimal graph produces a genuinely non-rigid result. On the **same 60 nonbootstrap pairs with valid connection directions**, Graph improves executed-to-future direction and far-FRESH preservation over Rigid in **59/60** cases. This is **conditional positive mechanism evidence**, not an overall reconciliation success: **26/94 pairs are unsupported**, six more nonbootstrap Graph outputs have a first segment shorter than 1 cm, and Graph adds relative-motion/shape distortion. Correspondence contributes only a small additional change. **INSUFFICIENT EVIDENCE** that this formulation is a generally better or executable handoff policy.

All work below is offline, on saved real-model predictions. No inference, rendering, physics, navigation or controller execution occurred. The formulation was not retuned after observing exclusions, collapsed segments or unfavorable results.

## Sources and provenance

| Run | Saved source directory | Requests / adjacent pairs | Source local Git SHA |
|---|---|---:|---|
| A | `outputs/nova-e16-hospital-lights-baseline-20261010-01` | 48 / 47 | `1013ccd872994f4e6cca1a35c4ca62b33a863cb6` |
| B | `outputs/nova-e16-no-cart-15-repro-20261011-01` | 48 / 47 | `cb18a80affa8c026b3a101ce1fa3059309fa8aac` |

These are **94 temporally dependent pairs from two successful navigation runs**, not 94 independent navigation trials. The successful original executions do not validate executing a corrected path. Source configurations match; all 96 native arrays are finite `(30,2)`. Existing loaders, exact observation/application pose checks and saved-run audit validators are reused.

- Analysis began at clean local/fetched `origin/hardcase-probe` **`7a7380baeec2756d08de771ac81b9cd651166df6`**.
- Both source runs and this analysis record upstream **`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`**. No official upstream source was changed.
- Source base-model identity: **`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`**; checkpoint SHA256: **`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`**.
- A source tracked-diff SHA256: `4f9fb42af93a8452f2914de464c4a361a070392734c89ae61ad29a9d091c0443`; B: `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855`. B had an untracked wrapper at execution: an empty tracked diff is not a claim of a clean source tree. Original dirty status, runtime versions, commands and checkpoint records are embedded in analysis `metadata.json`.
- A and B used different recorded host versions: driver 595.84 / 595.99.02 and kernel 7.0.0-34 / 7.0.0-38. No cross-run causal conclusion is drawn. The analyzer uses system Python 3.12.3, NumPy 1.26.4 and SciPy 1.11.4; no dependency installation or environment modification.
- Frozen config SHA256: **`789848bdc54e2aa8c56dccafdc909bb61fb5d22210424394b74bce8ef5f9bd71`**. `policy-freeze.json` precedes implementation/comparisons; `implementation-freeze.json` precedes selected evaluation. Both live in `outputs/e16-se2-graph-setup-20261011-01`. The latter binds code, tests, config and original evidence. Selected and full evaluation use identical frozen numerical code and weights.

The final commit SHA and normal push verification are recorded in the setup directory's `git-finalization.json`, avoiding a self-referential commit SHA inside this document.

## Frames, clocks and derived reference

Native TIC-VLA points are **cumulative offsets**, in metres, axes `[forward, left]`, origin at that request's measured observation robot pose. They are not increments and are never cumulative-summed. Each point is projected using its own observation pose:

\[
p_j^W=t_{obs}^W+R(\theta_{obs})p_j^{local},\qquad
R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix}.
\]

Here translation notation `t_obs^W` is distinct from scalar observation simulation time. SE(2) poses map body coordinates to world coordinates, are stored as `[world_x_m, world_y_m, yaw_rad]`, with `compose(a,b)` representing matrix product `T(a)T(b)` (apply b, then a). Positive yaw is counterclockwise from world +X toward +Y; angles are wrapped to the principal interval. Twists use `[v_x, v_y, omega]`. Saved SE(2) yaw is used directly; no quaternion conversion is introduced by this analysis.

Observation and actual application use simulation time. Service-ready timestamps are wall/monotonic clocks; **there is no exact simulator pose at service-ready time**, and none is invented. FRESH stays anchored at its observation even though the robot subsequently moves. `B` is the exact measured robot SE(2) at FRESH application, verified against the saved robot record.

Native arrays contain **no yaw or timestamp channel**. Nominal targets are assigned from the source-confirmed `+0.1,...,+3.0 s` model convention. Existing `boundary_seam` semantics provide a separate derived sample at nominal tau=0 equal to the observation origin, interpolate at the actual switch, and retain strictly later native target knots. This produces `F_0,...,F_M`; `F_0` is a derived switch sample, not a native waypoint. No point is extrapolated. Original native files stay separate, referenced by path/hash; derived XY/yaw and optimized arrays are separate artifacts.

For every reference node, heading is the direction of the chord in a clipped **±0.2 s** window of the remaining FRESH curve; endpoint windows are one-sided. Original XY is never smoothed or flipped. A chord must be at least **0.03 m** and have net-chord/traversed-arc coherence at least **0.25**. All reference headings must be valid for this minimal SE(2) formulation; otherwise the entire case is explicitly unsupported. Short segments and backward micro-segments relative to the chord direction are counted, not converted into native yaw measurements. This conservative all-node rule is a substantial coverage limitation.

Only measured execution at or before `B` is retained as committed history. Every pair is independent: its OLD source is the original saved preceding prediction, not a previous optimized result. No later measured robot pose is an optimization target or a future route oracle.

## Graph and baselines

The graph fixes **`X_0=B` by excluding it from optimization variables**. Editable nodes are parameterized as `X_j=F_j Exp(xi_j)`, `j>=1`. Proper SE(2) composition, inverse, stable Exp/Log and wrapped angular residuals are implemented in `research/se2_graph.py`. The solver starts at Rigid.

- **RAW:** original remaining FRESH reference, including unchanged `F_0`; no silent reanchoring.
- **RIGID:** `G=B F_0^{-1}`, `X_j=G F_j` for the whole remaining reference. All relative transforms remain unchanged.
- **GRAPH:** fixed B plus four residual families below.
- **GRAPH_NO_CORR:** exactly the same graph, initialization and weights with the correspondence family disabled.

Let `tau_j=t_j-t_switch`, `DeltaF_j=F_j^{-1}F_(j+1)` and `DeltaX_j=X_j^{-1}X_(j+1)`. The objective is one half the sum of squared **normalized, weighted** residuals. It is an optimizer objective, not a synthetic overall performance score.

| Family | Residual before scale/weight | Scale | Family lambda |
|---|---|---|---:|
| Relative FRESH motion | `Log(DeltaF_j^-1 DeltaX_j)` | XY 0.05 m, rotation 10 deg | 1 |
| FRESH intent, editable j>=1 | `Log(F_j^-1 X_j)` | XY 0.10 m, rotation 15 deg | 1 |
| Supported near OLD correspondence | `XY(X_j)-OLD_XY(t_j)` | XY 0.20 m | 0.25 |
| Boundary direction | `wrap(atan2(X_1.y-B.y,X_1.x-B.x)-psi_exec)` | 10 deg | 1 |

Degrees in this table are converted to radians in residual evaluation. Relative XY refers to the translational components of the SE(2) Log, not an unnormalized mixture of metres and radians.

Intent receives additional weight

\[
w_I(\tau)=0.1+0.9\min(\tau/1\mathrm{s},1)^2.
\]

This permits a local boundary adjustment while preserving the far FRESH future. It measures geometric retention only; semantic intent is not fully observed. There is no goal or shortest-path objective.

OLD correspondence exists only at editable shared absolute nominal times **strictly after switch and strictly before switch+0.5 s**, inside OLD's native +0.1...+3 s support. OLD is linearly interpolated at those times; no OLD yaw, nearest-neighbor match, ICP, DTW or extrapolation is used. Its additional weight is

\[
w_C(\tau)=\left(1-\tau/0.5\mathrm{s}\right)^2
\left[1+\left(\|OLD(t_j)-F_j^{XY}\|/0.3\mathrm{m}\right)^2\right]^{-1}.
\]

For `u=(X_j^XY-OLD(t_j))/0.20 m`, the residual is multiplied by
`sqrt(2/(sqrt(1+||u||^2)+1))`, a radial pseudo-Huber loss with delta=1, and by `sqrt(0.25*w_C)`. Radial robustness preserves world-rotation invariance. The confidence uses the original OLD/FRESH disagreement, not optimizer output. All eligible pairs have four or five correspondence factors. OLD remains an older prediction, not ground truth.

The execution direction is the measured XY chord over the preceding **0.1 s**, requiring displacement >=0.03 m, saved forward body speed >=0.05 m/s, and chord disagreement with measured B yaw <=30 deg. The join factor is disabled if that evidence is unreliable or the original first remaining FRESH segment is <0.01 m. Both startup pairs disable it for insufficient preceding displacement. These are fixed numerical reliability rules, not hard-case thresholds. No fallback yaw is fabricated.

The scale choices prioritize FRESH relative structure, allow softer local world displacement, strongly retain the later FRESH future, and give older uncertain geometry a weaker, local, robust influence. They were frozen before outcome inspection, with no search or per-pair changes. The formulation has **no lower bound on optimized first-segment length**; this matters in the results.

SciPy `least_squares(method='trf', jac='3-point')` uses the graph sparsity pattern, `x_scale='jac'`, `max_nfev=300`, and `ftol=xtol=gtol=1e-8`. A unit test starts a boundary+relative-only graph from a non-rigid initialization and recovers the rigid solution. Thus additional nodes alone do not demonstrate a useful non-rigid method; the extra constraints and their measured costs do.

## Metric definitions

All methods use the same nominal node times. **Zero Rigid/Graph position and pose-heading residual at B is a construction property, not a Graph advantage.** Two different RAW direction diagnostics are retained: its actual first remaining segment `F_0→F_1`, and a separately labeled hypothetical connector `B→F_1`. The latter does not alter RAW geometry. Main executed-to-future comparisons use each method's actual first segment. Rigid can have zero F_0 pose-heading error but a larger first-segment error because window-chord heading and immediate segment direction differ.

Direction/curvature metrics require segment lengths >=**0.01 m**. A shorter first segment produces **null**, never zero error. Connection error is the absolute wrapped angle between the first segment and preceding measured execution; measured B yaw comparisons are also exported. Near region is <=0.5 s after switch. Heading total variation and maximum step use valid adjacent segments; polyline curvature is wrapped turn divided by mean adjacent length. Curvature change compares only common valid turns. It is not a physical turning radius or a feasibility measure.

FRESH preservation includes relative SE(2) Log translation/rotation RMSE, segment-length and turn-angle distortion, optimized pose-heading change, independently re-estimated output chord-heading change, and pose-heading versus output-XY-heading mismatch. Optimized orientation variables need not exactly equal the tangent of optimized XY. Far displacement is world XY RMSE against original FRESH at editable times >=**1.5 s** after switch; endpoint error uses the same last nominal target. RAW's FRESH distortion is zero by definition, and Rigid's relative/shape distortion is zero to numerical precision.

## Prespecified selected cases

The five cases were evaluated and their figures inspected **before** the 94-pair batch. No favorable replacement case was selected. Angles below are executed-to-future tangent discontinuity; far values are XY RMSE relative to original FRESH. Each cell is **RAW / RIGID / GRAPH**.

| Pair | RAW position seam [m] | Connection angle [deg] | Far-FRESH RMSE [m] | Graph relative translation RMSE [m] |
|---|---:|---|---|---:|
| A C13→C14 | 0.012019 | unsupported SE(2) case | unsupported SE(2) case | — |
| B C15→C16 | 0.027009 | 3.819470 / 4.095780 / **0.253095** | 0 / 0.024183 / **0.000011101** | 0.002000 |
| B C47→C48 | 0.124294 | 9.069248 / 16.611868 / **undefined** | 0 / 0.234949 / 0.000092126 | 0.008140 |
| A C25→C26 | 0.010414 | 0.566337 / 1.358104 / **0.007298** | 0 / 0.060715 / **0.000005591** | 0.000946 |
| A C9→C10 | 0.074159 | 7.195103 / 5.216541 / **undefined** | 0 / 0.097183 / 0.000042951 | 0.004973 |

A C13→C14 has valid saved XY but only **29/30 valid derived SE(2) nodes**: node 27's clipped chord is 0.044194 m across 0.547582 m of arc, coherence **0.080708 < 0.25**. No fake heading or optimized result is supplied. Its uncorrected geometry and bad tangent window appear in the supplemental figure. The small-seam/large-disagreement overcorrection control therefore remains **unresolved under this implementation**.

B C15→C16 is a positive local turn example. Measured-robot-heading first-segment errors are RAW 3.277211°, Rigid 3.553521°, Graph 0.289164°. RAW's hypothetical B→F1 connector is 13.809247°. Graph retains the far reference while modifying local shape. A C25→C26 similarly improves this angle from Rigid 1.253688° to Graph 0.111714°; its RAW hypothetical connector is 6.517630°. Small original seams do not imply a need to correct them; the imposed directional objective is a modeling choice.

A C9→C10's Graph first segment shrinks to **0.001531 m**; B C47→C48's to **0.000009066 m**. Their direction metrics are invalid despite numerical convergence. B C47→C48 is a post-goal temporal-boundary diagnostic, **not a corner navigation failure**. Its Graph curvature-change RMSE is 2.797348 rad/m and relative translation distortion is the largest nonbootstrap value; it cannot be presented as a clean success.

## All 94 pairs: coverage and descriptive results

| Source | Accounted pairs | Eligible | Ineligible | Nonbootstrap eligible | Common valid connection comparison |
|---|---:|---:|---:|---:|---:|
| A | 47 | 31 | 16 | 30 | 29 |
| B | 47 | 37 | 10 | 36 | 31 |
| Total | **94** | **68** | **26** | **66** | **60** |

Every pair has an accounting row and four method rows, including explicit unavailable/null rows: **94 accounting rows, 376 method rows**, 272 actual method trajectories. Both Graph settings converged on all 68 eligible pairs. Exclusions are A C13→C14 and every A pair with OLD ID 33–47; B pairs with OLD ID 35–44. These are reference-heading exclusions, not invalid model tensors or optimizer failures. Across excluded node diagnostics there are 61 incoherent/backtracking chords and 58 short chords; these are node counts, not pair counts. Exact invalid node IDs/reasons are in each pair JSON.

**Same-sample medians on the identical 60 nonbootstrap pairs valid for all four connection-angle metrics:**

| Method | Executed→future angle [deg] | Far-FRESH RMSE [m] | Relative translation RMSE [m] |
|---|---:|---:|---:|
| RAW | 2.123793 | 0 | ~0 |
| RIGID | 4.712048 | 0.070606 | ~0 |
| GRAPH | **0.168288** | **0.000008505** | **0.001732** |
| GRAPH_NO_CORR | 0.166590 | 0.000009855 | 0.001678 |

The same-sample connection medians for A (n=29) are 2.069233 / 4.659240 / 0.159725° for RAW/Rigid/Graph; B (n=31) are 2.808778 / 4.927903 / 0.172190°. Graph improves connection angle over Rigid in **59/60**, worsens it in **1/60**, and improves far preservation in all 60. This two-metric result does not remove the extra deformation cost.

For all **66 eligible nonbootstrap pairs**, including the six invalid first-segment cases, preservation metrics remain reportable:

| Metric | RAW median | RIGID median | GRAPH median | GRAPH maximum |
|---|---:|---:|---:|---:|
| Boundary position [m] | 0.018095 | ~0 | 0 (fixed) | 0 (fixed) |
| Far-FRESH XY RMSE [m] | 0 | 0.076190 | 0.000008863 | 0.000092126 |
| Endpoint displacement [m] | 0 | 0.092268 | 0.000000058 | 0.000001106 |
| Relative translation RMSE [m] | ~0 | ~0 | 0.001749 | 0.008140 |
| Relative rotation RMSE [deg] | ~0 | ~0 | 0.193925 | 0.577310 |
| Segment-length distortion RMSE [m] | 0 | ~0 | 0.001215 | 0.007927 |
| Shape turn distortion RMSE [deg] | 0 | ~0 | 0.826035 | 2.428830 |
| Near heading total variation [deg] | 47.227580 | 47.227580 | 46.678245 | 83.593048 |
| Near maximum heading step [deg] | 19.821696 | 19.821696 | 19.469531 | 38.393058 |
| Near curvature-change RMSE [rad/m] | 0 | ~0 | 0.389993 | 2.797348 |

Far-FRESH Rigid/Graph medians are 0.065325/0.000008748 m for A (n=30) and 0.084540/0.000009106 m for B (n=36). The micrometre-scale numbers describe numerical closeness to a saved reference, **not prediction accuracy or physical positioning precision**. Optimized output chord-heading change has median RMS 0.290807°; optimized pose versus output chord-heading mismatch has median RMS 0.356883° and maximum 1.079698°. The graph's latent orientations and its XY tangents are not interchangeable.

### Unfavorable and startup outcomes

- Graph's first segment is <1 cm in **six nonbootstrap pairs**: A C9→C10 (1.531 mm); B C3→C4 (7.339 mm), C5→C6 (4.800 mm), C7→C8 (5.289 mm), C9→C10 (9.038 mm), C47→C48 (0.009066 mm). This loss of a stable future direction is an implementation failure mode, not improved continuity. Their other metrics and solver results remain in the dataset.
- **B C12→C13** has a valid but slightly worse executed join: Rigid **0.026087° → Graph 0.043706°**. Far preservation improves, so it is a trade-off rather than universal dominance.
- **29/66** eligible nonbootstrap Graph outputs have a larger near maximum heading step than RAW. The maximum across cases rises from RAW 31.097741° to Graph 38.393058°. Better first-segment alignment does not guarantee a smoother whole near curve; validity masks/counts are exported.
- Startup C1→C2 is retained separately in both runs. Its join factor is disabled because preceding executed displacement is too short. A's measured-heading first-segment error remains large (RAW 125.2338°, Rigid 114.7594°, Graph 106.8527°). B's Graph first segment is 9.931 mm and therefore undefined for direction evaluation. These are not primary success examples. Including startup gives **seven** short-first-segment Graph outputs.

## Correspondence ablation

Graph without correspondence already produces nearly all of the observed continuity/far-preservation effect. On the common 60 pairs its connection median is 0.166590°, compared with 0.168288° with correspondence; a difference of medians is not a paired effect estimate.

Across the 66 eligible nonbootstrap pairs, enabling correspondence changes the curve by a per-pair maximum XY displacement with median **0.001325 m**, maximum **0.002408 m**. Its paired changes are:

| GRAPH minus GRAPH_NO_CORR | Median change | Interpretation |
|---|---:|---|
| OLD correspondence XY RMSE | −0.001105 m | Slightly closer to OLD; OLD is not truth |
| Valid executed→future angle | −0.000837° | Small paired change, not essential to main result |
| Far-FRESH XY RMSE | −0.000001231 m | Very small geometric change |
| Relative translation RMSE | +0.000070 m | Additional deformation |

The largest valid connection-angle worsening from correspondence is +0.003517°. Without correspondence, five nonbootstrap first segments are invalid; enabling it adds **B C9→C10**, moving the length below the frozen 1 cm validity boundary. This is a negative trade-off. The evidence establishes an actual nonzero correspondence contribution but **does not establish its necessity, semantic benefit, or a strong OLD-specific reconciliation advantage**.

## Solver cost and residuals

Full-batch single-process CPU solve measurements include graph setup, finite-difference evaluations and result assembly, but not file loading or plotting. They are not real-time or end-to-end latency guarantees.

| Setting | Converged / eligible | Runtime median / p95 / max [ms] | nfev median / max | njev median / max | Actual residual calls median / max |
|---|---:|---|---|---|---|
| GRAPH | 68 / 68 | 20.107 / 26.423 / **687.982** | 5 / 226 | 5 / 219 | 85 / 3730 |
| GRAPH_NO_CORR | 68 / 68 | 19.087 / 22.728 / **477.798** | 5 / 171 | 5 / 160 | 85 / 2731 |

All 136 solves return status 2 (`ftol` termination); numerical failures 0. SciPy 1.11 does not expose iteration count: `iterations=null` with explanation, `nfev`, `njev` and actual callback counts are recorded rather than calling function evaluations iterations. In particular, numerical convergence does not imply a stable first segment or a physically useful result.

Median initial/final objectives: Graph **5.540516 / 0.173513**, no-correspondence **5.393969 / 0.032641**. Different residual families make these totals unsuitable as cross-ablation quality scores. Graph final family half-squared-norm medians/maxima are relative **0.025846 / 0.415118**, intent **0.006926 / 0.220634**, correspondence **0.133413 / 0.298871**, and join **0.000125 / 0.001013** (join n=66; others n=68). The 1/2 weighted squared norms are dimensionless. Full initial/final residual vectors and costs are saved per pair, not just aggregated.

## Validation and artifacts

**PASS:** 16 focused tests before real comparison, no skips. They cover matrix-oracle composition/inverse, Exp/Log, ±pi wrapping, global SE(2) frame invariance, exact fixed boundary, relative residuals, identity, rigid preservation, relative-only graph equivalence to Rigid, genuinely non-rigid response, nominal correspondence and no extrapolation, short/backward/endpoint tangents, no native yaw/time creation, convergence/failure reporting, and native/source immutability. Synthetic fixtures occur only in unit tests.

Independent verifiers using matrix and XY calculations, without importing graph/metric implementations, recomputed all **16 selected eligible method outputs** and then **272 full-batch method outputs**. They check the fixed boundary, rigid relative preservation, relative translation/rotation errors, XY/far/endpoint errors, direction validity, and curvature where supported. **700 original run files** (A 349, B 351) are hashed/inventoried before and after; the full input hash manifest has **708 files**, including code/config/receipt and the existing collider inventory. Native sources remain unchanged. Selected/full common results match exactly, excluding runtime measurements. All **125 frozen implementation files** and numerical config hashes are verified. Existing inference, simulator, controller, `continuous.py`, official DynaNav and prior experiment code are unchanged.

Five selected-case plots plus seven summary plots were generated as PNG/PDF with world XY equal aspect for path panels. Labels identify source run/pair, measured committed history, B, original FRESH observation origin, OLD prediction and the four future methods. Method comparisons share plotting scales. The ineligible A13 plot is explicitly unavailable in the main output; the supplemental review supplies its actual uncorrected geometry and invalid tangent window. All 12 pages of the combined figure book were rasterized and visually inspected for legibility, labels and clipping. Generated files are ignored and not committed.

| Artifact | Path |
|---|---|
| Policy/code freeze, test log, independent validation | `outputs/e16-se2-graph-setup-20261011-01/` |
| Selected-first evaluation and inputs | `outputs/e16-se2-graph-selected-20261011-01/` |
| Full 94-pair metrics, accounting, inputs, solver residuals | `outputs/e16-se2-graph-offline-20261011-01/` |
| Individual full-batch PNG/PDF figures | `outputs/e16-se2-graph-offline-20261011-01/figures/` |
| Common-sample statistics and all-pair limitation flags | `outputs/e16-se2-graph-review-20261011-01/` |
| Primary 12-page comparison PDF | `outputs/e16-se2-graph-review-20261011-01/comparison_figures.pdf` |
| A13 unsupported source geometry PNG/PDF | `outputs/e16-se2-graph-review-20261011-01/unsupported_selected_geometry.{png,pdf}` |

Main per-pair records are `pairs/{A|B}_Cxx_Cyy.json`; trajectories are separate per-method CSVs. `pair_accounting.csv` retains all 94 cases; `method_metrics.csv/json` retains all four methods even when unavailable. `summary.json` separates A/B/ALL and startup/nonbootstrap/all. `metadata.json` binds hashes, commands, code, source Git/checkpoints and dependency versions. Validation copies and final review receipts are stored under the full output's `validation/` directory.

Reproduction on this workspace uses saved data only and must choose fresh destinations. Example with the archived frozen receipt (never overwrite `-01`):

```bash
python3 -m unittest tests.test_se2_graph tests.test_e16_se2_graph_offline -v
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/compare_e16_se2_graph.py --phase selected --freeze outputs/e16-se2-graph-setup-20261011-01/implementation-freeze.json --output-dir outputs/e16-se2-graph-selected-NEW
# Inspect selected outputs before the full batch.
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/compare_e16_se2_graph.py --phase all --freeze outputs/e16-se2-graph-setup-20261011-01/implementation-freeze.json --selected-dir outputs/e16-se2-graph-selected-NEW --output-dir outputs/e16-se2-graph-offline-NEW
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/review_e16_se2_graph.py --analysis-dir outputs/e16-se2-graph-offline-NEW --output-dir outputs/e16-se2-graph-review-NEW
```

The archived receipt requires its exact source/code/config hashes. A changed formulation requires a new explicit freeze and a separate study; do not weaken receipt checks to reuse this evidence. The review module only reads saved results; it does not rerun optimization or modify numerical outputs. The figure-book step requires local Poppler `pdfunite`.

## Research conclusion and next single step

- **POSITIVE EVIDENCE:** this is genuine nonlinear SE(2) optimization, not renamed rigid alignment. Fixed history is preserved; 59/60 valid nonbootstrap connection comparisons improve both direction and far-FRESH retention over Rigid. Real examples B15→16 and A25→26 demonstrate the local geometric mechanism.
- **NEGATIVE EVIDENCE:** 26 unsupported pairs; six nonbootstrap collapsed first segments; one valid connection worsens; 29/66 near maximum heading steps worsen; correspondence adds deformation and one extra direction-invalid case. The prespecified A13 overcorrection control cannot be evaluated by this formulation.
- **TRADE-OFF:** local changes let the graph meet B while returning to the original far future. Rigid preserves relative geometry exactly but transports/rotates that future. Graph buys better connection/reference retention by accepting measurable relative/shape distortion; zero boundary residual is not a win by itself.
- **IMPLEMENTATION LIMITATION:** tangent-dependent all-node eligibility, no minimum first-segment length, latent pose orientations not constrained to exact XY tangents, no robot dynamics/obstacle feasibility, and single-initialization local least squares. Small cost or `success=True` does not resolve these limitations.
- **INSUFFICIENT EVIDENCE:** overall superiority, OLD correspondence necessity, semantic-intent preservation, or suitability for online control. Two runs and nominal timing cannot establish general behavior. No statistical significance claim is made.

**Answer:** different, yes; a better two-metric trade-off on an explicitly limited subset, yes; a generally better reconciliation method across actual E16 handoffs, **INSUFFICIENT EVIDENCE**.

**Next single recommended experiment, not implemented:** an offline ablation on these same 94 saved pairs that adds an explicit geometric minimum first-segment-length/progress constraint, with its policy fixed before evaluation. Compare against this unchanged baseline and count collapsed/unsupported cases plus continuity and FRESH distortion. This addresses the observed degeneracy before any closed-loop integration; it is not a delay sweep or new navigation experiment.

No collision-avoidance improvement, faster arrival, shortest path, robot feasibility improvement, navigation success-rate gain or successful closed-loop reconciliation is claimed. No future execution oracle, graph-corrected robot run, new model sample, human experiment or reconciliation runtime integration exists in this task.
