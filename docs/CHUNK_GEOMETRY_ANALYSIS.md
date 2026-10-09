# Successive native chunk geometry: saved crossing run

Analysis date: **2026-10-09**. Outcome: **positive evidence of aligned geometric revision, a geometry/controller trade-off, and insufficient evidence of a distinctive pedestrian-driven hard case**. Research decision: **INSUFFICIENT EVIDENCE**. No simulation, model construction, inference, sampling, scenario execution or new experimental raw-data generation was performed.

## Question and source evidence

Do consecutive predictions change their future geometry after removing observation-frame transport and matching the same nominal absolute future time?

Primary source is `outputs/crossing-20261009-03`; secondary control is `outputs/static-20261009-02`. Both contain exactly eight valid native chunks. Request IDs in this report are **1–8** (C1…C8); array indices in code are zero-based. These are not eight independent experiments: each source is one closed-loop run with seven adjacent, dependent pairs.

The analysis reads the saved arrays, per-request JSON, run metadata/summary, robot CSV, human PhysX CSV and archived image references. All eight dynamic requests pass shape `(30,2)`, floating finite values, unique IDs, `observation_body` frame, `[forward,left]` axes, meters, observation pose/time and image-presence checks. Every observation pose/time also matches the recorded robot tick. Static requests pass the same checks without requiring a human. Complete source-file SHA-256 manifests match before and after analysis, including ancillary files; source files were neither copied into a new raw dataset nor modified.

Observed simulation times in both sources are:

```text
0.0000000000000000
0.5000000260770321
1.0000000521540642
1.5000000782310963
2.0000001043081284
2.5000001303851604
3.0000001564621925
3.5000001825392246
```

Both source metadata files explicitly declare frozen physics during inference. This task characterizes successive-observation geometric revision, **not latency-induced transport/staleness**.

## Projection and nominal temporal alignment

For request k, read **only** `agent_pose_at_observation = [x_k,y_k,theta_k]`. With native point `p=[forward,left]`, project:

\[
p^W_{k,j}=t_k^{xy}+R(\theta_k)p^{R_k}_{k,j},\qquad
R(\theta)=\begin{bmatrix}\cos\theta&-\sin\theta\\\sin\theta&\cos\theta\end{bmatrix}.
\]

Here the translation vector `t_k^{xy}` is spatial; observation simulation time is separately denoted `s_k` below. World points remain 2D derived data. Ready, receive, application and next-observation poses are never projection anchors. Native arrays are never cumulatively summed, smoothed, re-anchored or augmented with an origin/yaw.

**Waypoint timing is assigned from the source-confirmed model target convention.** It is not a native tensor timestamp channel. Source evidence: [model default and 10 Hz action steps](../ticvla/models/ticvla.py), [10 Hz data grid and future offsets relative to the current frame](../data/s01_batch_json_generation.py), and [direct first-30 future-offset extraction](../ticvla/data/policy_data.py). See also [ACTION_SEMANTICS.md](ACTION_SEMANTICS.md). The assigned target offsets are +0.1…+3.0 s.

For each consecutive pair, compute the actual raw gap and integer shift:

\[
\Delta s=s_F-s_O,\quad \delta=0.1\ \mathrm{s},\quad
m=\operatorname{round}(\Delta s/\delta).
\]

All seven gaps are **0.5000000260770321 s**, giving **m=5** and residual **2.607703209e-8 s**, below the numerical alignment tolerance **1e-6 s**. No temporal interpolation of waypoints is used. The correspondence is:

```text
1-based: OLD points 6…30 ↔ FRESH points 1…25
Python:  OLD[5:30]      ↔ FRESH[0:25]
FRESH-relative nominal horizon: +0.1…+2.5 s
```

Paired absolute nominal times agree within the residual above. This is fixed nominal temporal alignment, not learned/automatic spatial correspondence. No nearest waypoint, closest-point, ICP, DTW or graph matching is used. Direct native-local subtraction is not computed or ranked; it would be **INVALID AS FRAME-INVARIANT REVISION METRIC**.

## Metrics and context

The complete chronological pair table is `pair_metrics.csv` / `pair_metrics.json`. All requested geometry, timing, controller, human and transport columns are present; `summary.json` separately ranks by **aligned_overlap_rmse_m descending**. There is no threshold, combined score or hard/non-hard label.

| Metric group | Definition |
|---|---|
| Point displacement | `d_i = ||FRESH_world_i - OLD_world_(i+m)||`; mean, RMSE, median, linearly interpolated sample 95th percentile, max, in meters |
| Signed revision | Transform aligned OLD world points with `R_F.T @ (p_OLD_world - t_F_xy)`; subtract from FRESH native points. +forward means farther forward, +lateral means left in the FRESH frame. Report mean/max absolute forward/lateral, mean signed lateral and an additional mean signed forward |
| Tangent | Segment `atan2(dy,dx)` in world coordinates; wrap FRESH minus OLD to [-pi,pi], then degrees. Mean/max are absolute; `initial_tangent_change_deg` is signed. **Derived geometric tangent; not native TIC-VLA yaw** |
| Tangent degeneracy | Segments <=1e-9 m in either curve have undefined differences, excluded from aggregates with null per-segment/initial values. This is a numerical zero-length policy, not a hard-case threshold. Every measured pair here has all 24 valid segments |
| Endpoint | `common_horizon_endpoint_shift_m = d_25`, comparing OLD point 30 with FRESH point 25, not each chunk's different-time last point |
| Controller context | OLD/FRESH v/w, delta_v, delta_w, lookahead-index change and saved filter states. Source lookahead indices retain their original zero-based convention; they are not request IDs or a correspondence algorithm |
| Robot transport | Norm of observation-to-observation planar translation; wrapped yaw difference in degrees. This is separate from geometry revision |
| Human context | Measured PhysX world position, expressed in the FRESH frame; planar forward/left and Euclidean XY distance. Distance is between reference positions, not collision clearance |

The human CSV begins at **0.01666666753590107 s**, so human position at C1/t=0 is **unavailable**. It is not extrapolated from the scripted target or copied from a later measurement. All seven FRESH observation times match measured PhysX sample times exactly (maximum offset **0 s**), consistent with the corresponding robot ticks. No interpolation was needed. The reusable time sampler supports linear interpolation only between bracketing measured samples and records sample times/weights/maximum sample offset; it never extrapolates. The first pair plot explicitly marks OLD human data unavailable.

## Dynamic results

Ranked by primary RMSE; other values describe the same 25 aligned points. Units are meters except the tangent columns in the next table.

| Rank | Pair | Mean | RMSE | Median | P95 | Max | Common-horizon endpoint |
|---:|---|---:|---:|---:|---:|---:|---:|
| 1 | **1→2** | 0.450973 | **0.457714** | 0.458307 | 0.538119 | 0.612935 | 0.458307 |
| 2 | 2→3 | 0.075383 | 0.086476 | 0.071649 | 0.135766 | 0.218387 | 0.110500 |
| 3 | 4→5 | 0.077242 | 0.085717 | 0.070273 | 0.125151 | 0.178058 | 0.104669 |
| 4 | 6→7 | 0.061792 | 0.070133 | 0.054738 | 0.106006 | 0.157852 | 0.048548 |
| 5 | 5→6 | 0.051438 | 0.059602 | 0.040300 | 0.092000 | 0.140802 | 0.031348 |
| 6 | 7→8 | 0.047534 | 0.055641 | 0.046079 | 0.082661 | 0.140401 | 0.009220 |
| 7 | 3→4 | 0.045064 | 0.053163 | 0.035638 | 0.085024 | 0.133296 | 0.044402 |

| Pair | Mean signed forward [m] | Mean/max absolute lateral [m] | Mean signed lateral [m] | Mean/max absolute tangent [deg] |
|---|---:|---|---:|---|
| 1→2 | +0.449771 | 0.023513 / 0.067625 | -0.017653 | 23.850073 / 171.097531 |
| 2→3 | +0.052014 | **0.045284 / 0.112647** | -0.044327 | 8.509969 / 28.829655 |
| 3→4 | +0.028942 | 0.018224 / 0.050675 | -0.009626 | 9.543829 / 30.943476 |
| 4→5 | +0.066529 | 0.028128 / 0.086015 | -0.024346 | 9.516964 / 33.575088 |
| 5→6 | +0.041844 | 0.016589 / 0.044038 | +0.002919 | 8.542917 / 33.805897 |
| 6→7 | +0.056059 | 0.015388 / 0.048542 | -0.003529 | 9.056616 / 32.313093 |
| 7→8 | +0.037517 | 0.015219 / 0.045478 | +0.003620 | 9.005540 / 27.660462 |

**1→2 is the largest observed revision candidate**, not a confirmed hard case. Its forward revision is +0.449771 m on average and 0.610570 m maximum absolute, dominating the 0.023513 m mean absolute lateral revision. Error is already **0.376413 m at +0.1 s**, averages 0.448573 m over +0.1…+1.0 s and 0.452574 m over +1.1…+2.5 s. Thus the first-pair change spans near and far future, rather than appearing only at the endpoint. All seven pairs reach their largest point displacement at FRESH +2.4 s; later pairs are smaller and fluctuate with waypoint index rather than growing monotonically with horizon.

The largest tangent value needs context. At FRESH +2.2→+2.3 s, OLD points 27→28 form a **0.031888 m short backward segment**, vector `[-0.031252,-0.006336]` m (heading -168.539997°). FRESH points 22→23 form a 0.140816 m forward segment, vector `[0.140675,0.006284]` m (heading +2.557533°). The wrapped difference is 171.097531°. This describes a local discrete segment reversal; it is not a 171° native yaw prediction or a reversal of the overall navigation plan. No smoothing or selective exclusion was applied. Segment lengths are retained in derived pair JSON for audit.

Auxiliary maxima do not all select the same pair:

- Largest RMSE, mean/max tangent difference and common-horizon endpoint shift: **1→2**.
- Largest mean and maximum lateral revision: **2→3**, 0.045284 / 0.112647 m.
- Largest absolute controller delta_w: **2→3**, signed **-0.015280340 rad/s**.

For 1→2, v remains **1.5 m/s**, w changes from 0.003023248 to 0.005731413 rad/s (**delta_w=+0.002708165**), and lookahead index changes 12→8. Its initial-overlap tangent difference is -21.706818°. The larger geometric revision causes only a small observed command change in this pair; this is descriptive context, not a controller performance judgment. **All seven delta_v values are zero.**

| Pair | FRESH time [s] | Human forward / left [m] | Human distance [m] | Robot transport [m] | Robot yaw change [deg] | delta_w [rad/s] |
|---|---:|---|---:|---:|---:|---:|
| 1→2 | 0.5 | +2.017076 / +2.018042 | 2.853259 | 0.469154 | -0.401939 | +0.002708 |
| 2→3 | 1.0 | +1.271196 / +1.518048 | 1.980002 | 0.749593 | +0.000480 | -0.015280 |
| 3→4 | 1.5 | +0.525251 / +1.018068 | 1.145579 | 0.749625 | -0.000835 | -0.002149 |
| 4→5 | 2.0 | -0.220711 / +0.518074 | 0.563128 | 0.749653 | -0.001190 | -0.006564 |
| 5→6 | 2.5 | -0.966635 / +0.018052 | 0.966804 | 0.749637 | -0.001823 | +0.004263 |
| 6→7 | 3.0 | -1.712509 / -0.481979 | 1.779042 | 0.749613 | -0.001287 | +0.001436 |
| 7→8 | 3.5 | -2.458334 / -0.982022 | 2.647220 | 0.749586 | -0.001214 | +0.009202 |

The largest RMSE is at FRESH t≈0.5 s, before the measured human crosses world y=0 near 2.5 s. The sampled human is closest at FRESH t≈2.0 s (0.563128 m between reference positions), where RMSE is 0.085717 m. At 2.5 s the human is already behind the robot in its FRESH frame. These are temporal relations, not evidence that the human caused any revision or that a collision occurred.

## Secondary static comparison

Identical projection, temporal alignment, metrics and tolerances were used within the saved static run. Pair RMSEs in chronological order are `0.376758, 0.058838, 0.050300, 0.052101, 0.053050, 0.055215, 0.047619` m. The complete secondary tables and derived arrays are under `static_control/`.

| Within-run statistic (7 pairs each) | Dynamic | Static |
|---|---:|---:|
| Median pair RMSE [m] | 0.070133 | 0.053050 |
| Maximum pair RMSE [m] | 0.457714 | 0.376758 |
| Median of pair mean absolute lateral revision [m] | 0.018224 | 0.026555 |
| Maximum absolute lateral revision over all aligned points [m] | 0.112647 | 0.095473 |
| Median of pair mean absolute tangent change [deg] | 9.056616 | 5.585332 |
| Maximum absolute tangent change over all segments [deg] | 171.097531 | 152.997673 |

Both runs have their largest RMSE in **1→2**. Both begin near rest (measured body forward velocity -0.0006765 m/s at C1) and reach about 1.499 m/s by C2, with lookahead 12→8. That shared startup context is a plausible contributor, not an experimentally isolated cause. The static first-pair tangent maximum likewise comes from a short backward OLD segment (0.036343 m), so large local tangent values are not unique to the human run.

Dynamic median RMSE and median tangent change are larger, but median lateral revision is **smaller**. The other six dynamic RMSEs span **0.053163–0.086476 m**, versus **0.047619–0.058838 m** for the other six static pairs. This is a descriptive comparison with background repeated replanning, not a paired intervention: trajectories, images, sampled language and internal reasoning/cache history can differ. No significance test, p-value, confidence claim or `dynamic-static = pedestrian causal effect` calculation is made.

## Figures and reproducibility

Final analyzed artifact directory: **`outputs/geometry-analysis-crossing-20261009-03-v2/`**. The first analysis output directory without `-v2` is retained; v2 moves legends away from human markers/error peaks and adds segment-length diagnostics without changing metric values.

Generated artifacts are ignored by Git:

```text
metadata.json                 # source manifests, code/config/Git identities, dependencies
pair_metrics.csv / .json       # all 7 chronological pairs and all requested columns
summary.json                  # RMSE ranking, separate auxiliary maxima, static distributions
validation.json               # independent scalar recomputation and immutability verification
derived/request_*_world.npy    # 30x2 world points; no added origin or yaw
derived/request_*_world.json   # source ID/hash, observation anchor and 3x3 transform
derived/pair_*.json            # index/time pairs, per-point error/signed revision, derived tangents/lengths
static_control/               # same tables/derived arrays for the secondary source
figures/                      # 13 figures, each PNG and vector PDF
```

Figures: `all_chunks_world`; seven `pair_01_02`…`pair_07_08`; `rmse_by_pair`; `lateral_revision_by_pair`; `tangent_revision_by_pair`; `human_relative_by_pair`; `overlap_error_vs_horizon`. World overviews use equal units; additional lateral-detail panels explicitly use unequal axis scales. Observation origins are separate markers, not invented waypoints. Human locations are measured-time matches, and executed robot segments come from `robot_state.csv`.

Run on ordinary Python with NumPy and Matplotlib; no Isaac or model dependency imports:

```bash
python3 research/analyze_chunk_geometry.py \
  --run-dir outputs/crossing-20261009-03 \
  --output-dir outputs/geometry-analysis-crossing-20261009-03-v2 \
  --static-run-dir outputs/static-20261009-02
```

Use a **new output directory** when reproducing: the retained name above now exists and must be rejected. Source-internal output paths are forbidden. Config is [chunk_geometry_analysis.json](../configs/research/chunk_geometry_analysis.json); its numeric tolerances validate alignment/degeneracy and do not classify hard cases. Existing system Python 3.12.3, NumPy 1.26.4 and Matplotlib 3.6.3 were used; nothing was installed. Matplotlib cache stays under `/tmp`.

Validation:

```bash
python3 -m unittest discover -s tests -p 'test_chunk_geometry.py' -v
```

**8 tests PASS**: known projection, exact 5-index shift, curved-world-path invariance across independently translated/rotated robot frames, known signed lateral shift, angle wrapping/zero-length segments, source immutability and no overwrite, invalid raw blocking, and measured-human time matching/no extrapolation. Synthetic test fixtures exist only in temporary test directories and are not experimental evidence. An independent scalar implementation separately verified all **14** real dynamic/static pair RMSEs to 1e-12; all source manifests match the task-start snapshot. PNG plots were visually reviewed, including the candidate pair, human context and horizon profiles.

Starting analysis SHA (local and remote verified): `026462c1999b75dfc526523ca5dccebcf9651025`, `hardcase-probe`. Upstream/main and origin/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. The raw run's metadata records its earlier starting SHA plus dirty runtime file hashes; it is not relabeled as a newly executed run. Analysis metadata records its actual execution SHA, dirty status/diff hash, both analysis module hashes, config hash, source metadata hash and every source-file hash for both runs. The final delivery SHA and remote verification are recorded after commit in the ignored analysis directory's `git-finalization.txt`; code/config hashes link the pre-commit analysis to the delivered files.

## Interpretation and research decision

**Q1 — Do chunks really change after alignment? Positive evidence.** Yes: 1→2 has 0.457714 m aligned RMSE, more than five times the next dynamic pair. This cannot be explained solely by directly comparing different robot coordinate frames because each chunk is projected from its own observation pose and matched at the same nominal future times. This is numerical revision under the model target convention, not statistical significance or a navigation-failure label.

**Q2 — Where does revision appear? Trade-off.** The largest pair changes predominantly forward progress across both near and far horizons; lateral deviations are smaller. Later pairs show centimeter-scale revision with their pointwise maxima at +2.4 s. Tangent maxima can reflect short local backward segments, not a global heading-intent reversal. Controller v is unchanged throughout, and the largest-RMSE pair has only +0.002708 rad/s delta_w.

**Q3 — Largest observed candidate?** Request **1→2** by the requested RMSE metric. Request **2→3** separately leads lateral revision and absolute delta_w. Neither is called a confirmed hard case.

**Q4 — Is this crossing scenario established as a productive next hard-case collection scenario? INSUFFICIENT EVIDENCE.** The largest revision also occurs at startup in the static control, and the dynamic median lateral revision is not larger. The saved data establish measurable revision, but do not isolate a distinctive crossing-related revision beyond ordinary startup/replanning variation. They do not justify confidently declaring the scenario too easy or guaranteeing that collecting more of it will yield qualifying hard cases. No next-stage scenario, continuous-physics loop, selection rule, correspondence study or reconciliation was implemented.

## Limitations and claims withheld

- One scenario and one run per condition; eight predictions/seven dependent pairs each, no general TIC-VLA behavior claim.
- Frozen physics during inference; **no latency-induced hard case or transport/staleness claim**.
- Nominal model target timing, not native waypoint timestamps or a guarantee the controller executes every target at that time.
- 2D position analysis; **derived geometric tangent, not native yaw**. Native BF16 values were saved losslessly as float32; discretization and short segments affect tangent interpretation.
- No automatic/spatial correspondence, correspondence validation, nearest-neighbor search, ICP or DTW.
- No reconciliation, graph optimization, hard-case threshold/score, smoothing, or reconciliation validation.
- No navigation performance, safety, collision-clearance or pedestrian-causality claim. The person is a scripted mesh in a fixed pose, as recorded in the source platform documentation.
