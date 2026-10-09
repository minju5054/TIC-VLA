# Route-switch elicitation and RAW handoff characterization

Date: 2026-10-09. **NO VALID BASE ROUTE CHOICE.** One prespecified two-gap static run produced 12 real TIC-VLA predictions but no valid LEFT or RIGHT route. The explicit static stop condition therefore prevented both mirrored dynamic runs. No geometry retuning, additional model sampling, artificial latency or reconciliation was performed. The failed route-choice result and a saved-only easy-control comparison are retained.

## Research question

Does FRESH choose the opposite valid route from active OLD, after nonzero OLD execution during actual action inference, and does an unmodified FRESH handoff exhibit a geometric seam at the measured switch? The strongest requested evidence is LEFT↔RIGHT reversal with valid static-obstacle geometry, actual latency transport, and a measured raw seam. This task does not establish such a case.

Starting local/remote `hardcase-probe`: `da2afafef33f576efce7a22b75fe6f5d827e05f3`. Both main refs were rechecked at `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Run metadata records that SHA plus dirty status/diff and source/config hashes. The final commit/push receipt is `outputs/route-switch-setup-20261009/git-finalization.txt`, written after delivery commit.

## Scenario geometry

The new configs are `route_switch_static.yaml`, `route_switch_block_left.yaml` and `route_switch_block_right.yaml` under `configs/research/`. They preserve the Jackal, front RGB, Isaac 6 simulation / isolated Isaac 5 model service, released checkpoint, seed 36, continuous pacing, 2 Hz observation target and unchanged lookahead controller. Instruction: **Move toward the green goal while avoiding obstacles.** No side is specified.

| Item | Prespecified value |
|---|---|
| Central blocker center XYZ | `(4.8, 0, 1.2)` m |
| Central blocker size XYZ | `(1.0, 1.6, 2.4)` m |
| Blocker XY bounds | `[4.3,5.3] × [-0.8,0.8]` m |
| Corridor inner faces | `y=±2.45` m (walls centered at ±2.5, thickness .1) |
| Left/right physical gap width | 1.65 m each |
| Goal / green wall | `(9,0,0)` m; existing 2 m wide terminal wall |
| Jackal collision-bound XY union | `[-.151,.230123] × [-.285795,.285795]` m about robot origin |
| Collision-bound circumscribed radius | .366927176 m |
| Conservative analysis footprint radius | .4 m |
| Gap center admissible y | LEFT `(1.2,2.05)`; RIGHT `(-2.05,-1.2)` m |
| Planned hidden human XYZ | `(5.8,0,0)` m |
| Planned human capsule radius | .25 m; no overlap with blocker far face x=5.3 |

Both gaps admit the .8 m conservative robot diameter with .85 m remaining width. The free corridors reconnect behind the blocker toward the goal. This is a geometry/footprint feasibility check, not a successful autonomous traversal of either gap. USD collision prims use `guide` purpose; the read-only audit includes guide purpose and instance proxies. Failed preflight API/purpose inspections and the successful bounds receipt remain under `outputs/route-switch-setup-20261009/`. No model was run during those inspections. The actual static stage repeats the audit in `route_scene_geometry.json` before inference.

`prespecified-configs.json` hashes all three scenario configs and the analysis config **before** the only model run. Those hashes remain unchanged. Existing crossing configs/results are preserved. Scene support is additive through `scene.obstacles`; old configs define none.

## Warm-up and reveal semantics

Twelve predictions were selected before execution: request 1 bootstrap, requests 2–4 route formation, request 4 the planned OLD before reveal, and requests 5–12 potential post-reveal FRESH. This provides eight post-warm-up handoffs without extending to a benchmark.

The static gate is prespecified: **requests 3 and 4 must both choose the same LEFT/RIGHT gap and both avoid every conservatively inflated static rectangle**. This establishes a stable pre-reveal choice rather than selecting a favorable request afterward. Dynamic CLI validation also requires the same scene, robot, camera, controller, inference, instruction, seed and route settings as the static prerequisite.

In the implemented triggered mode, the human exists from scene creation at `(5.8,0,0)`. Acceptance of request 4 records `raw/reveal_event.json`, its actual application simulation time, clocks and measured PhysX human pose. Triggering depends only on request ID, never predicted route content. Motion starts at that time plus the configured zero delay, moves continuously at +1 m/s toward LEFT or -1 m/s toward RIGHT, and clamps after 1.6 m of displacement. There is no spawn at reveal or teleport. Existing time-based crossing behavior is unchanged.

Both mirrors are required if the static gate passes, regardless of the first dynamic result. Here **neither was executed** because the static gate failed; this is the user's explicit prerequisite/stop condition, not selective retention of a favorable mirror. Both CLI gate checks reject before SimulationApp or output-run creation. Trigger/mirror motion is unit-tested but **not dynamically validated in this new scene**.

The initial camera-XY to hidden-human-center segment intersects the blocker rectangle. This is a **geometric occlusion proxy, not pixel-level visibility**. Per-observation proxy calculation is implemented for dynamic records, with source RGB references. Because dynamic execution was gated, there is no measured reveal/visibility sequence and no late-reveal experimental claim. Static RGB confirms the central blocker is rendered; it also occludes the green goal substantially, a limitation of this fixed scene.

## Route-intent and feasibility definitions

Each unchanged native `(30,2)` forward/left meter array is projected using only its observation pose:

\[
p^W_{k,j}=R(\theta_{obs,k})p^{R_{obs,k}}_{k,j}+[x_{obs,k},y_{obs,k}]^T.
\]

The decision gate is `x=4.8`. Intersect the **native predicted polyline**, without an added origin, with the gate and linearly interpolate `gate_y`. A unique crossing within the footprint-cleared LEFT/RIGHT interval gives that label. A crossing outside those intervals is `CENTER/BLOCKED`; no crossing is `UNKNOWN/DOES_NOT_REACH`; differing multiple crossings are `UNKNOWN/AMBIGUOUS`. There is no forced side for unknown paths. A strong reversal requires LEFT↔RIGHT, not a command-sign change alone.

Full predicted segments, not merely sampled vertices, are checked against radius-.4-expanded rectangles for the blocker, corridor walls and terminal goal wall. `minimum_static_obstacle_clearance_m` is the minimum signed Euclidean distance to these **inflated** rectangles; ≤0 flags intersection. This is conservative footprint geometry, not an exact yaw-dependent collision oracle or contact-force measurement. It can reject paths near inflated corners. Independent inspection confirms requests 3 and 4 also intersect the **uninflated** physical blocker with their centerlines, so this gate failure does not hinge on the conservative margin.

For OLD/FRESH shape revision, assign source-confirmed nominal target times `t_obs + .1,…,3.0`; these are not a native timestamp channel. Compare on the common absolute-time interval using linear temporal interpolation, sampling FRESH nominal knots plus exact overlap endpoints. No rounded index shift, spatial nearest neighbor, DTW, ICP or automatic correspondence is used. Euclidean RMSE, FRESH-frame lateral revision and wrapped derived segment-tangent differences are retained. Degenerate tangents stay undefined.

## RAW seam definition

For FRESH request k, let `B=R(t_switch,k)` be the measured state immediately after raw command application, before the next physics step, and `tau=t_switch,sim−t_obs,sim`. For this diagnostic only, prepend observation XY at tau=0 to the projected FRESH points at .1,…,3.0 s. This is a **derived temporal anchor, not a native TIC-VLA waypoint**. Linear interpolation gives `p_F(tau)`; no extrapolation outside [0,3] s is allowed.

\[
e_p^{raw}=\|B_{xy}-p_F^W(\tau)\|_2.
\]

The FRESH tangent uses the containing piecewise-linear segment (right-hand segment at an exact knot). The measured OLD tangent uses displacement over the last .1 simulation seconds before switch, clipped to the FRESH observation so the window stays within that pending interval. Measured trajectory positions at the window endpoints are temporally interpolated. Both angles require displacement >1e-6 m; otherwise they are null. The actual window displacement and FRESH segment length are saved to expose unstable small-motion angles.

\[
e_\phi^{raw}=|wrap(\phi_F(\tau)-\phi_{OLD}^{exec})|,
\qquad e_{heading}=|wrap(\phi_F(\tau)-\theta_B)|.
\]

Both are reported in degrees as **derived geometric tangent; not native TIC-VLA yaw**. The second compares actual robot yaw and remains a separate diagnostic. Very small measured motion can make the first numerically defined but physically uninformative; a large angle alone is not evidence of intent conflict.

Actual controller jumps are separately `delta_v` and `delta_w`; command sign uses a 1e-6 rad/s numerical zero tolerance. The model does not execute a waypoint queue. The geometric seam describes concatenating measured OLD history at B with observation-anchored FRESH geometry; it is not the actual controller jump. There is no re-anchor, correction, rigid alignment, blend or smoothing.

## Results

| Run | Result |
|---|---|
| `outputs/route-switch-static-20261009-01` | 12 real finite predictions; continuous infrastructure PASS; **route-choice gate FAIL** |
| `late_reveal_block_left` | **NOT RUN — static prerequisite failed**; fixed config retained |
| `late_reveal_block_right` | **NOT RUN — static prerequisite failed**; fixed config retained |
| `outputs/continuous-crossing-20261009-02` | Valid saved easy control, eight predictions; **no rerun** |

Static observation times are 0, 1.766666759, 2.266666785, …, 6.766667020 s. The long bootstrap gap reflects actual continuous inference. All subsequent gaps are about .500000026 s; the analyzer nonetheless supports arbitrary noninteger gaps, tested with .537 s.

| Request | Route proxy | Gate y [m] | Inflated static intersection | Clearance [m] |
|---:|---|---:|---|---:|
| 1 | UNKNOWN/DOES_NOT_REACH | null | false | .494850 |
| 2 | UNKNOWN/DOES_NOT_REACH | null | false | .418178 |
| 3 | UNKNOWN/DOES_NOT_REACH | null | true | -.711563 |
| 4 | CENTER/BLOCKED | -.002900 | true | -.900000 |
| 5 | UNKNOWN/DOES_NOT_REACH | null | false | 1.031507 |
| 6 | UNKNOWN/DOES_NOT_REACH | null | false | .496593 |
| 7 | UNKNOWN/DOES_NOT_REACH | null | true | -.119750 |
| 8 | CENTER/BLOCKED | -.112312 | true | -.900000 |
| 9 | CENTER/BLOCKED | -.132025 | true | -.900000 |
| 10 | CENTER/BLOCKED | -.095136 | true | -.900000 |
| 11 | UNKNOWN/AMBIGUOUS | null | true | -.900000 |
| 12 | CENTER/BLOCKED | -.249449 | true | -.900000 |

**Stable route choice: NONE.** No request chooses LEFT or RIGHT. Actual final robot XY is `(4.047273,-.034438)` at sim 7.266667 s, near the blocker front; later pending intervals move only .35–1.74 mm despite v=1.5 m/s targets. This is consistent with blocked execution, not successful gap navigation. No contact-force sensor is used.

### Largest position-seam diagnostic — not a qualifying candidate

Static C4→C5 has `CENTER/BLOCKED → UNKNOWN/DOES_NOT_REACH`, **no reversal**. FRESH has no static intersection within its finite predicted horizon, clearance 1.031507 m, but it does not reach either gap and OLD intersects the blocker. Thus it is not a valid route-switch hard case.

| Metric | Value |
|---|---:|
| Actual action-call latency | .111754785 s |
| Observation→switch wall latency | .166210318 s |
| Observation→switch simulation advance | .133333340 s |
| Actual robot transport | .199904345 m |
| RAW position seam | .076661021 m |
| Executed→FRESH tangent seam | .482426118° |
| Robot heading→FRESH tangent diagnostic | .481843586° |
| Controller delta_v / delta_w | 0 / -.072724330 rad/s |
| Aligned overlap RMSE | 1.422578382 m |
| Mean/max absolute lateral revision | .020843178 / .052289688 m |
| Mean/max overlap tangent revision | 58.904354 / 179.492334° |
| Human state | none; static run |

This is mainly a longitudinal/horizon geometry change; overlap-curve tangent extrema are distinct from the near-switch tangent seam. RGB references are `outputs/route-switch-static-20261009-01/diagnostics/rgb_000004.png` and `rgb_000005.png`; images were not edited. C3/C4 references are also plotted to expose the failed prespecified gate.

### Easy-control comparison

Identical warm-up exclusion is used for both runs: FRESH request ID >4. Thus the new static run contributes eight handoffs and saved easy control four. All non-bootstrap distributions (11 vs 7) are also retained in summary JSON. No formal significance test is performed.

| Post-warm-up metric, median / max | Two-gap static | Saved easy crossing |
|---|---:|---:|
| RAW boundary gap [m] | .014522518 / .076661021 | .032363886 / .066406182 |
| Executed→FRESH tangent seam [deg] | 7.534450 / 161.136309 | 2.728692 / 3.425586 |
| Robot heading→FRESH tangent [deg] | 97.471689 / 144.899243 | 2.725550 / 3.422747 |
| Absolute delta_v [m/s] | 0 / 0 | 0 / 0 |
| Absolute delta_w [rad/s] | .056175269 / .509416881 | .016024110 / .053381328 |

Position seams occupy a similar observed scale, with a lower static median and slightly larger maximum. Tangent/command diagnostics have larger static extrema, but these occur in invalid/blocked navigation contexts. The 161.136309° maximum at C7→C8 uses only **.001630778 m** of measured XY displacement and a .019658921 m FRESH segment; C11→C12's 116.978901° uses only .000352712 m of executed displacement. Neither proves a valid route reversal. The largest command jump is C10→C11, delta_w=-.509416881 rad/s, also with invalid FRESH geometry.

The easy run has no central decision gate (`NOT_APPLICABLE`). Its request 8 finite horizon intersects the terminal green goal-wall rectangle under this new conservative check; it remains in the descriptive background distribution and is flagged, not silently removed. This does not rewrite the earlier continuous-infrastructure PASS or make a new navigation-performance claim.

## Interpretation and decisions

- **Negative evidence:** this single fixed static scene does not establish a valid base route. No LEFT↔RIGHT reversal was observed in its 12 predictions; no dynamic intent-switch test was admissible.
- **Positive execution evidence:** actual OLD execution during inference and measurable raw seams remain present, but do not establish intent conflict.
- **Trade-off:** position-gap scale is similar to easy control; much larger tangent/command extrema occur with blocked/invalid predictions and tiny measured movement.
- **Insufficient evidence:** whether either mirrored reveal can induce a feasible intent reversal and a jointly elevated seam remains untested.

**Q1: INSUFFICIENT EVIDENCE** for the requested valid OLD/FRESH route conflict; **NO VALID BASE ROUTE CHOICE** is the decisive stop result.

**Q2: INSUFFICIENT EVIDENCE** for a qualifying intent-switch seam comparison. Easy control is available; the component-wise descriptive numbers above are not an elevated valid-candidate result.

**Combined: NO RECONCILIATION-RELEVANT HARD-CASE CANDIDATE FOUND.** Latency transport and raw geometry mismatch are measured, but valid base route choice, opposing feasible route intents, and actual mirrored reveal evidence are missing. No stronger scene tuning or baseline/graph implementation followed the failed gate.

## Reproduction, artifacts and tests

```bash
python3 -m unittest discover -s tests -v
bash scripts/isaac6_python.sh research/sim.py \
  --config configs/research/route_switch_static.yaml \
  --mode static --run-id route-switch-static-NEW \
  --prerequisite-run inference-20261009-02
# Only if route_choice_validated is true, run BOTH mirrors with fresh IDs.
# The measured run above does not satisfy that prerequisite.
python3 research/analyze_intent_handoff.py \
  --static-run outputs/route-switch-static-20261009-01 \
  --easy-control-run outputs/continuous-crossing-20261009-02 \
  --output-dir outputs/route-switch-analysis-NEW
```

For an independently authorized future run that passes the static gate, supply both `--block-left-run` and `--block-right-run` to the analyzer. It rejects one-mirror-only input and refuses to promote dynamic data following an invalid base. Missing/invalid easy evidence is reported as `EASY CONTROL UNAVAILABLE`, never recreated.

Final derived output: `outputs/route-switch-analysis-20261009-03/`: full per-handoff CSV/JSON, summary, input/code/config hashes, per-request world arrays/transforms, seam details/window lengths, and PNG/PDF world/pair/seam/control-comparison figures. Earlier `-01` and `-02` derived outputs are preserved; the final version records tangent-length context, explicit decisions and stricter source-variant validation without changing numeric metrics. All generated output is ignored by Git.

Nine new pure tests cover LEFT/RIGHT/center/unknown/ambiguous classification, arbitrary-time seam interpolation/no extrapolation, known position and wrapped tangent seams, undefined stationary tangent, .537 s temporal alignment, full-segment inflated collision/clearance and occlusion, fixed continuous reveal/mirror equivalence, prespecified static gate, independent dense clearance verification, and analysis no-overwrite/source protection. Together with existing frozen/continuous/geometry tests, **34 tests PASS**. Both actual CLI dynamic gates reject before simulator startup. Independent scalar recomputation of all 18 position seams agrees within 4.45e-16 m; all 71 analysis-source hashes, 237 protected-file hashes and pre-run config hashes remain unchanged.

## Not claimed

No graph-optimization validation, reconciliation-performance claim, correspondence validation, navigation benchmark/performance claim, real-world validation, general TIC-VLA behavior claim, pixel-level occlusion validation, or successful dynamic reveal. No new weights, installation, driver/CUDA/system Python/shell change, forced delay, latency sweep, training or source-output overwrite.
