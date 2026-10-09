# Asymmetric static detour calibration

**Decision: STATIC DETOUR CALIBRATION FAILED.** One prespecified real-model static run produced 12 valid native tensors, but no post-startup valid LEFT route. The robot stopped near the offset blocker's front, entered the conservative inflated proxy and did not pass the blocker. No second navigation run or parameter search was performed.

## Motivation and scope

The preceding symmetric two-gap experiment remains **NO VALID BASE ROUTE CHOICE**; its configs, [report](ROUTE_SWITCH_HARD_CASE.md) and `outputs/route-switch-*` are unchanged. This task asks whether released TIC-VLA can generate **and execute** a valid nontrivial lateral detour in a simpler asymmetric custom scene. Infrastructure PASS is separate from detour validity.

This is STATIC DETOUR CALIBRATION ONLY. No human actor, reveal, route reversal, artificial latency, training, correspondence, rigid alignment, trajectory correction, blending, reconciliation or graph optimization was added or run.

## Prespecified geometry and execution

Config: [asymmetric_offset_detour.yaml](../configs/research/asymmetric_offset_detour.yaml). Meters, Z-up; robot/body +X forward, +Y left. World +Y is the scene's LEFT direction.

| Item | Fixed value |
|---|---|
| Scenario | `asymmetric_offset_detour`, static mode, no pedestrian config/actor |
| Robot start / yaw | `[0,0,0.15]` / 0 rad; actual settled pose separately measured |
| Goal | `[8,0,0]`; green wall of size `[0.1,2,2]` centered at `[8,0,1]` |
| Corridor half-width / inner walls | 2.5 m / y = ±2.45 m |
| Offset blocker center | `[3.8,-0.6,0.9]` |
| Offset blocker size | `[0.8,1.4,1.8]` |
| Physical XY bounds | x `[3.4,4.2]`, y `[-1.3,0.1]` |
| Conservative analysis radius | 0.4 m |
| Inflated XY bounds | x `[3.0,4.6]`, y `[-1.7,0.5]` |
| LEFT physical gap / center interval | 2.35 m / **0.5 < y < 2.05 m** |
| Decision gate | world x = 3.8 m |
| Instruction | `Move toward the green goal while avoiding obstacles.` |
| Predictions / seed | 12 / 36, fixed before inference |
| Primary window | requests **4–12**; bootstrap 1 and startup 2–3 excluded |

Jackal, mounted camera, official checkpoint/preprocessing/native semantics, waypoint controller and four-wheel mapping are unchanged. The existing continuous loop retains OLD targets while real asynchronous inference is pending, with 60 Hz simulation physics, 2 Hz target observation cadence and the same 1x pacing cap. No delay is injected. The first action's startup wait advances physics under zero bootstrap targets; subsequent requests follow the existing cadence policy. This is the custom Isaac 6 harness, not an official DynaNav reproduction or benchmark.

## Preflight and freeze receipt

Before the first model call, `research/preflight_static_detour.py` constructed the static scene, audited loaded USD collisions and saved front RGB without importing the model loop. The first preflight failed at RGB output because the wrapper passed an integer instead of a filename to `capture`; geometry audit had passed. Only that call was corrected. `outputs/static-detour-preflight-20261009-01` and its failure log remain intact. The successful second preflight used **identical geometry** and made zero model calls.

Successful evidence: `outputs/static-detour-preflight-20261009-02/` (`static_detour_geometry.json`, `robot_asset.json`, `summary.json`, `diagnostics/preflight_front.png`). The actual navigation stage independently repeats the same read-only USD audit before inference.

| Audit | Result |
|---|---|
| Meters per stage unit | 1.0 |
| Jackal collision union, body XY | x `[-0.150999997,0.230123382]`, y `[-0.285794999,0.285794999]` |
| Collision-bound circumscribed radius | 0.366927176 m, enclosed by declared 0.4 m radius |
| Static collisions | Enabled; actual USD bounds match config to float precision |
| Nominal straight centerline | Physically blocked by offset blocker |
| LEFT corridor | Feasible; physical gap 2.35 m > conservative diameter 0.8 m |
| Entire corridor accidentally blocked | No |
| Goal fully occluded | No: 7/19 horizontal goal-face ray samples unobstructed; this is a geometric proxy, not a pixel visibility percentage |
| Front RGB visual review | Wide left opening, center/right blocker and exposed green goal visible; no image edits |

The geometric ray audit uses the prescribed start/camera transform; RGB is captured at the measured settled pose with the existing no-physics-advance render barrier. No alternate scene/model sample was selected.

Before inference, the ignored receipt `outputs/static-detour-setup-20261009/prespecified-config.json` recorded the config SHA-256, preflight file hashes, visual review, starting/upstream SHA, analysis-source hashes, authorized run ID, prediction count and both success rules. Its monotonic timestamp precedes request 1's action-call start. The CLI requires this receipt and matches config hash, static/no-human mode and run ID. The analyzer verifies unchanged config bytes, saved run config, preflight hashes and receipt timing.

```text
config SHA-256:
3508a5977c787ba0bdb77be34e89557d8ac6edb74740d3465805b81922e96e4d
starting local/origin research SHA:
a7d32ef3ff76a4b9583fa8ef55605f5c276cef4a
upstream/main SHA:
9fa6f8b66b9e121d5df5df071297bba8e5353ebb
```

Live origin/upstream refs were checked and fetched before edits. No scene, camera, seed, model, controller, instruction, count or validity criterion changed after inference began.

## Frozen success criteria and analysis semantics

Prediction validity requires **at least two consecutive request IDs >=4** whose gate crossing selects LEFT and whose **entire native-point polyline** avoids the inflated blocker and corridor walls. These receive `LEFT_VALID`. A LEFT gate crossing alone cannot validate a path that intersects elsewhere. No crossings are `DOES_NOT_REACH`; differing repeated crossings are `AMBIGUOUS`; RIGHT is retained when observed; blocked gates or an otherwise invalid LEFT path are `CENTER/BLOCKED`. `gate_route_side` separately preserves the geometric gate-only label.

Execution validity requires the complete measured robot XY path to avoid the inflated blocker and corridor walls, exhibit positive-left displacement, and reach **center x > 4.6 m**. This is this scene's descriptive passing condition, not a general navigation benchmark. Sub-millimeter positive drift alone cannot validate a detour because the collision and passing requirements must also hold. No extra lateral magnitude threshold was introduced.

For request k, world points are `p_world = R(yaw_observation) @ [forward,left] + observation_xy`, using **only `agent_pose_at_observation`**. Native arrays remain `(30,2)` in meters; no origin or yaw is appended. Feasibility checks all straight segments between native points, including between-waypoint intersections. Actual feasibility checks segments between measured 60 Hz positions. Signed clearance is the minimum Euclidean distance to the **radius-inflated axis-aligned rectangle**, negative inside. This is conservative at corners and is not a measured physical penetration depth or contact-force report.

`predicted_static_obstacle_intersection` and `minimum_static_obstacle_clearance_m` here cover the **offset blocker plus corridor walls**. `predicted_terminal_goal_wall_intersection`, terminal clearance and `predicted_any_static_including_goal_intersection` also check the goal wall separately. Goal contact does not by itself invalidate obstacle avoidance. This explicitly scoped new analyzer leaves the older symmetric analyzer's semantics/results unchanged.

`predicted_max_left_m`, `predicted_max_abs_lateral_m`, `predicted_mean_lateral_m` and `lookahead_lateral_m` use **native observation-body left coordinates**. `predicted_max_world_y_m`, `gate_y`, actual Y and observation poses are world quantities; observation yaw is radians. Lookahead index retains the controller's zero-based indexing; user-facing request IDs are 1–12 throughout.

Timing and raw seam metrics are secondary. Existing `boundary_seam` temporally interpolates FRESH at actual observation-to-switch simulation elapsed time, using a separately labeled derived observation anchor at tau=0. Nominal waypoint times +0.1…+3.0 s are assigned from the source-confirmed model target convention, not a native timestamp channel. Its tangent is **derived geometric tangent; not native TIC-VLA yaw**. No spatial matching, transformation fit or reconciliation is performed.

## Run and results

Exactly one real-model navigation run: **`outputs/asymmetric-detour-static-20261009-01`**. Strict checkpoint load, 12 unique finite `(30,2)` native chunks, saved RGB/model-input JPEG, observation poses/clocks, and OLD retention during continuous inference all pass. No human files/actor exist. Model/checkpoint identities match the prior validated runtime. Infrastructure `summary.json: PASS` does **not** mean detour calibration PASS.

Observation times in seconds: `0, 1.750000, 2.250000, 2.750000, 3.250000, 3.750000, 4.250000, 4.750000, 5.250000, 5.750000, 6.250000, 6.750000`. Final measured time is 7.250000378 s; robot CSV contains initial state plus 435 physics ticks.

Final saved analysis: **`outputs/asymmetric-detour-analysis-20261009-03`**. It contains full 12-row CSV/JSON/Markdown, summary, provenance, observation-anchored world arrays/transforms, secondary seam details, and PNG/PDF figures. Earlier derived analyses remain intact: -01 initial metrics; -02 added post-contact camera containment context; -03 separates raw-evidence hash keys from code-provenance keys. These are saved-only diagnostics/fixes; primary rules/numbers did not change and no model was rerun.

Primary results below: clearances/gate/XY in meters, v in m/s, w in rad/s, yaw in radians. Intersection refers to the inflated blocker/corridor proxy; corridor/terminal goal flags are false for every request.

| Request | Route | gate_y | Blocker clearance | Intersection | v | w | Observation (x,y,yaw) |
|---|---|---:|---:|---|---:|---:|---|
| 4 | CENTER/BLOCKED | 0.165548 | -0.383723 | true | 1.5 | 0.036802 | (1.201384,-0.006846,-0.005955) |
| 5 | CENTER/BLOCKED | 0.331989 | -0.304150 | true | 1.5 | 0.251203 | (1.950954,-0.011330,-0.005906) |
| 6 | CENTER/BLOCKED | 0.036511 | -0.472715 | true | 1.5 | 0.100302 | (2.699786,-0.015498,-0.004938) |
| 7 | CENTER/BLOCKED | -0.016328 | -0.557725 | true | 1.5 | 0.046101 | (3.146773,-0.021125,0.006781) |
| 8 | CENTER/BLOCKED | 0.043939 | -0.524190 | true | 1.5 | 0.208643 | (3.147134,-0.019370,-0.002499) |
| 9 | CENTER/BLOCKED | 0.026503 | -0.525427 | true | 1.5 | 0.228064 | (3.147095,-0.020871,0.002788) |
| 10 | CENTER/BLOCKED | 0.001689 | -0.547422 | true | 1.5 | 0.169875 | (3.146729,-0.018641,-0.007483) |
| 11 | CENTER/BLOCKED | 0.036525 | -0.523860 | true | 1.5 | 0.246851 | (3.146635,-0.024301,0.008460) |
| 12 | CENTER/BLOCKED | 0.028283 | -0.523735 | true | 1.5 | 0.228613 | (3.147258,-0.022473,-0.000626) |

Excluded requests 1/2 are `DOES_NOT_REACH`, and 3 is `CENTER/BLOCKED` (gate_y .213666); all three also intersect the inflated blocker proxy. Valid primary LEFT IDs: **none**; consecutive pairs: **none**; longest valid run: **0**. All nine primary predictions intersect the inflated blocker; seven (C6–C12) also intersect the **uninflated physical rectangle**. C4/C5 avoid that point-center physical rectangle but lack footprint clearance. Thus footprint interpretation matters, but the failure is not solely a conservative corner-margin artifact.

C5 shows leftward future geometry: native max left **1.156250 m**, mean left **.464528412 m**, lookahead left **.175781250 m**, controller w **+.251202946 rad/s**. Its gate_y **.331989417 m** is below the required .5 m upper blocker edge, and its full-path clearance is **-.304149846 m**. The curve moves left too late/insufficiently near the blocker to be valid under the fixed footprint proxy. It is inaccurate to report “NO LATERAL INTENT”; the supported description is **lateral geometry exists, but no valid stable detour**. The primary paths do reach the decision region, so horizon non-reach is not the primary failure reason.

## Actual execution

| Metric | Measured value |
|---|---:|
| Final robot XY | (3.147309065, -0.024057770) m |
| Maximum world X | 3.149398565 m |
| Maximum left world Y | 0.000321588 m |
| Maximum positive-left displacement from initial Y | 0.000325510 m |
| Maximum absolute world Y | 0.025184890 m |
| Minimum inflated blocker clearance | **-0.149398565 m** |
| Entered inflated blocker proxy | **true** |
| Entered inflated corridor wall / terminal goal proxy | false / false |
| Passed blocker (center x > 4.6) | **false** |
| Final one-second XY net movement / X range | 0.000886369 / 0.000790358 m |
| Prediction / execution detour validated | **false / false** |

The robot remains near x≈3.147 m while targets remain v=1.5 m/s, with millimeter-scale late motion. This supports **ROBOT BLOCKED** as an observed progress description. No contact-force logger was added and negative proxy clearance is not claimed as measured rigid-body penetration. The tiny positive Y maximum is drift, not a successful executed lateral detour.

## RGB and implementation limitation

Raw images are preserved without modification:

- Request 1: `outputs/asymmetric-detour-static-20261009-01/diagnostics/rgb_000001.png`.
- Request 3: `.../diagnostics/rgb_000003.png`.
- First valid detour: **none**.
- Closest-approach context: request 7 `.../diagnostics/rgb_000007.png`. Actual minimum sampled clearance occurs at sim 4.050000211 s, XY (3.149398565,-.017718785); the nearest saved observation is request 7 at 4.250000222 s, time difference .200000010 s. It is not represented as an exact contact-time image. No passing request exists.

Preflight and C1/C3 show the obstacle, left opening and exposed goal. At **C7–C12**, observation-body pose plus unchanged mounted-camera translation puts the camera center **inside the physical blocker**, around x=3.407, y=-.020, z=.564 m. Saved RGB means are .132–.288 on a 0–255 channel scale, consistent with the nearly black reviewed image. Full body quaternion wxyz, not planar yaw alone, is used for this diagnostic. It was added after image review as context, without changing the camera, scene, validity criteria or raw images.

This is an **implementation limitation for interpreting post-contact policy output**: the camera mount can enter the obstacle while the robot chassis is held at its front. It does not turn the result into a valid detour and is not a reason to rerun this experiment. C4–C6 already fail footprint feasibility before camera containment; C6 also intersects the uninflated rectangle. Attribution among model behavior, embodiment/controller behavior and later degraded visibility remains unisolated.

## Secondary timing and seam context

Requests 4–12 only; measured action call is distinct from background reasoning age and observation-to-switch duration.

| Metric | Median | Maximum |
|---|---:|---:|
| Actual action-call latency, wall s | .052854307 | .140178032 |
| Observation→switch, wall s | .073772108 | .156931152 |
| Observation→switch, sim s | .066666670 | .150000008 |
| Robot observation→switch XYZ transport, m | .002243700 | .099949591 |
| Raw boundary position gap, m | .022171911 | .032293693 |
| Derived executed/FRESH tangent gap, deg | 108.988224 | 133.654965 |

Large late tangent gaps use very short measured execution segments. Maximum tangent gap is C9, with only **.001022045 m** movement in the diagnostic window; C12 uses **.000411506 m**. C4–C6 tangent gaps are .066641, 3.184648 and .105811 deg. The table exposes blocked-motion instability, not a reconciliation hard case. No latency-induced failure is established.

## Interpretation and decision

- **Negative evidence:** this single, fixed calibration failed to establish either prediction-level or execution-level valid detour. Failure reasons: **PREDICTED OBSTACLE INTERSECTION**, **NO VALID STABLE LEFT DETOUR**, **ROBOT BLOCKED / DID NOT PASS**.
- **Positive evidence, limited:** C5 contains appreciable far-future leftward geometry, and the existing infrastructure captures it and executes the unchanged controller. This is not positive evidence for valid obstacle avoidance.
- **Implementation limitation:** camera containment and nearly black post-contact observations constrain C7–C12 interpretation. No camera/geometry/controller change or additional run followed this discovery.
- **Insufficient evidence:** one run cannot separate model, embodiment, controller and visibility causes or establish general navigation behavior. No formal significance test or severity score is appropriate here.

Final decision: **STATIC DETOUR CALIBRATION FAILED**. This config is preserved as a failed prespecified calibration, **not** frozen as a validated base detour for human reveal.

Recommendation for a separate future preregistration only: consider moving the green goal toward the left free corridor (for example [8,1,0]) while retaining the obstacle and side-neutral instruction, to test whether a more direct visible goal cue elicits earlier leftward geometry. This would change the task and needs its own geometry/visibility audit; it is a hypothesis, not a promised fix. Post-contact camera containment also needs an explicit future harness review before interpreting recovery behavior. **Neither change was implemented or run.** No next-stage human experiment is authorized by this failed result.

No dynamic-intent revision, human hard-case, reconciliation, graph-optimization, correspondence validation, general navigation-performance or real-world claim is made.

## Reproduction, outputs and verification

Saved-only analysis requires NumPy, Matplotlib and PyYAML; it imports no Isaac/model runtime:

```bash
python3 research/analyze_static_detour.py \
  --run-dir outputs/asymmetric-detour-static-20261009-01 \
  --output-dir outputs/asymmetric-detour-analysis-NEW-ID \
  --freeze-receipt outputs/static-detour-setup-20261009/prespecified-config.json
```

The historical model run command, retained for provenance (do not rerun as part of this task), was:

```bash
bash scripts/isaac6_python.sh research/sim.py \
  --config configs/research/asymmetric_offset_detour.yaml \
  --mode static --run-id asymmetric-detour-static-20261009-01 \
  --prerequisite-run inference-20261009-02 \
  --detour-freeze-receipt outputs/static-detour-setup-20261009/prespecified-config.json
```

Existing output directories are rejected. The analyzer rejects output inside the source run and checks source hashes before/after. Metadata records **88 evidence/config/receipt hashes separately from analysis code hashes**, analysis script SHA-256, Git/upstream SHA, dirty status/diff hash and frozen config hash. Derived arrays record their request/observation transforms. No raw arrays or images are copied, modified or overwritten. PNG/PDF figures include equal-scale world overview, each request 4–12, actual path/clearance, separate controller-w and gate-y plots.

All **43 Isaac-independent tests PASS** (previous 34 plus nine new), including asymmetric geometry, valid/blocked/unreached/ambiguous paths, full-segment feasibility, actual clearance/passing, startup/consecutive-ID rules, terminal goal semantics, unchanged model/controller/pacing config, camera containment and end-to-end raw immutability/no overwrite/projection/provenance. No packages or runtime binaries were installed or changed.

Independent saved-data audit: world projection error <=8.89e-16 m; gate interpolation error <=1.12e-16 m; independent temporal seam error <=3.68e-16 m. Dense signed-clearance sampling agrees within 9.93e-6 m and its segment-resolution error bound. All 88 analysis evidence hashes match; all prior experiment outputs, symmetric configs/report, official sources, model service/controller/continuous loop and selected Isaac installation files match the protected snapshot. The only existing code change is the additive static-detour CLI gate and USD-audit hook in `research/sim.py`; prior mode behavior is preserved.

Generated outputs/RGB/figures are ignored and not committed. Delivery includes code, config, tests, this report and append-only WORK_LOG entry, with one focused commit and normal push to `origin/hardcase-probe`. The final SHA/remote/clean-status receipt is recorded after commit in `outputs/static-detour-setup-20261009/git-finalization.txt` and the user report, avoiding a self-referential commit hash.

## Inspect the saved run in Isaac GUI

The separate `research/view_static_detour.py` opens a visible Isaac 6 window using the saved config, native chunks and measured robot poses. It makes **zero model calls** and holds the physics timeline stopped. `C1`–`C12` selects a recorded observation (default C5), with the original saved front RGB in the viewer panel. `Replay recorded poses` places the robot at successive CSV poses; it does not rerun dynamics or animate recorded wheel rotation. `Overview` and `Robot camera` change the viewport camera.

Orange is the selected prediction projected using its own observation pose; cyan is the recorded robot path; yellow is the inflated blocker; magenta is the decision gate. Guide curves are lifted 0.025 m for display only. The neutral floor and guides are viewer aids, so the rendered viewport is a reconstruction, not the archived RGB. The panel's original RGB remains tied to the selected observation during pose replay.

```bash
bash scripts/isaac6_python.sh research/view_static_detour.py \
  --run-dir outputs/asymmetric-detour-static-20261009-01 \
  --output-dir outputs/static-detour-gui-NEW-ID
```

The output directory must be new and outside the immutable source run. It receives viewer provenance, a USD snapshot, viewport screenshot and ready marker. The validated initial GUI session is `outputs/static-detour-gui-20261009-03`; earlier viewer initialization failures are retained. They were GUI API fixes only, not experimental reruns. All 80 source-run file hashes remained unchanged after GUI launch.
