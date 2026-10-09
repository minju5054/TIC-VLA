# Simulated Nova Carter execution validation — Hospital E16

Date: 2026-10-10 (Asia/Seoul). Scope: **NOVA CARTER EXECUTION VALIDATION ONLY**. All robots here are simulated in Isaac; no physical robot is involved.

## Motivation and source identity

The preserved Jackal Hospital evidence (`outputs/hospital-e16-static-20261009-01`) has early positive-left predictions and angular targets, but almost straight actual motion. Its historical report remains unchanged. This task asks first whether the **same saved target sequence** produces measured yaw with Nova Carter, then whether Nova executes its **own closed-loop TIC-VLA predictions** in the same official episode context. It does not assume Jackal is the only cause.

Starting local/origin `hardcase-probe`: `d9385f28d5d106641cf7a3acefb92258b0334bec`. Freshly fetched and live-checked upstream `main`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. `DynaNav/benchmark.py`, `DynaNav/configs/benchmark_full.yaml` and `DynaNav/behavior/nova_carter_test_ticvla.py` match that upstream byte-for-byte. No official source was changed.

Exact episode16 uses the original Hospital URL `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Hospital/hospital.usd`, start `[7.38,1.49,.01]`, yaw90° via USD RotateZ, goal `[-12.23,10.19,.01]`, people0 and official timeout85s. Research stores yawπ/2 rad, forward+Y world, left−X world. The runner adds .1m to Nova spawn Z: applied Z=.11m. The exact instruction is:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

Goal coordinates are diagnostics/provenance, not a new model input. There is no custom geometry/light, human, recovery, planner or dynamic actor.

## Nova source and loaded-asset audit

| Property | Source / loaded result |
|---|---|
| Asset (from official runner) | `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Carter/nova_carter_sensors.usd` |
| Root | `/World/Robots/Nova_Carter` |
| Chassis | `/World/Robots/Nova_Carter/chassis_link` |
| Drive joints, controller order | `joint_wheel_left`, `joint_wheel_right` |
| Source controller defaults | radius .152m, base .413m |
| Loaded analytical drive collision radius | .139999996871m (configured .14m) |
| Loaded drive-wheel center separation | .4132m |
| Applied DifferentialController geometry | radius .14m, base .4132m |
| Front camera | `/World/Robots/Nova_Carter/chassis_link/front_hawk/left/camera_left` |
| Capture | Actual existing camera, 1920×1080, original transforms/intrinsics/projection retained |
| Stage / scene | Metres, Z-up, original Hospital identity root |

The source defaults and actual asset geometry differ. This was discovered and resolved **before model calls**, not tuned from navigation results. Drive collision cylinders have radius .14/.125/.10m; the outer cylinder determines the drive radius. Their transforms have unit scale and centers at bodyY±.2066m, bodyZ.14m. Joint anchorY±.1726m alone omits the .034m body1 offset and must not be mistaken for track width. Both joint `axis=Z` values, rotated by localRot0, point along body+Y; positive w therefore makes left wheel slower/right wheel faster. Authored Cylinder `extent` values are stale ±50 and are not suitable geometry measurements; analytic radius/height and transforms were inspected instead. No USD correction was authored.

Model-free discovery outputs `nova-asset-audit-20261010-01` and `-02` are retained; the second adds transforms/scales to resolve this geometry discrepancy. Top-level asset10374bytes SHA256`7e73f6720ce7fc124e0955ee6e3c270aeab1d57fba2e0f1a3098f16edc3c30c8`; referenced assets/textures are not recursively checksummed. The source behavior SHA256 is `6411d53e30fc7e2f376052bf4818d340dd22e9463e0e0541d562540b722f04d0`.

Installed Isaac6 `isaacsim.robot.wheeled_robots.controllers.DifferentialController` is used directly, with `[v,w]` input and `[left,right]` rad/s targets: `(v−wb/2)/r`, `(v+wb/2)/r`. Its actual path and SHA are stored in each `simulation-runtime.json`. The adapter uses float64 target arrays, retains separate target/applied columns and changes no installed controller code. There is no Jackal four-wheel mapping in the Nova adapter.

The optional replay slew is an exact arithmetic copy of source `_slew`, tested against an AST-extracted source oracle: linear accel/decel2.0/2.5, angular3.0/3.5, deadbands.001/.0005. Accel/decel selection depends on the sign of `target−current`, exactly as upstream, not a redesigned magnitude-based limiter. Slew advances once per physics tick, never merely on command acceptance. Backup/recovery/waiting policies are not imported.

## Additive integration and logging

`research/robots/nova_carter.py` exposes state/apply_target/wheel_state/stop. `NovaSimulation` reuses existing synchronized capture and physics-step semantics but has its own initialization, two-wheel controller and explicit CSV schema. The original Jackal `sim.py`, four-wheel mapping/configs and tests are unchanged. `continuous.py` gains only an optional saved-evidence validator argument; its observation scheduling, one outstanding request, OLD retention, pacing and command extraction are unchanged. Hospital scene metadata selects an explicit Nova spawn label without changing default Jackal output.

Each tick records target/applied v/w, two wheel targets, two measured wheel velocities, measured world pose/quaternion (wxyz), body forward/left/up velocity, body angularZ, pose-derived yaw rate, simulation time, active/pending request IDs and replay source tick. Initial pose-rate is undefined; later pose rate is wrapped measured yaw increment divided by actual tick dt. `response_metrics` unwraps yaw over the complete saved trace. No native TIC-VLA yaw channel is invented.

In StageB direct mode, target=applied v/w. Requests retain original native30×2 chunks and controller/lookahead/state/timing data, plus explicit target/applied fields and two-wheel order. No fake FL/RL/FR/RR fields are used. OLD target retention is independently checked on every actual pending tick with the two-wheel schema.

## Stage A0: model-free sanity

Run: `outputs/nova-a0-20261010-01`. Fresh official scene/start, stationary2s, v=.4/w=0 for2s, stop1s, then v=0/w=+.6 for2s. The .2m forward and .1rad positive-turn minimums are inherited sanity criteria, fixed before results.

Spawn .11m settles to root pose approximately `(7.380001068,1.492211819,−.000000566)`, yaw1.570786292rad. This asset's root/chassis reference is near ground level; it is not Jackal's body-center height. XY drift .002211819m, both drive wheels supported, no initial non-floor environment contact, and no camera-proximity hit. No drive/caster gains, friction or asset geometry were retuned.

| Check | Result |
|---|---:|
| Stationary maximum yaw excursion | 4.284621585e−6rad |
| Straight actual forward displacement | .779652245m |
| Positive-turn applied yaw integral | 1.200000063rad |
| Actual positive yaw change | 1.162258752rad =66.592521208° |
| Actual/integral response ratio | .968548910 |
| Physics-vs-pose yaw-rate RMSE during turn | .003782258rad/s |
| Non-floor contacts during A0 | None |

**A0 PASS.** No model calls. Unmodified [initial front Hawk RGB](../outputs/nova-a0-20261010-01/diagnostics/a0_initial.png) was visually reviewed: centered staircase, corridor, floor and furnishings visible. Isaac6 reports the old asset's fisheye projection as deprecated but renders it; no replacement camera or projection adaptation was introduced.

## Stage A1: exact saved command replay

Read-only source: `outputs/hospital-e16-static-20261009-01`. CSV SHA256 **`e72467f21c504398cda9106c2633c567cfaaa0c0a748ef1121ecacbaf1741a2b`**. The source is FAIL/INCOMPLETE overall but the selected clean C4–C7 prefix is intact.

- Initial primary pose boundary: C4 application at tick172, sim2.866666816s.
- Applied primary rows: **ticks173–288 inclusive**, 116 steps, ending at sim4.800000250s.
- Duration1.933333434s. Active command IDs are exactly4,5,6,7.
- CSV row n records the command applied on `(t[n−1],t[n]]`. Row288 finishes before C8 observation/inference, so it belongs to the requested interval.
- To avoid an arbitrary jump from rest at C4, both Nova variants first replay the exact saved ticks1–172 (bootstrap zeros/C1–C3). The primary comparison still uses only C4–C7. This pre-roll policy was recorded before replay/model results.

Every target is copied from its saved physics row, not reconstructed from request-level constants. Both Nova runs start at official episode XY/yaw/rest and use identical target history. Their state histories can diverge due to embodiment/actuation, which is part of the comparison. No Jackal rerun or TIC-VLA call occurs in StageA.

| Variant | ∫target w (rad) | ∫applied w (rad) | Actual Δyaw (rad / deg) | G_target / G_applied |
|---|---:|---:|---|---|
| Saved Jackal | .254607021 | .254607021 | .001386383 / .079433879° | .005445187 / .005445187 |
| Nova direct | .254607021 | .254607021 | .270911684 /15.522096094° |1.064038544 /1.064038544 |
| Nova official-slew | .254607021 | .254607021 | .270463991 /15.496445198° |1.062280177 /1.062280177 |

Run IDs: `nova-replay-direct-20261010-01`, `nova-replay-slew-20261010-01`. Saved-only comparison: `outputs/nova-response-analysis-20261010-01`.

| Tracking/context metric | Jackal saved | Nova direct | Nova slew |
|---|---:|---:|---:|
| Mean target/applied w (rad/s) | .131693280 | .131693280 | .131693280 |
| Mean measured body yaw rate (rad/s) | −.020592644 | .143439468 | .143135444 |
| Peak absolute measured yaw rate (rad/s) | .034272360 | .182408860 | .181194399 |
| Physics yaw-rate RMSE vs applied (rad/s) | .156497744 | .012438768 | .012748878 |
| Pose-rate RMSE vs applied (rad/s) | .133936367 | .009353184 | .009381629 |
| Physics/pose rate disagreement RMSE (rad/s) | .022906261 | .005274197 | .005698107 |
| Physics sign agreement | 0% |100% |100% |
| Pose-derivative sign agreement |100% |100% |100% |
| Actual path length (m) |2.898463385 |2.901538215 |2.901553552 |
| Interval-initial body forward/left (m) |2.898463176 / .000631131 |2.870924426 / .350724462 |2.871094134 / .349487345 |
| Primary non-floor contact spans |0 |0 |0 |

The Jackal pose changes are tiny positive while its saved physics angular-velocity signal is negative: both are reported, without relabeling either or rewriting the earlier run. The primary Δyaw/response ratios use the requested **unwrapped measured pose**. Nova shows much closer agreement between its two measurements. Rate tracking is descriptive; no ideal yaw-source claim is made.

Slew does not bind the small angular target changes during C4–C7, so target and applied integrals are equal there. It does affect startup/pre-roll acceleration; the close direct/slew responses do not show that slew can never matter. Mean absolute target/applied w equals mean signed w here because all primary targets are positive. Ratios are undefined for near-zero integrals.

The preregistered gate requires A0 success, both replay Δyaw values above10×stationary excursion+1e−6rad (4.384621585e−5rad), majority positive pose-rate agreement and no primary non-floor contact/integration failure. **NOVA CARTER ANGULAR RESPONSE VALIDATED.** This is strong response evidence, not perfect tracking.

## Stage B preregistration

StageA success and initial front-RGB visual review gate the one primary real-model run. Config SHA256 **`ca57c06f49977a9191273db335bfb2e29c1cfb57f45a6e07945914e52f1148eb`**; pre-model freeze receipt SHA256 **`278338bb6580fd12e61c4882b26ea578cede9a355fe6831808b65143d7a7b542`**, at `outputs/nova-e16-setup-20261010/stage-b-freeze.json`. Config, StageA decision/source identity and recursive research-code hashes were frozen after65 pure tests passed.

Fixed48 predictions, unchanged2Hz target cadence/continuous physics/one outstanding request/OLD retention, original checkpoint/preprocessing/`robot_type="wheeled robot"`, exact instruction and same `waypoint_command` 1m lookahead. **Direct Nova only**, with no slew/recovery/backup. The original camera height/optics/resolution differ from Jackal, explicitly; request IDs across their closed loops do not represent identical model inputs.

Prediction geometry reuses the existing Hospital analyzer's four-indicator, two-consecutive-request rule, excluding C1–C3. Tangents are derived segment geometry, not native yaw. Actual turn requires two consecutive contact-free, clean-visibility intervals after stable prediction with positive measured yaw and episode-left-dominant displacement. StageB asks conversion of positive predictions into actual left motion and does not require an earlier forward-only prediction label.

Before this run, degraded-visibility evidence exclusion was set to channel p95≤20/255, explicitly stricter than the previous report's inadequate p95≤2 cutoff. The original capture guard std<1 is **unchanged**. A rejected same-buffer image is archived separately and never submitted to the model; no second render or guard relaxation is used. Contact/camera proximity logging continues. Changing this descriptive image-quality exclusion does not change the policy input/control path.

## Stage B: one real-model closed-loop run

Run **`outputs/nova-e16-closed-loop-20261010-01`** completed with **48 unique finite native(30,2) predictions**, strict released-checkpoint loading, 1520 robot-state rows and final sim25.316667987s. Observations are C1=0, C2=1.816666761s, then approximately .5s apart through C48=24.816667961s. The initial bootstrap overrun is retained; 2Hz is the target cadence, not a claim of .5s first gap. There were262 physics ticks wholly inside action calls, including157 after bootstrap, with OLD targets preserved on every pending tick. No second model run or result-driven adjustment occurred.

Saved-only analysis: **`outputs/nova-e16-analysis-20261010-01`**. StageB prediction geometry is observation-body forward/left, metres. World points are derived with each request's own observation XY/yaw, never application pose. All48 request responses and original arrays remain available; native arrays were not augmented with origin/yaw/time channels.

The first eligible stable positive-left group is **C4–C5** (C4obs2.816666814s, confirmed C5). Another stable group is **C14–C16**, with much larger lookahead lateral offsets. No eligible forward-only prediction precedes C4, so this does not establish a sharp forward-to-left intent switch at C4. Actual forward motion does precede the later substantial turn.

| Request | Endpoint left (m) | Mean left (m) | Lookahead left (m) | Bearing (deg) | Target w (rad/s) | Derived total tangent change (deg) |
|---|---:|---:|---:|---:|---:|---:|
| C4 | .211914 | .071767 | .031250 | 1.591140 | .099716 | 35.011131 |
| C5 | .184570 | .067633 | .039063 | 1.961418 | .095329 | 66.474348 |
| C14 | 1.109375 | .424058 | .292969 | 15.746058 | .491330 | 181.638310 |
| C15 | 1.171875 | .491447 | .384766 | 20.896003 | .672341 | 92.424381 |
| C16 | .613281 | .292662 | .223633 | 11.092797 | .417156 | 157.526286 |

All listed v targets are1.5m/s. C4 lookahead is zero-based native index13, forward1.125m; request labels remain1-based. Tangents are **derived geometric tangents, not native TIC-VLA yaw**. Short irregular/backward segments can yield large total winding; those values are not actual robot turn amplitudes or independent strong semantic evidence.

### Direct prediction → target → measured response

Accepted interval starts at request application and ends when the next accepted command replaces it (or run end). Each integral uses its actual saved tick duration, and actual Δyaw uses unwrapped measured pose at the same boundaries. Observation-to-observation intervals below serve a separate trajectory criterion; they must not be substituted for accepted command intervals.

| Accepted request | Target w (rad/s) | Hold (s) | ∫target=applied w (rad) | Actual Δyaw (rad / deg) | Response ratio |
|---|---:|---:|---:|---|---:|
| C4 | .099716 | .466667 | .046534 | .050117 / 2.871486° | 1.076993 |
| C5 | .095329 | .500000 | .047664 | .051005 / 2.922346° | 1.070074 |
| C14 | .491330 | .500000 | .245665 | .253348 / 14.515788° | 1.031275 |
| C15 | .672341 | .566667 | .380993 | .395155 / 22.640700° | 1.037169 |
| C16 | .417156 | .433333 | .180768 | .192495 / 11.029160° | 1.064876 |

The complete table includes mean measured yaw rate, rate RMSE, sign agreement and contact counts for every accepted request. Whole-run target/applied integral1.563500938rad and actual yaw1.483282819rad give descriptive ratio.948693271. Physics-rate vs applied RMSE=.039576021rad/s; pose-rate vs applied=.038334325rad/s; physics vs pose=.008901619rad/s. Physics and pose sign agreement are94.751773% and97.659574%, respectively. Startup lag and small later tracking errors remain visible; this is not ideal actuator tracking.

### Actual left-turn execution

The two consecutive qualifying intervals are **C15→C16 and C16→C17**, before the first excluded image:

| Observation interval | Sim time (s) | Episode forward Δ (m) | Episode left Δ (m) | Actual yaw Δ (deg) | Non-floor contact spans |
|---|---|---:|---:|---:|---:|
| C15→C16 | 8.316667100–8.816667126 | .462992 | .584654 | +19.065306 | 0 |
| C16→C17 | 8.816667126–9.316667153 | .257684 | .701748 | +14.971626 | 0 |

Both are positive-yaw and left-dominant measured displacements following stable left prediction/earlier forward progress. No vertical/steep environment contact span or camera-proximity hit was logged during these intervals, or elsewhere in the completed run. The contact classifier uses environment contact normals with |normal.z|<.5 and retains contacts until LOST; it is not a complete proof of collision-free geometry. The evidence does not indicate a turn produced only by a wall/contact impulse.

Final worldXY=(-20.547197342,9.542320251)m. Final official-episode-local forward/left=(8.052320251,27.927197342)m; **displacement from the measured settled start**, expressed in those same episode axes, is (8.050108433,27.927198410)m. Final signed yaw change **+84.985845344°**, maximum signed change+98.596330194°, total XY path34.924147770m and maximum measured body yaw rate.702711863rad/s. The world trace exhibits forward approach followed by a substantial left turn and hallway travel.

**NOVA CARTER E16 TURN EXECUTION VALIDATED.** This decision addresses conversion of the model/controller's own left-directed predictions into measured left motion.

### Visibility, goal and timing limits

Unmodified raw C4/C15/C16/C17/C18 RGB was visually inspected. C4 shows clear staircase context. C15–C17 are dim, with hallway/wall/floor context still discernible; their p95 values34/23/21 exceed the frozen20 cutoff, but C16/C17 are close to it. Do not describe them as uniformly bright or treat this simple brightness rule as a semantic visibility guarantee.

First degraded observation is **C18**, sim9.816667179s, p95=12/255. Excluded IDs are **C18–C37 and C39–C41**. All48 images and model results were retained; none tripped the unchanged std<1 capture abort. Those darker later predictions are excluded from clean turn evidence, even though their measured robot motion remains recorded. Empty 2cm camera-proximity queries do not prove non-containment or visual observability. The camera, scene lighting, exposure and guard were not tuned after output.

The robot came within **.445495638m** of the diagnostic goal, then continued past it to final goal distance **8.342377400m**. Therefore there is **no full-instruction/goal-stop success claim**. Nominal constant-speed lookahead execution and the requested absence of recovery/goal-stop additions remain unchanged. Late visibility limits further restrict policy-quality interpretation.

Non-bootstrap median/max action-call wall time=.055223432/.116006671s; observation-to-switch wall=.263643736/.556581371s; simulation delay=.083333338/.150000008s; XYZ transport=.125090777/.225152901m. These are context only, not an OLD/FRESH conflict, latency-induced failure or reconciliation validation.

## Interpretation and research implication

- **Repository-confirmed fact:** exact official scene/instruction/start and source front camera were used; installed DifferentialController maps two wheels; loaded radius/base differ from source defaults and were explicitly verified before inference. Official/model/controller extraction source remains unchanged.
- **Positive evidence:** the identical saved target history yields .079434° Jackal yaw versus15.522096°/15.496445° Nova yaw in the prespecified C4–C7 window. Nova's own closed loop then produces contact-free measured left-turn intervals consistent with its positive target w.
- **Negative evidence:** stopping at the specified goal was not demonstrated; the final trace overshoots. This does not reverse the narrower positive turn-execution result.
- **Trade-off:** source-camera fidelity and responsive actuation expose an observable turn, but do not guarantee full navigation success; small command tracking errors and dim later imagery persist. Slew makes little difference in the selected small-command window, not necessarily in other regimes.
- **Implementation limitation:** Isaac6 plus isolated Isaac5 model service,2Hz research scheduling, direct controller, float64 command wrapper and geometry-verified radius/base are explicit deviations from a complete unchanged official DynaNav runtime. Legacy fisheye rendering, brightness-only exclusion and limited contact/proximity classification constrain interpretation.
- **Insufficient evidence:** one episode/one48-prediction run cannot establish general TIC-VLA or physical-robot performance. StageA changes embodiment plus downstream wheel conversion/asset physics together; it does not isolate a particular friction, drive or mapping defect. StageB camera and state histories also differ, so matching Jackal/Nova request IDs are not identical-input comparisons.

Combined decision: **POSITIVE EVIDENCE THAT JACKAL/CUSTOM DOWNSTREAM EXECUTION WAS A MAJOR CONFOUND**. This does not establish that Jackal was the only cause.

Freeze this **simulated Nova Carter + official Hospital context** (verified .14m/.4132m geometry, actual front Hawk, current direct2Hz execution and saved config) as the execution platform for a separately specified next dynamic-obstacle experiment. Preserve the goal-stop/visibility limitations when designing that future experiment. This task adds no human, dynamic obstacle, cadence change, recovery, reconciliation, correspondence or graph method.

## Artifacts, checks and reproduction

StageA [response table](../outputs/nova-response-analysis-20261010-01/response_metrics.csv), [command/measured plots](../outputs/nova-response-analysis-20261010-01/figures/commanded_measured_response.png) and [embodiment comparison](../outputs/nova-response-analysis-20261010-01/figures/embodiment_comparison.png) are local generated evidence. StageB [request geometry](../outputs/nova-e16-analysis-20261010-01/request_metrics.csv), [accepted responses](../outputs/nova-e16-analysis-20261010-01/request_response.csv), [world trace](../outputs/nova-e16-analysis-20261010-01/figures/actual_world_trajectory.png), [episode-local trace](../outputs/nova-e16-analysis-20261010-01/figures/actual_episode_local_trajectory.png), [yaw response](../outputs/nova-e16-analysis-20261010-01/figures/target_measured_yaw_response.png), [response ratios](../outputs/nova-e16-analysis-20261010-01/figures/request_response_ratio.png), [chunk geometry](../outputs/nova-e16-analysis-20261010-01/figures/chunk_turn_geometry.png) and [automatically selected world chunks](../outputs/nova-e16-analysis-20261010-01/figures/selected_chunks_world.png) remain outside Git. The last selects previous C3/first stable C4; C3 is explicitly excluded startup, not a validated intent-transition pair.

All **65 Isaac-independent tests PASS**: previous54 plus11 Nova source/controller-sign/slew/tick-only ramp/replay/integral/unwrap/ratio/Jackal-regression/no-overwrite/accepted-boundary/OLD-retention checks. The local installed controller oracle is AST-extracted without importing Isaac; its two tests explicitly skip if that source is unavailable elsewhere. Run:

```bash
python3 -m unittest discover -s tests -v
```

Independent scalar recomputation verifies all48 accepted intervals and both288-row replay inputs. Maximum command-integral error2.78e−16rad, pose-yaw error2.50e−16rad, world-projection error3.56e−15m. StageA174 and StageB252 source hashes remain unchanged. The35 frozen research-code hashes and config hash still match; the freeze precedes every observation. All1636 pre-task protected files were checked, allowing only the two described integration edits and append-only work log; the three prior research reports, old raw outputs, upstream files and selected Isaac environment files are unchanged. No package, driver, CUDA, system Python, checkpoint or Isaac-installation modification occurred.

Saved-only reanalysis needs NumPy/Matplotlib/PyYAML/Pillow and no Isaac/model runtime. These commands use fresh output IDs; existing output paths are rejected:

```bash
python3 research/analyze_nova_response.py \
  --jackal-run outputs/hospital-e16-static-20261009-01 \
  --a0-run outputs/nova-a0-20261010-01 \
  --direct-run outputs/nova-replay-direct-20261010-01 \
  --slew-run outputs/nova-replay-slew-20261010-01 \
  --output-dir outputs/nova-response-analysis-REVIEW
python3 research/analyze_nova_hospital.py \
  --run-dir outputs/nova-e16-closed-loop-20261010-01 \
  --output-dir outputs/nova-e16-analysis-REVIEW
```

Historical simulator commands below record what was executed, not a request to rerun. Each was prefixed with `bash scripts/isaac6_python.sh`; source audit used `research/audit_nova_asset.py --output-dir outputs/nova-asset-audit-20261010-01` (then fresh `-02` after extending audit details).

```text
research/nova_hospital.py --mode a0 --run-id nova-a0-20261010-01
research/nova_hospital.py --mode replay_direct --run-id nova-replay-direct-20261010-01 --a0-run outputs/nova-a0-20261010-01
research/nova_hospital.py --mode replay_slew --run-id nova-replay-slew-20261010-01 --a0-run outputs/nova-a0-20261010-01
research/nova_hospital.py --mode closed_loop --run-id nova-e16-closed-loop-20261010-01 --stage-b-receipt outputs/nova-e16-setup-20261010/stage-b-freeze.json
```

The closed-loop entry point requires a prior StageA PASS, visual review and a pre-model receipt binding config/code hashes to one authorized fresh run ID. The historical receipt cannot authorize a different ID. Runtime/analysis metadata record starting Git SHA, upstream SHA, dirty diff identity, recursive research hashes, config/source hashes and strict checkpoint identity. Delivery is one focused `Validate Nova Carter execution on Hospital episode 16` commit and normal push to `origin/hardcase-probe`; final SHA/remote/clean-tree verification is recorded after commit in ignored `outputs/nova-e16-setup-20261010/git-finalization.txt`, avoiding a self-referential commit hash.

Not claimed: **no physical-robot validation; no dynamic-obstacle validation; no OLD/FRESH conflict validation; no reconciliation validation; no graph-optimization validation; no full DynaNav benchmark reproduction.**
