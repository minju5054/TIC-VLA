# Nova Carter / Hospital dynamic-human RAW handoff probe

Date: 2026-10-10 (Asia/Seoul). Scope: one scripted dynamic human, saved-only OLD/FRESH characterization and stopped-physics Isaac GUI replay. All actors are simulated. No graph, correspondence, rigid alignment, blending, compensation or reconciliation is implemented.

## Question and frozen baseline

Q1: does a human moving into an already accepted, valid turn future coincide with a valid FRESH geometry revision? Q2: after actual OLD execution during inference, what RAW geometric seam appears at application of observation-anchored FRESH?

Starting local/origin hardcase-probe SHA **7f74a9f0f17d14d8410abca6eae907427490ee7f** was freshly fetched/checked. Upstream/main **9fa6f8b66b9e121d5df5df071297bba8e5353ebb**. The read-only no-human baseline is **outputs/nova-e16-closed-loop-20261010-01**. It has 48 real native `(30,2)` predictions and measured left-turn execution; C15 target w +.672341 rad/s, C15→C16 measured yaw+19.065306° and left .584654 m. This prior evidence supplies the base-route prerequisite; no static run is repeated.

The primary pair is fixed before inference: **OLD C15 → FRESH C16**. C14→C15/C16→C17 are secondary only. Official Hospital USD / episode 16 XY/yaw/instruction, released checkpoint, seed 36, actual front Hawk 1920×1080, direct DifferentialController radius .14 m / base .4132 m, 1 m lookahead, 60 Hz physics/control, 2 Hz target observations, continuous physics and one outstanding request are unchanged. OLD command remains active while FRESH is pending. There is no cadence/latency sweep, artificial delay, controller/goal-stop change or recovery policy.

## Human source and intervention derivation

DynaNav's Hospital data-collection template delegates character assets/spawning to Isaac Replicator Agent. Installed official IRA 0.7.19 defaults to /Isaac/People/Characters/ (with 4.5 fallback), and CharacterRandomizer documents walking at approximately 1 m/s; its 1.1 m/s value is a command-duration estimate, not an exact gait-speed contract. DynaNav human collision scoring uses a separate .2 m center-distance check, which is not reused as a physical body radius here.

Visible asset: **https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd**. This is the already validated Isaac 6 member of the same official People character family. Top-level 851067 bytes, SHA256 **2e5d7fc72d6e2ed67f0ab86e2f3aaa91066019a4c71266361735956f232ccff0**; dependencies are not recursively checksummed. Source USD units/up-axis/bounds and exact visual transform are archived. The visible human is an actual character mesh in its authored rest/T pose, translated continuously; **no walking gait or full DynaNav crowd-policy reproduction** is claimed.

The human exists from scene initialization, before world reset/settling and before the first model observation. A hidden physical capsule uses configured radius .25 m and cylinder height 1.2 m. Position is measured from PhysX every 60 Hz tick and at observation/response-detected/application. CSV yaw is the fixed configured root orientation; it is not a separately estimated heading signal. Trigger is solely acceptance of request 15. It records C15 application and current human position without changing pose; the next physics tick advances by speed×dt. No spawn/teleport or prediction-content-dependent placement occurs.

Placement uses only saved baseline evidence:

1. Crossing center is C15's one-based point 15, nominal +1.5 s, projected using C15's own observation pose: **(3.674660743, 10.878066808) m**.
2. Baseline measured C15→C17 turn chord gives unit tangent **(−.872422095,+.488753198)**. Crossing is perpendicular to this measured direction.
3. A conservative body-origin circumradius is computed from loaded Nova mesh vertices and analytical primitive bounding corners, including visual and collision geometry. Result **.606982542 m**; largest bound is a caster collision cylinder. Stale authored primitive extents are not used. This intentionally conservative radius is a diagnostic footprint proxy, not a change to wheel geometry or an exact yaw-dependent collision oracle.
4. Combined human/Nova radius is **.856982542 m**. Each staging candidate is center±normal×(combined radius + .20 m). Both directions receive identical floor/collision checks; the first candidate satisfying all conditions is selected.
5. Both paths were physically clear in the preflight query, but side+1 already conflicted with the curved OLD polyline (−.048104 m), so side−1 was selected. Selected staging OLD clearance **+.143156584 m**; at the baseline C16 observation delay after C15 application (.416666688 s), expected OLD clearance is **−.261603514 m**.

Frozen human path:

| Property | Value |
|---|---|
| Start/staging XYZ | (4.191264340,11.800201732,0)m |
| End XYZ | (3.158057145,9.955931885,0)m |
| Speed | 1.0 m/s, then stop at endpoint |
| Heading | −2.081456380 rad, fixed |
| Trigger | Immediately after C15 acceptance/application |
| Path length | 2.113965084 m |

Preflight **outputs/nova-human-preflight-20261010-03** PASS, zero model calls. Query samples at≤5 cm spacing use downward floor rays and .25 m overlap spheres at five body heights, preserving Hospital walls/stairs/rails. This is a conservative sampled geometric check, not crowd/navmesh validation. Measured final human endpoint error 2.31474e−7 m.

Earlier model-free outputs are retained: -01 exposed CollisionAPI on an Xform container in the footprint audit; fixed by traversing actual descendant mesh/primitive geometry. -02 completed movement checks but failed YAML serialization of NumPy scalar coordinates; native float conversion fixed this without changing placement/speed. Its empty config is preserved in setup evidence. No dynamic model had been called.

The preflight original-camera image shows the character may be visible before trigger. Staging outside OLD does **not** guarantee hidden visual cues. C1–C15 baseline comparison is therefore required; do not assume an identical pre-intervention policy state.

## Pre-model freeze and analysis definitions

Config: configs/research/nova_e16_dynamic_human.yaml, SHA256 **77a5f2bfb209b91b215c2a36854ead0e8f7a4873e66264c2bf41603168ad60e2**. Receipt: outputs/nova-dynamic-setup-20261010/pre-model-freeze.json, SHA256 **5f8f9f424d3e1fe3348e7e901c75381d88d348221c13b89cd26609293653f23b**. It binds 43 research-code hashes, 272 baseline/preflight/source files, 79 passing pure tests, primary IDs, placement and one authorized 48-prediction run ID before inference.

For each chunk, world points are observationXY+R(observationYaw)×native(forward,left). No ready/application re-anchoring or pre-alignment is performed. Native output has no yaw/time channel. Waypoint timing is assigned from the source-confirmed model target convention, +.1…+3 s.

Consecutive curves are linearly interpolated over common absolute nominal target time, using FRESH nominal knots and exact overlap endpoints. No nearest-neighbor/ICP/DTW correspondence. Signed world displacement is rotated into the FRESH observation frame; report forward/lateral components, Euclidean mean/RMSE/max and derived segment-tangent changes. Common-horizon endpoint shift uses the same absolute time; full native endpoints and controller-selected lookahead points are separately labeled because they have different target times.

Human conflict at C16 uses the measured human center held fixed: minimum **segment-to-center** distance minus combined radius. OLD is clipped to its remaining nominal future from C16 observation; FRESH uses its native future. A separate dynamic diagnostic temporally interpolates measured human motion and nominal robot curves, unions all temporal knots, and minimizes the piecewise-linear relative-distance segments. It never extrapolates beyond recorded human coverage.

RAW boundary B is the C16 application pose, after command-target application and before the next physics step. With tau=applicationSim−observationSim, FRESH position at tau is interpolated using an observationXY anchor at tau = 0 **only in the derived diagnostic**. Position seam is distance from B to that point. Tangents compare FRESH's local segment to measured OLD motion over the last .1 s, clipped to observation time, and separately to B's robot yaw. Tiny segment/window displacement≤1e−6 m gives undefined tangent; out-of-horizon tau gives undefined seam, never extrapolation. These are derived geometric tangents, not native TIC-VLA yaw.

Validity requires finite usable geometry, a path reaching the unchanged 1 m lookahead, p95 > 20/255 visibility context, no camera-proximity or relevant contact artifact, and OLD outside the human proxy originally. Static validity is limited to measured contact/RGB context, not a complete predicted-polyline Hospital collision oracle. Strong qualification additionally requires new OLD conflict, valid FRESH avoidance, actual nonzero OLD transport inside inference and a nonzero RAW seam. Numerical 1e−6 tolerances are not severity thresholds. Otherwise classify partial revision, no useful revision or insufficient evidence; no result-driven success threshold or primary-pair selection.

## Dynamic evidence and primary decision

Exactly one real-model dynamic run was executed: **outputs/nova-e16-human-20261010-01**. It completed 48 finite native `(30,2)` predictions and 1,523 aligned robot/human tick records through sim 25.366667990 s. All requests retain source frame/axes/units and observation/detected/application metadata. No position, speed, asset, trigger, seed, controller, camera, cadence or prediction count was changed after the first model call. The no-human baseline was never rerun.

C15 acceptance triggered human motion at **8.450000441 s**. C15 observation was 8.366667103 s; C16 observation/application were **8.866667129 / 8.966667134 s**. Trigger-time position equals the staged position, and the maximum measured per-tick human movement is .016667127 m, consistent with 1 m/s at 60 Hz within float measurement precision. There is no teleport.

| Human snapshot | Measured world X [m] | Measured world Y [m] |
|---|---:|---:|
| C15 observation | 4.191264153 | 11.800201416 |
| C15 application, before acceptance callback | 4.191264153 | 11.800201416 |
| C16 observation | 3.987617254 | 11.436692238 |
| C16 application | 3.938741922 | 11.349450111 |

**Primary classification: INSUFFICIENT EVIDENCE.** The attempt does not establish a valid OLD-conflict/FRESH-avoidance hard case. Both chunks are finite and usable, but C15/C16 original RGB p95 values are **19 / 12** on the 0–255 scale, failing the pre-frozen visibility rule (p95 > 20). C15 is the first degraded image; C15–C39 are excluded for visibility. C48 separately has four logged static-contact spans. There is no logged human contact and no primary-window static contact. All raw records remain available.

OLD's initial human clearance is **+.724200751 m**, so the human initially lies outside its predicted corridor. At C16's measured scene state, the fixed-human proxy clearance is **+.308888841 m for remaining OLD** and **+.327155160 m for FRESH**. Neither intersects. Improvement is only +.018266319 m. Thus the intended new-conflict condition also failed independently of image validity: the actual pre-trigger closed loop had already diverged from the saved baseline used to design the intervention.

### Geometry, transport and RAW comparison

Both columns use the same saved-only analyzer. Positive forward/left revisions mean FRESH is farther forward/left in the FRESH observation frame. Tangents are **derived geometric tangents; not native TIC-VLA yaw**. Large tangent maxima can reflect irregular short/far segments rather than a large executed turn.

| C15→C16 metric | Dynamic human | Saved no-human baseline |
|---|---:|---:|
| Common absolute nominal-time samples | 25 | 25 |
| Overlap RMSE [m] | .701867702 | .626602818 |
| Overlap mean / max [m] | .674143467 / .994861480 | 0.610554381 / 0.864757850 |
| Mean absolute forward revision [m] | .663871410 | 0.601391458 |
| Max absolute forward revision [m] | .966180868 | 0.834438679 |
| Mean / max absolute lateral revision [m] | .110366054 / .237157950 | .084325586 / .226976276 |
| Mean signed lateral revision [m] | +.109300708 | -0.080635137 |
| Common-horizon endpoint shift [m] | .994861480 | .819008187 |
| Full native endpoint shift, different target times [m] | 1.317445842 | 1.045555628 |
| Controller-selected lookahead shift, different target times [m] | .775378170 | 0.800189434 |
| Mean / max absolute tangent revision [deg] | 38.987570 / 139.832205 | 41.647147 / 138.685666 |
| Controller delta_v [m/s] | 0 | 0 |
| Controller delta_w [rad/s] | +.000479876 | −.255185079 |
| C16 action-call wall latency [s] | .068752140 | .111484903 |
| C16 observation→application wall [s] | .273959149 | .352171623 |
| C16 observation→application sim [s] | .100000005 | .150000008 |
| Robot translation during pending interval [m] | .150038841 | .224679059 |
| Robot yaw transport [deg] | +1.467111 | +6.029955 |
| Human translation during pending interval [m] | .099999933 | absent |
| RAW position seam [m] | .013713487 | .045590723 |
| RAW executed→FRESH tangent seam [deg] | 1.454847 | 2.412858 |
| RAW heading→FRESH tangent gap [deg] | 2.070010 | 4.112286 |
| Measured OLD tangent-window displacement [m] | .150038833 | .149824644 |

Dynamic common nominal time is **8.966667129–11.366667103 s**. Geometry revision is predominantly longitudinal/progress-related, with a smaller leftward component and far-horizon displacement approaching 1 m. These are nominal prediction differences, not actual future execution errors. Command v remains 1.5 m/s and w changes only **.242392508→.242872384 rad/s**. There is no route-reversal or abrupt control-jump evidence.

C16 has six physics ticks between observation and application, including **four ticks inside the actual action call** with OLD retained. The run has 260 ticks inside action calls, 151 excluding bootstrap. This verifies real continuous OLD execution; wall latency also includes capture/IPC/render/poll overhead and must not be treated as action-call latency.

RAW boundary B is **(3.219877243, 10.540268898) m**, yaw 2.630971733 rad. FRESH at tau=.100000005 s is **(3.227404101, 10.528805647) m**. The measured OLD tangent window is 8.866667134–8.966667134 s; its tangent is 2.620235111 rad versus FRESH 2.594843237 rad. A measurable RAW seam exists, but all three primary seam measures are smaller than the saved baseline. No success threshold is introduced, and no necessity for reconciliation follows.

A separate **time-varying measured-human / nominal-robot** diagnostic gives OLD clearance **−.367208010 m**, FRESH **+.079419620 m**. This is an avoidance-like pattern when comparing the full nominal temporal futures with subsequent human motion. It does not replace the predeclared fixed-at-C16 conflict test, repair failed visibility, prove an executed collision or establish human causation.

Secondary C14→C15 and C16→C17 RMSE values are **.485501469 / .426955250 m**, respectively; both are also classified INSUFFICIENT EVIDENCE. They do not replace the primary pair.

### Pre-trigger consistency and limitations

| Request | Observation XY difference [m] | Yaw difference [deg] | World chunk RMSE at equal relative nominal horizon [m] | Dynamic minus baseline target w [rad/s] |
|---|---:|---:|---:|---:|
| C1 | 0 | 0 | .148040811 | −.133970284 |
| C5 | .052912820 | −1.285713 | .118033678 | +.094317863 |
| C10 | .187907563 | +5.863379 | .386073562 | +.045242104 |
| C14 | .621901105 | +10.951000 | .734081604 | −.102565292 |
| C15 | .752727660 | +8.950374 | .671641609 | −.429948886 |

C1 starts at the same measured pose/time, but outputs already differ. The staging human is visible in original early RGB and the C1 geometric frustum/LOS checks are positive. Subsequent observations differ by about .05 s because bootstrap response timing differs. The .753 m C15 pose divergence precedes the human-motion trigger, so baseline-designed crossing cannot guarantee occupation of the actual dynamic OLD corridor. A pre-trigger human cue may contribute, while stochastic generation/reasoning and subsequent state differences prevent isolated attribution.

The frustum diagnostic approximates a fisheye camera with a pinhole check. The line-of-sight query can hit the robot's own chassis (including C12–C16); it does not filter self-geometry. A false LOS result therefore does **not** prove Hospital-wall occlusion. Original RGB remains the visual source of truth. The paired CSV also records own-local shape differences only as **INVALID AS FRAME-INVARIANT REVISION METRIC**, never as the primary revision result.

This is one episode, one intervention and one 48-prediction run. The character uses a fixed authored rest pose and a configured capsule. The conservative circular proxy, sampled preflight and measured contact classifier are not a full collision oracle. Nominal target timing is not an intrinsic timestamp channel or guaranteed execution schedule. Low-light frames, pre-trigger state divergence and later static contacts restrict interpretation. No general navigation, dynamic-obstacle, real-world, automatic correspondence, graph or reconciliation performance claim is made. There was no result-driven rerun, parameter adjustment, artificial latency or correction implementation.

## Saved analysis artifacts and reproduction

Analysis output: **outputs/nova-e16-human-analysis-20261010-01**. It contains 48-row dynamic/baseline CSV and JSON metrics, `primary_pair.json`, `baseline_primary_pair.json`, `secondary_pairs.json`, pre-trigger CSV/JSON, detailed aligned curves, source/code hashes and seven figures. `figures/` contains world overview, primary close-up, temporal forward/lateral/displacement, RAW seam, fixed/time-varying human clearance, command timing and baseline comparison. Generated evidence remains ignored by Git.

Saved-only analysis command (fresh output directory required):

```bash
MPLCONFIGDIR=/tmp/ticvla-matplotlib python3 research/analyze_nova_dynamic_handoff.py \
  --run-dir outputs/nova-e16-human-20261010-01 \
  --baseline-run-dir outputs/nova-e16-closed-loop-20261010-01 \
  --freeze-receipt outputs/nova-dynamic-setup-20261010/pre-model-freeze.json \
  --output-dir outputs/nova-e16-human-analysis-REVIEW
```

The historical simulation CLI is recorded in the run metadata. Its one-use authorization receipt binds the original config, code and run ID; it must not be reused to rerun navigation. The only post-model research-code edit is the viewer: an asset quaternion precision fix and display/UI adjustments. Runtime, analyzer, placement, thresholds and primary pair remain as frozen. Both the original freeze hashes and final viewer hash are retained.

## Isaac GUI replay and verification

Viewer: **research/view_nova_dynamic.py**; pure replay data reader: **research/nova_replay.py**. Validated non-headless output: **outputs/nova-e16-human-gui-20261010-05**. `ready.json` reports **GUI_READY**, zero model calls, no physics re-execution and unchanged source hashes. `viewer_scene.usda`, `metadata.json`, `overview.png` and `desktop-validation.png` are saved. The desktop capture confirms the control panel and original recorded RGB image; viewport captures confirm Hospital, Nova, visible character and colored guides. Self-test exercises recorded robot/human transforms, primary jump and all three camera choices with the physics timeline stopped.

The viewer loads the exact source Hospital/Nova/character assets. It selects the last recorded physics tick at or before replay time (no trajectory resimulation); root quaternion ordering is WXYZ. Robot/human pose updates use saved measured positions. Human heading remains its configured fixed yaw. Robot wheel articulation and human gait are not reconstructed. Selected chunks remain fixed in world coordinates at their own observation poses.

Cyan/magenta show measured robot/human paths; orange/green show selected OLD/FRESH; white is FRESH application B; yellow is FRESH observation; the red guide is the inflated human proxy **at selected FRESH observation**, not the human's current replay location. Curves are lifted slightly above the floor for readability. Overview clips the roof with its near plane. An interior inspection light follows the actors **for display only**; the original recorded RGB panel is never relit or modified. The camera named Nova front Hawk is a reconstructed viewport, distinct from archived model input.

Controls provide Play/Pause/Restart, .25/.5/1/2× speed, timeline, previous/next observation, OLD/FRESH selection, primary-pair/jump buttons and Overview/Follow/front Hawk cameras. The information panel reports time, recorded tick, active/pending request, robot/human state, command and selected FRESH seam. Primary jump starts .25 s before C15 application at .5× speed. `FRESH C16 pending | OLD C15 active` and `RAW SWITCH C15 → C16` are driven by saved events. Selected curves are retrospective: showing green before C16 acceptance is an inspection overlay, not evidence that the controller already had that prediction.

The validated export has **46 PNG frames at 20 fps**, covering **8.200000441–10.450000441 s**, in `frames/` with exact state/event records in `frames.json`. Frames 14–15 record C16 pending / C15 active; frame 16 records the raw C16 switch. Existing ffmpeg was unavailable, so no MP4 was created and no dependency was installed. Viewport frames omit the GUI panel; the interactive GUI carries timing labels and original RGB.

All earlier viewer outputs are preserved: -01 failed because the referenced Nova quaternion attribute was float precision; a separate recorded-transform op fixes it. -02 initialized/exported but was too dark, -03/-04 tested external lighting which was occluded by the Hospital ceiling, and -05 uses an interior light and passed visual review. These are saved-pose GUI launches, **not navigation or model reruns**.

From the repository root, the following exact command opens the saved result. The review output name must not already exist; use another fresh output name for later launches.

```bash
bash scripts/isaac6_python.sh research/view_nova_dynamic.py --run-dir outputs/nova-e16-human-20261010-01 --analysis-dir outputs/nova-e16-human-analysis-20261010-01 --output-dir outputs/nova-e16-human-gui-REVIEW
```

Add `--self-test --export-primary` for a new recorded-pose validation/export. The GUI starts paused just before the primary intervention; press **Jump to dynamic handoff** to play that sequence.

## Validation and delivery

**79 pure unit tests PASS**: the original 65 are retained and 14 cover acceptance-only trigger, continuity/determinism, baseline derivation, freeze enforcement, segment-based clearance, temporal overlap, dynamic relative-motion clearance, RAW seam/wrapping/no extrapolation, recorded transforms, source immutability, primary selection and absence of model/physics execution in the viewer. Run `python3 -m unittest discover -s tests -v`; the existing socketpair test needs ordinary host local-socket access. A sandbox-only run failed that one existing IPC test with EPERM; the full host run passes without changing it or dependencies.

Independent scalar verification reproduces both primary RMSE and RAW position seam to 1e−12. It confirms all 48 raw chunks, exact human/robot tick alignment, motion continuity, freeze-before-observation, 272 frozen source hashes and 43 frozen code hashes (viewer-only post-model changes). A 1,949-file protection manifest verifies earlier outputs, official sources and Isaac runtime identity; only the two declared integration edits and append-only work log are allowed to change. Verification receipts and logs live under **outputs/nova-dynamic-setup-20261010**.

Only additive research code/config/tests, this document, the work-log append and two small integration hooks are committed. Official TIC-VLA/DynaNav sources, baseline/raw evidence, environments, checkpoint and earlier reports are preserved. Delivery uses the focused **Probe Nova Carter dynamic-human handoff conflict** commit and normal `git push origin hardcase-probe`; final commit/remote/status are recorded in the ignored setup receipt and final report.
