# Official Hospital episode 16: upstream behavior validation

This experiment asks whether released TIC-VLA produces forward motion, stable instructed left-turn prediction, and actual left-turn execution in an official scene through the current Jackal research harness. It is not a full DynaNav benchmark reproduction or a hard-case collection experiment.

## Source episode and identity

Starting local/remote `hardcase-probe`: `e7728409068bff3ca1066cfba93d1fd1a6f6e060`. Freshly fetched and independently checked live upstream `main`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. The two official source files below match that upstream commit byte-for-byte and were not edited.

- [`DynaNav/configs/benchmark_full.yaml`](../DynaNav/configs/benchmark_full.yaml), SHA256 `62d8abec03dc793dad9da351c5c5652bcd854014f29a91880309bd70c128b08c`.
- [`DynaNav/benchmark.py`](../DynaNav/benchmark.py), SHA256 `e0f9aeb78ce0b523e0ca8f11ff7da6f794b8b0aae63d722398d3ecc6d1fe01aa`.

Exact official entry:

```yaml
episode_16:
  scene: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Hospital/hospital.usd
  start: [7.38, 1.49, 0.01]
  start_yaw: 90.0
  goal: [-12.23, 10.19, 0.01]
  instruction: Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.
  timeout: 85
  num_people: 0
  robot_type: nova_carter
```

This episode has no people and explicitly commands a left turn after approaching a visible staircase. The exact instruction string is preserved after YAML parsing. Goal coordinates are provenance and XY-distance diagnostics only; the existing RGB/instruction/state/history model input schema is unchanged.

The downloaded top-level Hospital USD identity is 125626 bytes, SHA256 `83e5d606fbb2de873314379e0ff74900930735eb3ae99b312c9bf17db0033383`, server Last-Modified 2025-08-06. The runtime resolves the original URL, including its relative dependencies. The top-level hash is not a recursive checksum of remote textures/assets.

## Yaw and scene coordinates

The official episode loader passes `start_yaw` into `_spawn_robot` (`benchmark.py`, lines 398–400). That method gets/adds a USD `RotateZ` op and calls `rotate_z_op.Set(float(start_yaw))` (lines 1011–1072), also printing the angle with a degree symbol. Therefore:

| Field | Value |
|---|---|
| Official config | 90.0 |
| Official source convention | USD RotateZ, degrees |
| Research stored convention | radians |
| Applied research yaw | 1.5707963267948966 rad |
| Body forward in world | +Y |
| Body left in world | −X |

The research `official_usd` mode sublayers the complete Hospital source at its original paths, without translation/rotation/scale. Both source and composed stage use metres and Z-up; `/Root` has the same identity world transform. No research ground, corridor wall, green goal wall, obstacle or light is created. `/Root/GroundPlane/CollisionPlane` is the original Hospital floor, not the research default ground. Default configs without `scene.mode` keep their prior custom scene behavior.

## Official vs research differences

| Component | Official context | This experiment |
|---|---|---|
| Robot | Nova Carter (`nova_carter_sensors.usd`) | Existing four-wheel Jackal |
| Simulator | DynaNav Isaac 5 baseline | Isaac 6.0.1 compatibility installation |
| Runtime | Official DynaNav runner/behavior system | Existing continuous research harness |
| Front sensor | Official robot sensors | Unchanged Jackal front RGB mount/intrinsics, 640×480 |
| Observation cadence | Official benchmark behavior | Existing 2 Hz target cadence |
| Inference | Released TIC-VLA | Same strict checkpoint, isolated Isaac 5 Python, no second SimulationApp |
| Controller | Official behavior includes additional runtime policies | Existing 1 m lookahead extraction and four-wheel mapping, no added recovery |
| Scene/instruction/XY/yaw/people | Episode 16 | Exact official values/convention |
| Duration | Timeout 85 s | Prespecified 48 predictions; no full-goal requirement |

The scene-context change does not isolate causality from all other cross-run differences. No camera, model, preprocessing, checkpoint, controller, instruction or continuous-pacing tuning was performed.

## Preflight and freeze

Model-free `outputs/hospital-e16-preflight-20261009-01` stopped on a contact-audit implementation error: sleeping wheels do not keep emitting recent PERSIST callbacks. The corrected observational audit retains contacts until CONTACT_LOST and independently checks wheel-bottom geometry against floor raycasts. That failed output is preserved; it made zero model calls.

`outputs/hospital-e16-preflight-20261009-02` passed with the same physical configuration and zero model calls. Official Z=.01 is an episode value, not a Jackal body-center height; the official runner itself adds .1 m for Nova Carter. The unchanged Jackal collision geometry has a lowest local extent about −.0635 m. A .15 m spawn over floor Z≈0 provides settling clearance. Measured settled base was `(7.380000114, 1.489618421, .063499115)`, yaw `1.570796461` rad; XY settling displacement .000381579 m. All four wheels have upward floor contacts; wheel-bottom floor errors are below 3.5e−8 m. Four nearby downward PhysX rays hit the original Hospital ground. The 2 cm camera proximity query reports no environment collision.

Unmodified preflight image: [`hospital_start.png`](../outputs/hospital-e16-preflight-20261009-02/diagnostics/hospital_start.png). Visual review confirms the staircase ahead at center-right, foreground floor, door/corridor context, ceiling lights and furnishings. It is not a black scene or an image-statistics-only pass. The camera proximity test is not a general closed-mesh containment classifier.

Before the first model call, `outputs/hospital-e16-setup-20261009/prespecified-config.json` froze exact official parameters, config, code, source/preflight hashes, visual review, startup exclusion and rules. Receipt SHA256: `3e62f1e41282b2fd42852c493994197ebcf65eefd0e990bc8ea0dd375556e1c1`. Config SHA256: `6db99ccbdbfc10ce94301ca23f7842fab2f0e685614e1ef351e8fbccafd990e3`. The real-run CLI enforces the receipt's authorized run ID and config/code hashes.

The fixed horizon is **48 predictions**, nominally 24 s at 2 Hz plus bootstrap/call overrun, not a 24 s hard timeout. The start RGB already exposes the staircase. Official start-to-goal components are forward 8.70 m and left 19.61 m; their 28.31 m Manhattan distance is less than 36 m at the unchanged 1.5 m/s cap over 24 s. This preregistered opportunity for a first turn does not guarantee progress or full-goal success. No result-driven extension is permitted.

## Prespecified analysis

[`analyze_hospital_turn.py`](../research/analyze_hospital_turn.py) reads saved evidence only (NumPy/Matplotlib/PyYAML; no Isaac/model import). It validates continuous timing/OLD command retention, unique complete native requests and observation anchors. C1 is bootstrap; C2–C3 are additionally excluded warm-up; primary requests start at C4. All requests remain reported.

Each native array is `(30,2)` in metres, axes `[forward,left]`, relative to its own observation body frame. World projection uses **only** `agent_pose_at_observation`: `world = R(observation_yaw) @ native + observation_xy`. The episode-local frame uses official start XY and π/2 yaw: `local = R(π/2)^T @ (world − official_start)`. Robot poses are measured PhysX states, quaternion order wxyz. No origin waypoint or native yaw is invented.

Per-request CSV/JSON includes endpoint forward/left, maximum/mean left, original zero-based controller lookahead index, lookahead forward/left/bearing, v/w, observed world pose/time, latency/transport context, RGB quality and camera proximity. Initial/terminal segment headings and unwrapped total path heading change are **derived geometric tangent; not native TIC-VLA yaw**. Segments shorter than 1e−6 m are excluded; fewer than two valid segments leaves total turning undefined.

Four directional indicator groups are: positive endpoint **and** mean left; positive lookahead left **and** bearing; positive controller w; positive derived total heading change. Weak evidence requires at least three; stable intent requires all four in at least two consecutive eligible requests. Numerical sign tolerances are 1e−6 m/rad/rad·s⁻¹, solely for floating noise. Near-black observations (channel p95≤2/255) are excluded. No post-output magnitude threshold or combined severity score is selected.

Forward-dominant prediction has endpoint/lookahead forward greater than absolute left and fewer than three left-indicator groups. The automatic boundary is the first member of the earliest qualifying stable group; its preceding request is the transition OLD. Last forward-dominant eligible request before that boundary is reported separately. This is behavior characterization, not a reconciliation candidate.

Actual execution is checked independently: earlier clean forward-dominant measured motion, then two consecutive observation intervals after stable prediction with positive measured yaw change and episode-left-dominant displacement (`left > abs(forward)`). This conservative pre-run rule demands substantial change in motion direction without tuning a magnitude cutoff to outputs. Qualifying intervals exclude degraded images, camera proximity hits and measured vertical/steep environment contacts. The latter use |contact normal.z|<.5 and retain contact until LOST, including sleeping bodies. This contact diagnostic is not a complete collision classifier or a reconstructed Hospital map. One-second progress windows report distance/path length and commanded v without manufacturing a stuck severity score.

Continuous execution retains OLD targets while one inference is pending and advances physics with existing real-time pacing. Observation, action-call, response detection and application clocks remain distinct. Timing is secondary context; this task does not test a latency-induced conflict mechanism.

## Execution and incomplete-run handling

Exactly one real-model run was attempted: **`outputs/hospital-e16-static-20261009-01`**. The fixed target was 48 predictions. It saved **25** complete requests before the existing `sim.capture` RGB validity check raised `Invalid/blank RGB: (480, 640, 3)` at the next capture. The source run is **FAIL / INCOMPLETE**, with `failure.json` and no successful `summary.json`. C26 was neither archived nor inferred. Nothing was restarted, extended, retuned or synthesized to fill the missing requests.

Observation times are C1=0, C2=1.800000094, then .500000026 s gaps through C25=13.300000694 s; the last of 829 measured robot-state rows is at 13.800000720 s. Bootstrap physics advanced with zero OLD command. Strict checkpoint identity and all 25 finite `(30,2)` arrays, request IDs, observation poses, raw RGB, clocks, previous accepted targets and pending physics ticks were revalidated. There are 218 measured physics ticks wholly inside action-call intervals across this prefix.

After the abort, a saved-only loader was added to accept only an intact recorded blank-RGB-failure prefix and retain explicit FAIL/INCOMPLETE status. It does not bypass raw contracts, change the frozen phase/metric rules or rerun the model. The only post-freeze research code changes were this analyzer input handling and figure presentation. Config and simulation/model/controller/pacing code remain frozen. Initial analysis `hospital-e16-analysis-20261009-01` is retained; final output is **`outputs/hospital-e16-analysis-20261009-02`**.

## Prediction turn evidence

The earliest stable *directional-indicator* group is **C4–C7**, first member C4 at 2.800000146 s, confirmed by C5. It is already present at the first eligible request. No eligible request satisfies the prespecified forward-only phase before it, so **there is no established forward-dominant → meaningful left-turn phase transition**. Early curves have larger forward than lateral components but already satisfy left indicators. C4 is an eligibility boundary, not an inferred time when the model began following the instruction.

| Request | Endpoint F/L (m) | Mean left (m) | Lookahead left (m) | Bearing (deg) | v / w (m/s, rad/s) | Derived total turn (deg) |
|---|---|---:|---:|---:|---|---:|
| C3, excluded warm-up | 2.281250 / .255859 | .094590 | .034912 | 1.765238 | 1.5 / .084007 | 141.000378 |
| C4, first eligible stable | 2.203125 / .302734 | .104512 | .051514 | 2.483930 | 1.5 / .094936 | 175.459497 |
| C5, confirmation | 2.234375 / .472656 | .170282 | .063965 | 3.276906 | 1.5 / .118795 | 92.002109 |
| C7, end of early group | 2.078125 / .601563 | .208350 | .091797 | 4.508960 | 1.5 / .156859 | 70.144031 |
| C14, staircase context | 1.476563 / .574219 | .199023 | .176758 | 9.052898 | 1.5 / .314251 | −266.684742 |

**Tangent caveat:** these totals describe the unwrapped direction of successive native point segments, not robot rotation or commanded turn amplitude. Initial/terminal columns use that same unwrapped segment-angle sequence. C4 initial/terminal angles are −2.584513° / 172.874984°; its final step goes backward in forward-coordinate (.03125 m) while left increases .00390625 m. That short terminal reversal contributes a large angle although the curve predominantly extends forward. Some later paths wind across ±π multiple times (C22 total457.52°). No smoothing, post-output segment threshold or favorable tangent replacement was introduced. These large values **do not establish a large useful left turn**.

The automatic example is previous **C3 → C4**, both projected with their own observation pose. OLD command `(1.5,.084006940)` becomes FRESH `(1.5,.094935672)`; lookahead left .034912109→.051513672 m, endpoint left .255859375→.302734375 m. C3 remains excluded from primary stable evidence. There is no “last clearly forward-only request”; that field is null. This example is neither a sharp intent switch nor a reconciliation pair.

All C4–C25 pass the weak (three-of-four) sign rule; C8–C15 fail the all-four rule because derived unwrapped turning is negative. The numerical rule also produces a late C16–C25 group, but its dark RGB prevents a useful semantic-navigation interpretation. Only the clear early C4–C7 group supports the limited prediction-only conclusion.

## Actual turn evidence and failure context

| Measured quantity | Value |
|---|---:|
| Initial XY / yaw | (7.380000114,1.489618421) / 1.570796461 rad |
| Final XY / yaw | (7.264005661,19.096107483) / 1.572684800 rad |
| Forward displacement from settled start | 17.606489062 m |
| Left displacement from settled start | .115994453 m |
| Final official-start local F/L | (17.606107483,.115994339) m |
| Final signed yaw change | +.108193827° |
| Maximum signed yaw change | +.913898450° |
| Cumulative absolute yaw change | 2.428591466° |
| Executed XY path length | 17.634021584 m |
| Initial / minimum / final goal XY distance | 21.453408683 / 19.604661253 / 21.432102258 m |
| Qualifying sustained actual left-turn intervals | None |

The robot initially moves forward and continues almost straight. Positive w targets are not evidence of measured turn execution. Target/measured angular response differs markedly; this run does not isolate controller extraction, skid-steer dynamics, embodiment or friction as a cause. Do not infer those mechanisms solely from the mismatch.

There are 13 vertical/steep-surface contact reports from sim13.750000717–13.800000720 s, involving original Hospital wall/base trim and Jackal chassis. Some contact impulses are nonzero (maximum absolute component12.3471 in the raw report). The preceding one-second windows still cover approximately1.474 m at target v1.5; the trace ends only .05 s after first wall contact. This shows a terminal contact and RGB abort, **not enough post-contact time to establish sustained stuck behavior**.

Visibility degrades before that contact. Raw C4 shows the staircase and surrounding context; C14 is close to the staircase/rail; C16 is dark under/along the staircase. C17, C20, C22 and C25 are visually very dark, with much of the frame losing useful scene detail. In C16–C25, channel means are2.05–8.13/255 and p95 values5–20/255. The frozen p95≤2 near-black indicator flags **none** of the saved images; its failure to capture this qualitative degradation is explicitly retained, not fixed after seeing outputs. The existing std<1 capture guard later aborts C26 before saving it; its rejected image cannot be reviewed. All saved 2 cm camera proximity queries are empty, which does not prove good visibility or general non-containment. Do not claim the earlier darkness was caused by the later contact.

Supplemental visual review is stored in `outputs/hospital-e16-setup-20261009/saved-rgb-visual-review.json`. Unmodified raw references:

- [C4, first eligible stable prediction](../outputs/hospital-e16-static-20261009-01/diagnostics/rgb_000004.png).
- [C14, close staircase context](../outputs/hospital-e16-static-20261009-01/diagnostics/rgb_000014.png).
- [C16, onset of clearly reduced visibility](../outputs/hospital-e16-static-20261009-01/diagnostics/rgb_000016.png).
- [C25, last saved observation](../outputs/hospital-e16-static-20261009-01/diagnostics/rgb_000025.png).

No image exists for a validated actual turn because none was observed.

## Timing context

Across saved non-bootstrap C2–C25, median/max action-call wall time is .047346367/.296442092 s; observation→switch wall .128365080/.330043335 s; observation→switch sim .066666670/.316666683 s; measured XYZ transport while pending .099945488/.474869097 m. These are real continuous-execution context, not an OLD/FRESH conflict or latency-hard-case result. Original request timing records remain intact.

## Decisions and research implication

Stage 1: **OFFICIAL HOSPITAL EPISODE CONTEXT VALIDATED** — correct source scene, coordinate/yaw convention, exact instruction, people=0, supported Jackal start, clear relevant initial RGB. This validates the initial official scene context; it does not assert valid visibility for the entire run.

Stage 2: **TIC-VLA LEFT-TURN PREDICTION OBSERVED; EXECUTION NOT VALIDATED** — limited positive directional prediction evidence at C4–C7, negative actual turn evidence, no established forward-only→nontrivial left-turn transition, and incomplete later observation due to dark RGB/contact. This is not strong upstream behavior validation or full-episode success.

Do **not** proceed to a dynamic hard case or reconciliation from this result. The next separate uncertainty is **Jackal/custom-harness vs closer official Nova Carter execution**, including measured turning response and late visibility. No Nova Carter comparison, stronger custom scene, recovery, controller/camera tuning or dynamic obstacle was implemented. The exact episode/config is preserved as evidence, not promoted to a validated nontrivial base route.

## Reproduction and artifacts

The recorded one-off primary command (already executed; run-ID reuse is rejected):

```bash
bash scripts/isaac6_python.sh research/sim.py \
  --config configs/research/dynanav_hospital_episode16_static.yaml \
  --mode static --run-id hospital-e16-static-20261009-01 \
  --prerequisite-run inference-20261009-02 \
  --official-freeze-receipt outputs/hospital-e16-setup-20261009/prespecified-config.json
```

Saved-only analysis can be repeated with a **new** output directory:

```bash
python3 research/analyze_hospital_turn.py \
  --run-dir outputs/hospital-e16-static-20261009-01 \
  --output-dir outputs/hospital-e16-analysis-NEW
python3 -m unittest discover -s tests -v
```

Final derived output includes `request_metrics.csv/json` for all25, `summary.json`, provenance/source hashes, own-observation world arrays/transforms, actual interval diagnostics and six figures: actual world, episode-local motion, v/w, chunk turn geometry, actual yaw and selected world/native chunks. Raw source is never changed; output reuse or placement inside/above the source is rejected. Saved-only partial-prefix acceptance is tested and never relabels the original failed run PASS.

All **54 pure tests PASS** (43 existing +11 new). Independent scalar world projection agrees within3.56e−15 m; lookahead bearing and all saved image p95 values agree with raw data. All136 source-run file hashes match before/after analysis. The1315-file pre-task protected manifest confirms prior experiments, official source, controller/model/continuous code and selected environment files are unchanged, except the intended additive `sim.py` integration and append-only work log. No dependency/runtime installation occurred. Tests using socket IPC require the normal host runtime because this sandbox denies socket sends.

## Limitations and not claimed

One people-free episode, one incomplete run (25 of48 predictions), current Jackal embodiment/controller, Isaac6 compatibility, selected rather than exhaustive environment hashing, top-level rather than recursive asset identity, and limited camera/contact diagnostics. Derived tangents are sensitive to short native segments and winding; sign consistency alone does not establish a useful large turn. The saved RGB quality cutoff misses visually dark frames. There is no full-goal success, full official DynaNav runtime or broader TIC-VLA performance result. Earlier custom-scene negative reports remain unchanged.

**No dynamic obstacle claim; no OLD/FRESH conflict claim; no reconciliation claim; no graph-optimization claim; no full DynaNav benchmark reproduction; no general TIC-VLA performance claim.**
