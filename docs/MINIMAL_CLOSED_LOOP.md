# Controlled four-wheel TIC-VLA closed loop

This report preserves the validated frozen implementation/results. The additive continuous execution path, actual latency/transport measurements and separate config are documented in [CONTINUOUS_HANDOFF.md](CONTINUOUS_HANDOFF.md).

Date: **2026-10-09 (Asia/Seoul)**. This is a data-producing simulation platform, not a DynaNav reproduction, navigation benchmark, hard-case discovery result, or reconciliation implementation. The fork remains the home of future TIC-VLA research; no files from the prior LightNav repository were read, copied or changed for this task.

## Architecture and runtime

```text
Isaac 6: Jackal physics + mounted RGB + scripted human
    -> private socketpair: JSON event/state + lossless PNG
Isaac 5 Python (no SimulationApp): official DynaNav TICVLA.predict_async
    -> native BF16 (1,30,2), losslessly transported/stored as float32
Isaac 6: official lookahead formula -> (v,w) -> four joint velocities
    -> 30 physics steps -> fresh synchronized RGB -> next prediction
```

| Component | Actual runtime |
|---|---|
| Simulation | `/home/gpuadmin/isaacsim`, `6.0.1-rc.7+release.42383.32955d8d.gl`; Python 3.12.13, NumPy 2.3.1; RTX RayTracedLighting |
| Model only | `/home/gpuadmin/isaacsim-ticvla-5.0.0/python.sh`, `5.0.0-rc.45+release.23960.184afb15.gl`; Python 3.11.13, torch 2.7.0+cu128, torchvision 0.22.0+cu128, NumPy 1.26.0, transformers 4.57.6 |
| GPU | NVIDIA GeForce RTX 5060 Ti, 16,311 MiB, driver 595.84 |
| Environment changes | None. No pip installs, driver/CUDA/system Python changes, Isaac reinstall, shell-profile changes, or new inference framework |
| IPC | Inherited Unix `socketpair`, one outstanding sequential request; 8-byte length header, <=1 MiB JSON and <=8 MiB image; no pickle, public listener or file polling |

`scripts/isaac6_python.sh` isolates inherited CUDA/ROS library paths in the child process. The inference launcher separately removes Isaac 6 setup variables before starting the existing Isaac 5 interpreter. A real CUDA matrix operation passed there without starting SimulationApp; the service additionally checks that its SimulationApp module was not imported.

Physics and held wheel targets run at **60 Hz simulation time**. Predictions are requested at **2 Hz simulation time**, eight times per closed-loop smoke. Physics pauses during sequential inference and capture. These are not wall-clock rates and the native 0.1 s target spacing is a separate model contract. Background VLM reasoning still uses its original thread and wall-clock lifecycle.

## Robot, wheels and coordinate conventions

Selected asset: [official NVIDIA Jackal basic USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/Robots/Clearpath/Jackal/jackal_basic.usd), from the Isaac 6 asset collection. NVIDIA documents Jackal as a four-DOF mobile robot in its [wheeled asset catalog](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_robots_wheeled.html). It exposes four driven joints and stable pose/velocity reads through the Isaac 6 bundled `WheeledRobot` API. No local Jackal asset was present; the official remote reference was loaded and cached by Isaac.

| Property | Verified value |
|---|---|
| Root / body | `/World/Jackal` / `/World/Jackal/base_link` |
| Explicit command order | `front_left_wheel`, `rear_left_wheel`, `front_right_wheel`, `rear_right_wheel` |
| Articulation's own DOF order | FL, FR, RL, RR; the API resolves commands by the explicit wheel-name list |
| Wheel radius | **0.098 m**, from each collision `Cylinder.radius`; the visual mesh bounding radius is 0.100 m |
| Geometric track width | **0.37559 m**, joint centers at y = +0.187795 and -0.187795 m |
| Signs | `[+1,+1,+1,+1]`; local joint X rotated by `localRot0` points along body +Y for all four joints |

Every launch audits these USD joints, positions, collision radii and axes; geometry/sign mismatches fail. Values also agree with the [manufacturer's Jackal description](https://github.com/jackal/jackal/blob/noetic-devel/jackal_description/urdf/jackal.urdf.xacro). The geometric track is **not a calibrated effective skid-steer separation**: tire slip makes actual yaw response differ from the ideal equations.

Commands in rad/s use `left = (v - b*w/2)/r`, `right = (v + b*w/2)/r`, mapped `[left,left,right,right]`. Positive v means body +X; positive w means a left turn about +Z. Actual straight and positive-yaw tests precede model control.

World and robot conventions are meters, seconds, radians, Z-up, body FLU (+X forward, +Y left, +Z up). Canonical planar robot pose is `[world_x,world_y,yaw]`. Quaternions are **wxyz**, and `R(q)` maps body vectors into world; velocity conversion uses `R(q).T @ velocity_world`. Robot pose and wheel velocities come from the physics articulation, not integration of requested commands.

Native waypoints are `(forward,left)` position offsets sharing the **current observation body origin**, not differences between consecutive targets. No raw cumsum, smoothing, waypoint yaw, or world projection is performed. If projection is added later, its only anchor is the stored observation pose, never the ready/application pose.

## Camera and capture

Robot-fixed prim `/World/Jackal/base_link/research_front_rgb`, RGB **640x480**, body translation `[0.26,0,0.5]` m, quaternion wxyz `[0.5,0.5,-0.5,-0.5]`. USD camera optical -Z maps to body +X; image up +Y maps to body +Z. Focal length 10.4775 mm, aperture 20.955 x 15.71625 mm gives a 90-degree horizontal FOV; clipping 0.05–100 m. Pose settling precedes experiment time zero.

Capture flushes physics transforms with `world.render()`, resets DLSS temporal history, then uses `rep.orchestrator.step(delta_time=0, pause_timeline=False, wait_for_render=True, rt_subframes=4)`. This explicit completion barrier follows NVIDIA's [capture-at-simulation-time workflow](https://docs.isaacsim.omniverse.nvidia.com/latest/replicator_tutorials/tutorial_replicator_isaac_snippets.html) and [Replicator capture semantics](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/replicator_tutorials/tutorial_replicator_getting_started.html). The code asserts no simulation-time advance across capture. Nonempty RGB checks alone are insufficient; saved frames are also visually inspected.

Early probes used three `world.render()` calls alone. Frame review found likely stale/temporally blended data despite changed robot poses. Those raw outputs remain intact but are superseded for observation/pose alignment by the explicit-barrier runs below. Capture time means completed render/readback at a frozen physics state, not an invented hardware exposure timestamp.

## Preserved official model and replaced wrapper

Upstream baseline: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. **No changes to `DynaNav/`, `ticvla/`, the model architecture, or official controller source.**

The service imports the actual `DynaNav/ticvla.py` by path and reuses its `ticvla_vlm.py`, vision utilities, image tiling/normalization, BF16 precision, action head, asynchronous reasoning thread, reasoning text/KV generation, cache polling and `predict_async`. It constructs the original model and applies the official `model.` checkpoint prefix removal with `strict=True`.

| Artifact | Identity |
|---|---|
| Base | `OpenGVLab/InternVL3-1B`, revision `4415a3b810e636d11dfa86b0e9ba40bb00535aa8`; local `/home/gpuadmin/Workspace/models/tic-vla/InternVL3-1B` |
| Base weights SHA-256 | `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484` |
| Released checkpoint | `/home/gpuadmin/Workspace/models/tic-vla/checkpoints/TIC-VLA-model.ckpt` |
| Checkpoint SHA-256 | `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036` |

Both weight hashes are verified on service startup; the fuller model manifest is [artifacts.json](artifacts.json). The real output is BF16 `(1,30,2)`. Its 60 values are exactly representable in float32, so IPC and `.npy` use float32 without changing values/coordinates. Native dtype and both shapes remain recorded.

Changes from the DynaNav wrapper are explicit:

- Isaac 6 custom world, Jackal articulation, camera and one scripted human replace the benchmark scene/behavior/people system. No `omni.anim.people`, crowd avoidance, IRA migration, episode scores or DynaNav navigation manager is used.
- Lossless PNG transports camera RGB; the service decodes BGR and encodes default-quality OpenCV JPEG just like the original image buffer, then calls original preprocessing. Both diagnostic PNG and actual model-input JPEG are archived. No fabricated historical frames: current plus available frames at nominal -3/-6/-9 **simulation seconds** are selected. Original image-count sampling assumes 10 Hz; this adapter samples timestamps at 2 Hz.
- Original wrapper velocity convention is preserved explicitly: `[vx_body, -vy_body, vz_body, yaw_rate_body, dx_delayed, dy_delayed]`. `predict_async` drops vz and adds delay to form `[vx,vy,yaw_rate,dx,dy,delay]`. Lateral velocity is negated as upstream does; delayed displacement is **not** negated. The body-FLU raw measurements are retained beside model inputs.
- Delayed displacement is `R_start.T @ (p_now-p_start)` from the oldest of up to two recorded reasoning-start poses, matching upstream's deque convention. Delay is actual simulation-time difference instead of `frame_difference/30`. Tick means a 60 Hz physics count; it is not an upstream 30 Hz frame. Before any reasoning start, dx/dy/delay are zero as an explicit bootstrap policy, not missing event metadata disguised as zero.
- The first call retains upstream's initial reasoning wait; subsequent calls reuse/poll the original background cache. Reasoning start step/pose and completion **poll** step remain distinct. Exact worker-ready time and authoritative action-ready simulator pose are unavailable and recorded as null. The delayed-reference deque does not prove one-to-one identity with every internally consumed KV tensor.
- Previous-waypoint prompt text is built from measured one-second body displacements with the original formatting, zero filtering and 3D past-motion convention. These are past motion, not invented native future yaw. Sampling follows simulation time instead of fixed upstream frame counts.
- The extracted controller matches Nova Carter's waypoint-to-lookahead-to-command block: 1 m segment-arc-length lookahead, index clipped to 2…T-3, yaw filter gain .35, curvature `2*y/L^2`, epsilon .001, v limit 1.5 m/s, w limit 1.2 rad/s, and `.5*v*curvature + .8*filtered_bearing`. An AST-extracted original-source oracle tests parity. The arc-length cumsum is not a cumsum of native offsets.
- Only that block is reused. DynaNav backup/recovery, waiting gates and downstream command slew limiting are absent. One derived command is held for 0.5 simulation seconds and reapplied each physics tick. This is not full Nova Carter behavior and no controller improvement is claimed.
- Physics stays frozen through synchronous IPC; wall-time inference latency does **not** become physical scene motion during that wait. This harness cannot yet establish continuously advancing, wall-latency-induced handoff behavior. Torch/NumPy seeds are fixed; asynchronous worker scheduling, sampled language generation and GPU kernels do not imply bitwise deterministic model output.

## Single crossing scenario

All selected scenario/model/control parameters and the unchanged instruction are in [crossing_pedestrian.yaml](../configs/research/crossing_pedestrian.yaml):

`Move straight ahead toward the green wall, avoiding obstacles.`

Robot starts near (0,0) facing world +X; a green wall at (8,0) is the visible goal, with side walls at y=±2.5 m. Model runs use this same geometry and instruction. Wheel tests contain only ground, lighting, robot and camera.

The human is [NVIDIA's `male_adult_construction_03` USD](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd). Its source is meters/Z-up with measured bound height 1.938354 m; the adapter centers the visual and places its feet on the ground. A kinematic root and invisible capsule provide a simple collision proxy. The human-looking mesh translates in its authored **T/rest pose**, with no gait animation, walking policy, crowd AI or pedestrian realism claim. This is not a moving-primitive fallback.

World target: `p_H(t) = [2.5,2,0] + clip(t-0.5,0,4.0)*[0,-1,0]`, yaw -pi/2. It crosses the robot's nominal forward line at t=2.5 s. Targets update each physics tick; **PhysX global rigid-body positions** are read separately after stepping. The eight-prediction run ends at t≈4 s, before the configured path stops at 4.5 s. No thresholds were tuned to manufacture a hard case.

Top-level remote USD identities inspected on 2026-10-09:

| Asset | Bytes | SHA-256 |
|---|---:|---|
| Jackal basic | 1,176,779 | `af5bee665a5efb26a5835a9788a77f47316d80e4b530a5aad300daca77648395` |
| Human | 851,067 | `2e5d7fc72d6e2ed67f0ab86e2f3aaa91066019a4c71266361735956f232ccff0` |

`outputs/closed-loop-setup-20261009/asset-provenance.json` also retains URLs, sizes, ETags and modification dates. These hashes cover top-level USD files, not every referenced texture/dependency. Remote assets require network/cache availability and can change; there is no complete immutable asset bundle in Git.

## Raw records and clocks

Each run creates a fresh `outputs/<run_id>/` and refuses existing directories. All artifacts are ignored by Git. Raw records are write-once within a run; no previous probe is deleted or overwritten.

| Path | Contents |
|---|---|
| `metadata.json` | Scenario/config snapshot, command argv, mode, prerequisite run, exact local/upstream SHAs, dirty Git status/diff hash and runtime-source hashes |
| `simulation-runtime.json`, `inference-runtime.json` | Actual interpreter/runtime versions, model source, strict load, verified weights and GPU identity |
| `robot_asset.json`, `pedestrian_asset.json` | Loaded USD geometry/axes and human unit/visual/collision conventions |
| `robot_state.csv` | Initial state plus every 60 Hz physics tick: time, measured world pose/quaternion/velocities, four measured wheel speeds and commanded v/w/wheel targets |
| `raw/requests/request_NNNNNN.npy` | Exactly `(30,2)` finite float32 native values, common observation origin, meters, no yaw |
| `raw/requests/request_NNNNNN.json` | Unique ID, observation/image/instruction, event clocks/poses, raw semantic declarations, model state/delay/cache metadata and applied controller command |
| `raw/pedestrian_state.csv` | Per-tick authored world targets **and separately measured PhysX world xyz**, after stepping |
| `diagnostics/rgb_NNNNNN.png` | Synchronized front RGB for each request |
| `runtime_images/request_NNNNNN.jpg` | Actual model-input image buffer, retained for audit |
| `runtime-config.json`, `inference.log`, `summary.json` | Service config snapshot, runtime log and validation results; errors create `failure.json` |

`raw/` also contains requested control events, labeled as commands/targets rather than physical measurements. No transformed waypoint arrays or derived SE(2) data are generated. JSON and NPY are exclusive writes, but not a two-file atomic transaction: a crash can leave an incomplete pair, which must not be treated as a successful request. A missing PASS summary means an incomplete/failed run.

| Event field | Exact meaning |
|---|---|
| `observation.sim_time` | Isaac physics time minus the post-settling origin; `isaac_sim_time` retains the underlying value |
| `observation.wall_time_ns` / `monotonic_ns` | Host clock sampled immediately after the render-completion barrier, before the annotator-array copy/PNG write; frozen physics state, no sensor-exposure claim |
| `inference_start.*` | Service clock immediately before original `predict_async`, after image decode/JPEG/history/state preparation |
| `ready.*` | Service clock after action call, GPU synchronization and CPU conversion/shape/finite checks |
| `receive.*` | Simulator clock after full IPC response; distinct from service ready |
| `application.sim_time` / `wall_time_ns` | State sampled just after issuing the new four-wheel API target and before physics advances; not a measurement of motor response |
| `agent_pose_at_observation` | Measured robot world `[x,y,yaw]` at the frozen image state; full 3D state is also retained |
| `agent_pose_at_ready` | null: no independent simulator sample at the exact service-ready event; a receive-time state is separately retained |
| `agent_pose_at_application` | Measured state at command application; subsequent CSV physics samples prove actuation |
| Reasoning timing | Submission/start is identified by tick/pose; completion-poll step is not exact worker completion; worker-ready wall time is null |

`wall_time_ns` is Unix wall time, `monotonic_ns` is same-host monotonic time. Use monotonic differences for durations. Observation and application often share simulation time because physics is paused, but their wall/monotonic times differ. Stage C intentionally has null application/wheel commands because its prediction is not executed.

## Commands and stage gates

From the repository root, use fresh IDs; the names below are the retained validation runs. Reusing them must fail. The CLI requires a PASS summary of the immediately preceding stage (it does not automatically certify matching code/config; review the captured provenance when changing settings).

```bash
bash scripts/isaac6_python.sh research/sim.py --mode wheel --run-id wheel-20261009-03
bash scripts/isaac6_python.sh research/sim.py --mode inference --run-id inference-20261009-02 --prerequisite-run wheel-20261009-03
bash scripts/isaac6_python.sh research/sim.py --mode static --run-id static-20261009-02 --prerequisite-run inference-20261009-02
bash scripts/isaac6_python.sh research/sim.py --mode dynamic --run-id crossing-20261009-03 --prerequisite-run static-20261009-02
```

The launcher is invoked with `bash` (its existing file mode is not executable). The service starts/stops as an owned child; no unrelated Python process is killed. Timeouts bound startup/requests; failed stages must be investigated before proceeding. Large outputs, USD assets, caches and weights are not staged in Git.

Pure tests, with no simulator startup:

```bash
env -u LD_LIBRARY_PATH -u PYTHONPATH -u PYTHONEXE PYTHONNOUSERSITE=1 \
  /home/gpuadmin/isaacsim-ticvla-5.0.0/python.sh -m unittest discover -s tests -v
```

Ten tests cover straight/turn/arbitrary wheel mapping, camera axes, exact official-controller parity over deterministic cases, common-origin offsets, `(30,2)`/finite schema, unique IDs/no overwrite, bounded IPC, and the time-based human path. IPC needs a working local socketpair; the restricted tool sandbox denied socket sends, while the same tests in the authorized host runtime pass.

## Validation and readiness

Outcome: **READY FOR HARD-CASE COLLECTION**, within this controlled, sequential simulation scope. There is no remaining blocker to collecting native successive chunks in the single human-looking crossing scenario. Continuously advancing physics during inference and realistic human gait remain outside the validated scope. This does not claim a hard case was found or that navigation/reconciliation performance was validated.

| Validation | Result and evidence |
|---|---|
| Isaac 6 startup | PASS — RTX ready, scene initialization and actual physics |
| Four-wheel spawn | PASS — all four named DOFs resolved and audited |
| Straight wheels | PASS — 0.783938712 m forward with v=0.4 m/s for 2 s |
| Turn wheels | PASS — +0.291861864 rad yaw with w=0.6 rad/s for 2 s |
| RGB | PASS — mounted view, 640x480, explicit capture barrier; eight distinct images per closed-loop run |
| Model construction | PASS — original DynaNav TICVLA, InternVL3-1B |
| Official checkpoint | PASS — verified hash, strict load |
| Real RGB inference | PASS — Stage C uses actual captured RGB, no action application |
| Native output | PASS — original BF16 action tensor, no fabricated predictions |
| Shape / finite values | PASS — `(1,30,2)` native, `(30,2)` stored; every value finite |
| Official-style controller | PASS — native chunk consumed; source-parity tests pass |
| `(v,w)` | PASS — final dynamic request 1 gives `(1.5, 0.0030232481)` |
| Four-wheel application | PASS — each request applied through wheel-name-resolved articulation action |
| Actual robot motion | PASS — per-step measured poses and wheel velocities, not command integration |
| Second observation | PASS — new synchronized RGB at t≈0.5 s after measured motion |
| Second real prediction | PASS — request 2 and six further native chunks |
| Crossing human | PASS — human-looking USD, visually observed; PhysX displacement 3.500000238 m |
| Dynamic closed loop | PASS — eight real requests, wheel execution and moving human |
| Pure tests | PASS — 10 tests, separate from Isaac runtime evidence |
| Navigation score / hard-case analysis / reconciliation | NOT ATTEMPTED — outside task scope |

Final evidence set:

| Run | Predictions | Robot displacement | Robot yaw change | Human displacement |
|---|---:|---:|---:|---:|
| `wheel-20261009-03` | 0 | Straight/turn figures above | See above | — |
| `inference-20261009-02` | 1 | 0 (not applied) | 0 | — |
| `static-20261009-02` | 8 | 5.714657429 m | -0.005298114 rad | — |
| `crossing-20261009-03` | 8 | 5.716396844 m | -0.007124606 rad | 3.500000238 m |

Closed-loop displacement is net planar initial-to-final distance, not path length. Each loop executes 240 physics ticks over ~4 simulation seconds and records 241 robot states. Dynamic human measurement count is 240; target-vs-PhysX maximum position difference is `5.874e-8 m`. Requests occur at simulation times 0, .5, …, 3.5 s. The final run's saved source/config/launcher hashes match the delivered implementation.

The post-run audit checks all 17 selected real predictions for shape, finite values, IDs, distinct RGB, 0.5 s scheduling, ordered monotonic observation/start/ready/receive/application clocks and raw records. An independent diagnostic projects the known green-wall corners using each static observation pose and camera extrinsic/intrinsic. Maximum image-edge discrepancy is **1.535 pixels** for final synchronized capture, compared with **36.869 pixels** in the earlier render-only static run. This is a scene-specific alignment check, not general camera calibration. Evidence: `outputs/closed-loop-setup-20261009/raw-audit-final.json`.

Earlier `wheel-01` used visual radius .100 m before inspecting the .098 m collision cylinder; corrected runs supersede it. Earlier `static-01` and `crossing-01/02` completed real inference/motion but are superseded for camera/pose alignment. Their raw files and original summaries are unchanged; they must not be used as synchronized collection evidence. `crossing-02` first added direct PhysX human measurements. An initial direct launcher attempt failed with permission denied before creating a run; invoking the existing script with `bash` resolved it.

Both installations' VERSION/python.sh/requirements hashes match the pre-task record. Isaac 6 pip freeze matches the prior baseline byte-for-byte when run in the same inherited environment from `/tmp` (SHA-256 `5eabcde29d898ca9743569f4d5b8a2d801ba500b13fa7302bd03bffe274bb855`). An isolated launcher inventory from the repo differs because ROS paths are removed and local TIC-VLA package metadata is visible; this is not package installation/removal. Driver 595.84, system CUDA 12.6 and system Python 3.12.3 remain unchanged. These are selected integrity checks, not a complete installation-tree checksum.

Starting local/remote research SHA: `26624415655eb2600d425430ec36e669788de87f`; branch `hardcase-probe`. Origin and upstream main were rechecked at `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Each experiment records the starting SHA plus its dirty file hashes; no run is falsely labeled as already executed at the future delivery commit. The resulting commit/remote verification is written after commit to `outputs/closed-loop-setup-20261009/git-finalization.txt` and reported to the user.
