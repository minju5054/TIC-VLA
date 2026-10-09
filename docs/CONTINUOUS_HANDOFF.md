# Continuous TIC-VLA handoff

## Motivation and scope

The frozen harness validates RGB → real prediction → controller → four-wheel execution, but removes transport during the action request. This additive path asks whether actual action availability latency allows measured robot/environment transport between observation and FRESH application. It preserves the corridor, Jackal, camera, instruction, released checkpoint, controller and scripted crossing parameters. It is not a DynaNav port or navigation benchmark.

The frozen config `configs/research/crossing_pedestrian.yaml`, blocking `research/loop.py`, official model/service/controller, and prior frozen evidence remain preserved. [CHUNK_GEOMETRY_ANALYSIS.md](CHUNK_GEOMETRY_ANALYSIS.md) is unchanged: its positive geometry-revision evidence and insufficient pedestrian hard-case evidence are not reinterpreted as latency evidence.

## Execution semantics

Use `configs/research/crossing_pedestrian_continuous.yaml`. The only config differences are `pause_physics_during_inference: false`, `real_time_pacing: true`, and `target_real_time_factor: 1.0`. There is no forced inference delay.

`research/continuous.py` wraps the existing blocking private socketpair client with one `ThreadPoolExecutor(max_workers=1)` future. A second submission is rejected until the first response is consumed, even if it has already arrived. Only request dictionaries and encoded image bytes enter the worker. All Isaac APIs, control application, physics stepping, camera capture and measured-human reads stay on the main simulation thread. Isaac 5 Python hosts the unchanged official model without SimulationApp; Isaac 6 owns simulation.

Request 1 is **bootstrap**: no previous source request and `old_controller_command: null`. Its stationary wheel target is explicitly `(0,0)`, without inventing an OLD chunk. Bootstrap includes the official initial reasoning wait and is excluded from all handoff aggregates/proofs. Small measured rest-state drift is not OLD/FRESH transport evidence.

For requests 2–8, OLD is the currently active `(v,w)` derived from the previous accepted chunk. Every pending physics tick reapplies that command and its four wheel targets. `robot_state.csv` adds `monotonic_ns`, `physics_step_start_monotonic_ns`, `active_control_source_request_id` and `pending_request_id` only in continuous mode. These fields identify actual ticks and the targets applied to them; wheel velocities and robot poses are also measured. The waypoint array is not a timed execution queue. Committed history is the actual measured trajectory; `MotionHistory` retains measured one-second body-frame displacement prompts.

After `future.done()`, the main thread samples response-detection state, consumes the native action, computes the unchanged lookahead command, applies wheel targets, and samples application state. Application is an API command switch **before the next physics step**, not a claim of instantaneous mechanical response.

The target observation interval remains 0.5 simulation seconds. The next capture target is `max(old_observation_time + 0.5, application_time)`. An early response is applied immediately and execution continues to the nominal next observation. A late response is applied immediately and the next capture uses that actual state; its observation starts a new cadence. No queued missed captures, rewinding, overlapping requests or catch-up compression. The final accepted command executes the same remaining nominal period.

The pacer measures each apply/step/log operation and sleeps only for the unused part of `dt / target_RTF`. A slow tick never causes catch-up bursts. This caps physics speed; it cannot make slow work faster. Capture uses the existing explicit Fabric/render barrier and zero simulation-time advance. PNG serialization, capture and command bookkeeping still occupy wall time; full observation→response RTF can therefore be below 1, especially for short action calls. Model loading and scene setup precede request timing.

Dynamic setup requires one stationary physics tick before bootstrap capture to register the runtime-created pedestrian rigid body in PhysX. `raw/pedestrian_initialization.json` records that tick, command, actual robot/human poses, null OLD/pending IDs, and exclusion from handoff metrics. The crossing's original simulation-time schedule is unchanged; first dynamic observation is at one tick (about 0.016667 s). Human movement during bootstrap is recorded but excluded from handoff proof.

## Timing and state definitions

All wall clocks are integer Unix nanoseconds. Durations use same-host integer monotonic nanoseconds; simulation seconds and physics ticks are separate domains. Each event includes its source:

| Event | Source / boundary |
|---|---|
| observation | Main thread, post synchronized RGB capture and pose sample, before PNG serialization finishes; carries sim time/tick and pose |
| request_submit | Main thread immediately before executor submission |
| inference_start | Isolated model service immediately before official `predict_async` |
| ready | Model service after return, CUDA synchronization, shape/finite validation and CPU copy |
| response_detected | Main thread observes completed future and samples simulator state |
| application / switch | Main thread after applying new wheel targets; state sampled before next tick |

`agent_pose_at_ready` remains **null**: no exact simultaneous simulator pose is sampled at the service-ready event. Response/application are separately sampled, not fabricated ready poses. Dynamic human event positions come from `get_rigidbody_transformation`, not authored targets. Event reads are sequential on the main thread with no intervening physics step, with their own clocks and matching sim time.

For each request, derived metrics use:

```text
action_wall_s = (ready.mono - inference_start.mono) / 1e9
observation_to_switch_wall_s = (application.mono - observation.mono) / 1e9
pending_sim_s = response_detected.sim_time - observation.sim_time
pending_physics_ticks = response_detected.tick - observation.tick
achieved_rtf = pending_sim_s / ((response_detected.mono - observation.mono) / 1e9)
robot_translation_m = norm(application.position_xyz - observation.position_xyz)
robot_yaw_change_deg = degrees(wrap(application.yaw - observation.yaw))
human_translation_m = norm(human_application.position_xyz - human_observation.position_xyz)
```

Planar XY robot translation is retained alongside XYZ. Yaw here is measured **robot** yaw, not native waypoint yaw. A 1e-6 m floating-point measurement floor is used only to distinguish nonzero transport, not as a hard-case threshold. Pending-tick validation checks all row IDs/commands/wheels and contiguous tick coverage. `physics_ticks_wholly_inside_action_call` counts rows whose main-thread step-start and post-step measurement both fall inside the service start→ready interval: stronger evidence than merely observing ticks before future detection.

## FRESH frame and controller consumption

Native chunks remain 30 × 2 `[forward,left]` meter offsets in the **observation body frame**, with no native yaw or timestamps. For derived visualization/data:

\[
p^W_j = R(\theta_{obs})p^{R_{obs}}_j + [x_{obs},y_{obs}]^T.
\]

Only `agent_pose_at_observation` anchors derived world arrays. `world_transforms.json` records the source request and transform. Ready/detection/application poses are never substituted. The unchanged controller computes its lookahead target from native offsets and applies the resulting `(v,w)` at the later switch, without transport compensation. This deliberate baseline does **not** reconcile stale geometry. The measured switch boundary is available for future study, with no graph constructed here.

## Validation and reproduction

From the repository root, with fresh run IDs:

```bash
python3 -m unittest discover -s tests -v
bash scripts/isaac6_python.sh research/sim.py \
  --config configs/research/crossing_pedestrian_continuous.yaml \
  --mode static --run-id continuous-static-NEW \
  --prerequisite-run inference-20261009-02
bash scripts/isaac6_python.sh research/sim.py \
  --config configs/research/crossing_pedestrian_continuous.yaml \
  --mode dynamic --run-id continuous-crossing-NEW \
  --prerequisite-run continuous-static-NEW
```

The CLI requires a PASS inference gate before static and a PASS **continuous static** gate before dynamic. Run creation and request writes reject reuse. Seven new pure tests cover one-outstanding behavior, OLD retention and audit tampering, bootstrap/source linkage, clocks/RTF and yaw wrapping, pacing/no catch-up, config equivalence, actual frozen-loop regression, observed-frame projection, source immutability and no overwrite. All 25 tests pass, including the earlier controller/source-parity, IPC and geometry tests. Unit fixtures are synthetic temporary files and are not experiment evidence.

Raw data remains in `raw/requests/`, `robot_state.csv`, `raw/pedestrian_state.csv`, event initialization records and diagnostics RGB. Run metadata records local/upstream SHA, dirty diff hash, code/config/launcher hashes; runtime metadata records strict checkpoint, both weight hashes and dependency identities. No model/controller source or installation was modified.

Pure NumPy/Matplotlib saved-only analysis produces durations/distances/RTF, aggregate summary, CSV/JSON, observation-anchored world arrays and timeline/transport figures under a fresh derived directory. No raw event clocks are overwritten. Rerun example:

```bash
python3 research/analyze_handoffs.py \
  --run-dir outputs/continuous-crossing-20261009-02 \
  --output-dir outputs/continuous-crossing-20261009-02/derived/handoff-NEW
```

The analyzer hashes source request arrays/JSON, RGB, runtime/metadata and CSVs before/after analysis, rejects incomplete/invalid data, excludes bootstrap, and verifies OLD targets against every pending physics row. Raw artifacts and derived outputs/figures are ignored by Git.

## Results

Starting local/remote research SHA: `15155f689359d6a14a77657c55324dd61e5683c6`; both main refs: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`, rechecked against remote before work. Runtime ran from that SHA plus the source-file hashes in each run metadata. Delivery SHA/normal push verification is in the ignored `outputs/continuous-setup-20261009/git-finalization.txt` receipt (written after commit).

**Static:** `outputs/continuous-static-20261009-01` PASS, followed by **dynamic:** `outputs/continuous-crossing-20261009-02` PASS. Each has eight real finite native `(1,30,2)` predictions, stored `(30,2)`, strict checkpoint load, one bootstrap and seven non-bootstrap handoffs. All seven handoffs in each run have OLD targets held on every pending tick, and 2–6 complete physics ticks wholly inside the actual service action-call interval. The dynamic human moves during requests 2–7; request 8 occurs after its configured 4.5 s stop and has zero human transport.

The first dynamic attempt `continuous-crossing-20261009-01` failed before any action request because PhysX had not registered the newly created human before its first physics tick. It is preserved with failure/log evidence. The explicit pre-observation initialization tick resolves this without inventing a human pose, resetting the scenario clock or changing crossing parameters. Static -01 remains valid: the only subsequent execution change is this dynamic-only initialization; the frozen path is unchanged.

Non-bootstrap aggregates below use seven requests per run. Values are **min / median / mean / max**:

| Metric | Static | Dynamic |
|---|---|---|
| Actual action-call wall latency [s] | 0.041362387 / 0.048302614 / 0.059538460 / 0.104927651 | 0.043060506 / 0.047922073 / 0.067532956 / 0.111264788 |
| Observation→switch wall latency [s] | 0.079741911 / 0.098419079 / 0.104722109 / 0.146224303 | 0.087731549 / 0.104627380 / 0.117674146 / 0.165657557 |
| Simulation advance while pending [s] | 0.050000003 / 0.066666670 / 0.073809528 / 0.116666673 | 0.050000003 / 0.066666670 / 0.083333338 / 0.133333340 |
| Robot XYZ translation [m] | 0.017490646 / 0.099942008 / 0.095302964 / 0.174897791 | 0.015142868 / 0.099952578 / 0.109252770 / 0.199890350 |
| Absolute robot yaw change [deg] | 0.000293172 / 0.000493194 / 0.008048436 / 0.052959087 | 0.000173395 / 0.000469960 / 0.019365277 / 0.132586646 |
| Achieved observation→response RTF | 0.629627075 / 0.687436290 / 0.698051151 / 0.800019979 | 0.572296359 / 0.681633471 / 0.694042412 / 0.806636785 |
| Measured human translation [m] | — | 0.000000000 / 0.066666663 / 0.066666663 / 0.133333325 |

Bootstrap is separate: static action 1.722181744 s, observation→switch 1.777587735 s, pending simulation 1.733333424 s; dynamic action 1.748881141 s, observation→switch 1.795236458 s, pending simulation 1.750000091 s. Bootstrap rest-state drift is about 1.4 mm in each run and dynamic human translation is 1.266666770 m; none contributes to handoff aggregates or PASS proof.

RTF medians 0.6874/0.6816 use the entire observation→response interval, including roughly 30 ms of post-capture PNG/IPC preparation per request and discrete polling. Pacing targets 1× for each physics tick; it does not compensate for overhead by speeding simulation afterward. These measured intervals must not be presented as exact real-time execution.

Per-request non-bootstrap evidence (action latency, observation→switch translation and human translation):

| Run | Request | Action [ms] | Pending ticks | Ticks fully inside action | Robot [m] | Human [m] |
|---|---:|---:|---:|---:|---:|---:|
| Static | 2 | 76.474 | 5 | 4 | 0.017491 | — |
| Static | 3 | 51.143 | 4 | 3 | 0.099941 | — |
| Static | 4 | 47.329 | 4 | 3 | 0.099942 | — |
| Static | 5 | 104.928 | 7 | 6 | 0.174898 | — |
| Static | 6 | 48.303 | 4 | 3 | 0.099942 | — |
| Static | 7 | 47.230 | 4 | 3 | 0.099942 | — |
| Static | 8 | 41.362 | 3 | 2 | 0.074964 | — |
| Dynamic | 2 | 73.735 | 5 | 4 | 0.015143 | 0.083333 |
| Dynamic | 3 | 47.047 | 4 | 3 | 0.099946 | 0.066667 |
| Dynamic | 4 | 43.061 | 3 | 2 | 0.074967 | 0.050000 |
| Dynamic | 5 | 111.265 | 8 | 6 | 0.199890 | 0.133333 |
| Dynamic | 6 | 47.922 | 4 | 3 | 0.099956 | 0.066667 |
| Dynamic | 7 | 44.139 | 4 | 2 | 0.099953 | 0.066667 |
| Dynamic | 8 | 105.563 | 7 | 6 | 0.174914 | 0.000000 |

Independent scalar recomputation agrees with saved metrics within 1e-12. Measured robot movement between the first/last physics samples wholly inside the service action call is nonzero for all 14 non-bootstrap requests; dynamic request 5 alone has 0.124932595 m between those samples. All dynamic event human positions exactly match the measured PhysX CSV at the same simulation tick. The protected 175-file manifest (prior frozen runs/geometry output, original frozen config/loop/service/controller, geometry report, selected Isaac installation files) remains unchanged.

Generated evidence: raw run folders above; derived CSV/JSON/world arrays and PNG/PDF plots under `derived/handoff/`. Static `derived/handoff-v2/` preserves the same numbers with a corrected timeline legend layout; the original derived folder is preserved. Dynamic `derived/handoff/` already has the corrected layout. The independent audit is `outputs/continuous-setup-20261009/independent-audit.json`.

### Largest measured non-bootstrap transport example

Dynamic request **5**, using OLD source request **4**. Eight pending physics ticks, six wholly inside the real action call; `(v,w)=(1.5, +0.0249294015)` m/s, rad/s; FL/RL/FR/RR targets `(15.2583508473, 15.2583508473, 15.3538940506, 15.3538940506)` rad/s.

| Event | Simulation seconds | Unix wall ns | Monotonic ns |
|---|---:|---:|---:|
| observation | 3.266666837037 | 1791533605939713172 | 114802284162927 |
| request_submit | unobserved in service / not sampled | 1791533605971009682 | 114802315459217 |
| inference_start | unobserved in service / not sampled | 1791533605977236345 | 114802321686581 |
| ready | unobserved in service / not sampled | 1791533606088501244 | 114802432951369 |
| response_detected | 3.400000177324 | 1791533606105008531 | 114802449458315 |
| application | 3.400000177324 | 1791533606105370930 | 114802449820484 |

Robot observation pose `(x,y,yaw)` is `(1.954076767, -0.014063299, -0.007358119)`; application pose is `(2.153961658, -0.015540498, -0.007349225)` in meters/radians. Translation is **0.199890350 m**, wrapped yaw change **+0.000509586 deg**. Actual action latency is **0.111264788 s**, observation→switch wall latency **0.165657557 s**, and pending simulation advance **0.133333340 s**.

Measured human observation XYZ is `(2.5, -0.766666830, 0)`; application XYZ is `(2.5, -0.900000155, 0)`, translation **0.133333325 m**. Detection and application share simulation tick 204, but their actual wall clocks differ. No exact service-ready robot pose is claimed.

## Action availability versus background reasoning

`reasoning_new_start_step`, `reasoning_completion_poll_step`, `kv_cache_available`, `delay_seconds_input`, delay reference and sampled image IDs are preserved from the original service. The exact reasoning worker-ready wall timestamp stays null. Completion poll step is when a completed worker result was detected, not its exact completion time. `delay_seconds_input` is a simulation-time age relative to the adapter's oldest retained reasoning-start reference, not the measured wall duration of this action call. Bootstrap may wait for initial reasoning; subsequent actions can return quickly using available KV state while reasoning proceeds independently. No semantic-worker duration is inferred from a fast or slow action call.

For example, dynamic request 5 has action-call latency **0.111264788 wall s**, `delay_seconds_input=3.2500001695` **simulation s**, new reasoning start step 196, completion poll step 196, and KV available. Requests 3/4 have no new start step and retain completion poll step 1; requests 6/7 retain poll step 196. Those discrete observations preserve background reasoning state without assigning an invented worker completion timestamp or calling 3.25 s the FRESH action latency.

## Limitations and decisions

This is one easy corridor/crossing scenario, eight predictions per validated run (seven non-bootstrap handoffs), one RTX 5060 Ti shared by renderer/physics/model, and one configuration. Real-time factor is measured, not assumed exactly 1; synchronous image acquisition and discrete polling remain. The human is a scripted kinematic rest-pose mesh with measured PhysX translation, no gait/crowd intelligence. Its path progresses during bootstrap and stops at the original 4.5 s.

There is no artificial model latency, stronger scenario, latency sweep, hard-case score/threshold, correspondence algorithm, smoothing, reconciliation or graph optimization. No pedestrian-driven hard-case or navigation-performance claim follows from transport alone. Prior frozen evidence remains separate.

**Q1: CONTINUOUS HANDOFF VALIDATED.** Static and dynamic non-bootstrap requests prove one outstanding inference, continued measured physics, and retained OLD commands.

**Q2: MEASURABLE ACTUAL-LATENCY TRANSPORT.** Non-bootstrap robot motion is measured during actual action calls and before switch; the dynamic human also moves during six such handoffs. This establishes the execution condition for a later study, not that this easy scene is a hard case or that reconciliation is necessary.
