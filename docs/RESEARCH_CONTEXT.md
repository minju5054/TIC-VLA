# Research context and scope

## Main research

The current research repository is this [TIC-VLA fork](https://github.com/minju5054/TIC-VLA), tracking [official TIC-VLA](https://github.com/ucla-mobility/TIC-VLA). It hosts TIC-VLA reproduction, DynaNav experiments, future successive action chunk collection, hard-case analysis, and the future reconciliation method.

[minju5054/se-3-reconciliation](https://github.com/minju5054/se-3-reconciliation) contains earlier **LightNav-based** reconciliation research. It is not the destination for this fork's future method implementation and is not modified by this task.

The research asks whether explicit SE(2) reconciliation can reduce geometric and temporal inconsistency between successive navigation trajectory/action chunks while preserving new navigation intent. OLD is the preceding prediction still influencing execution; FRESH is a newer prediction from a more recent observation. The eventual method is not implemented in this stage.

TIC-VLA is the **upstream navigation VLA**, not a competing reconciliation-method baseline. The current platform is a controlled Isaac 6 environment with a four-wheel Jackal and one scripted human. It reuses official TIC-VLA inference and Nova Carter's lookahead formula; it does not reproduce or port the DynaNav benchmark. Keep research tooling and any future method separate from official implementation.

## Current and future stages

Current scope:

```text
official TIC-VLA reproduction
    -> Isaac 6.0.1 runtime/API compatibility investigation
    -> separate Isaac 6 simulation / Isaac 5 Python inference
    -> front RGB -> real native prediction -> four-wheel physics
    -> successive raw records in one controlled crossing scenario
```

Only on a separate explicit request:

```text
OLD/FRESH correspondence and hard-case scanning
    -> qualified handoff selection
    -> reconciliation research in this TIC-VLA fork
```

The user explicitly authorized minimal append-only raw prediction, event, image and robot-state records to validate this closed loop. No severity metric, OLD/FRESH correspondence, SE(2) trajectory conversion, graph optimization, reconciliation, training, fine-tuning, or navigation-performance benchmark is implemented.

Current execution status: **READY FOR HARD-CASE COLLECTION** within the controlled sequential simulation scope; see [MINIMAL_CLOSED_LOOP.md](MINIMAL_CLOSED_LOOP.md). The platform produces successive real predictions, four-wheel motion and measured scripted human motion. Physics pauses during inference, and the human translates in its authored rest pose. These are explicit scope limits; no hard case has been established. The unmodified DynaNav/legacy human-system blockers in [ISAAC6_COMPATIBILITY.md](ISAAC6_COMPATIBILITY.md) remain historical facts; neither Isaac installation was changed.

The separately authorized continuous path is now **CONTINUOUS HANDOFF VALIDATED** with **MEASURABLE ACTUAL-LATENCY TRANSPORT**; see [CONTINUOUS_HANDOFF.md](CONTINUOUS_HANDOFF.md). It preserves the default frozen config and uses a separate config, one outstanding IPC future, main-thread physics, OLD command retention and measured clocks/poses. Static and dynamic runs each contain seven non-bootstrap handoffs; dynamic robot transport median/max is .099953/.199890 m. This is execution infrastructure evidence, not a pedestrian hard case, navigation result or reconciliation validation. [CHUNK_GEOMETRY_ANALYSIS.md](CHUNK_GEOMETRY_ANALYSIS.md) remains the separate saved frozen-run geometry result.

## Required distinctions

```text
TIC-VLA performance validation != reconciliation mechanism validation
TIC-VLA navigation failure != OLD/FRESH handoff inconsistency
raw TIC-VLA output != derived SE(2) trajectory
hard-case discovery != graph optimization validation
simulation evidence != real-world evidence
```

A successful runtime smoke test is only evidence that the execution path works. It does not demonstrate that qualifying hard cases exist. Static observations about asynchronous reasoning or command replacement are hypotheses about collection opportunities, not measured inconsistency.

## Implications for subsequent collection

Read [ACTION_SEMANTICS.md](ACTION_SEMANTICS.md) before designing a logger. The default native output is 30 two-dimensional future position offsets in a common current ego frame. The Nova Carter controller derives a velocity command from a lookahead point; it does not commit an entire timed waypoint queue. The OLD/FRESH definition therefore needs separate native prediction and controller execution records. VLM reasoning generation and action-head prediction have different lifecycles.

Do not fabricate timestamps that are missing upstream. Do not derive yaw and label it as observed. Any later derived SE(2) representation requires an explicitly documented transform and yaw rule, with raw output preserved separately.

## Upstream metadata audit and future schema

The availability column below describes the inspected upstream source. The new minimal harness implements a subset with exact field mappings in [MINIMAL_CLOSED_LOOP.md](MINIMAL_CLOSED_LOOP.md); it does not retroactively change upstream availability. `null` means unknown/unobserved, never zero by default.

| Proposed field | Availability and future collection requirement |
|---|---|
| `episode_id`, `run_id` | Existing `TICVLA_EPISODE_NAME` and `BENCHMARK_RUN_ID`; persist explicitly. |
| `request_id` | Absent; introduce an action prediction sequence ID separately from a reasoning generation ID. |
| `observation_time` | Saved front-image filename contains benchmark frame count. No sensor exposure timestamp or monotonic capture timestamp is retained. Frame/30 is an upstream convention, not measured latency. |
| `inference_start_time` | Action call boundary is observable at `NovaCarterTICVLA._inference_task`; no per-action timestamp is stored. Reasoning submission has `_kv_cache_start_time = perf_counter()` and `_kv_cache_generation_step`. Submission is not actual worker-start time. |
| `ready_time` | Action tensor exists after `predict_async`; no per-action ready timestamp is stored. Reasoning `_kv_cache_completion_step` records completion detection at poll/wait, not exact worker or GPU completion time. |
| `execution_or_switch_time` | Command assignment and `apply_wheel_actions` boundaries exist; no request-linked switch event or exact physical execution timestamp is recorded. Must distinguish requested command from measured actuation. |
| `agent_pose_at_observation` | Current pose is sampled before inference; image capture and pose reads are sequential and not guaranteed sensor-synchronized. Preserve both sample identities. |
| `agent_pose_at_ready` | Not captured as a dedicated request-linked pose. A later debug pose read is not an authoritative ready-time sample. |
| `agent_pose_at_switch` | Not captured as a dedicated request-linked pose. Existing periodic robot logs are insufficient to assume it. |
| `raw_action_chunk`, `raw_action_shape`, `raw_action_dtype` | Tensor available on return from `predict_async` and before `waypoints[0].float().cpu().numpy()`; not persisted upstream. Preserve native tensor values/dtype before controller conversion. |
| `raw_action_semantics`, `raw_action_frame`, `raw_action_units` | Semantic declarations must be versioned; nominal current ego FLU offsets in meters, with frame caveats in ACTION_SEMANTICS. |
| `instruction` | Existing environment/config and behavior state. |
| `rgb_observation_reference` | Temporary front-image paths exist but upstream prunes/deletes them. A future collector must archive referenced frames without changing model input. |
| `reasoning_generation_id`, `reasoning_input_frames` | No durable IDs; generation start step/pose and image list exist transiently. Track separately from action requests. |
| `reasoning_submit_time`, `reasoning_worker_ready_time`, `reasoning_poll_time` | Submit clock exists transiently; actual worker completion clock absent; poll frame exists. Do not collapse into one timestamp. |
| `reasoning_state_used_id`, `delay_input`, `delay_reference_pose` | Latest KV state is used; behavior computes delay/displacement from oldest of up to two tracked starts. Association needs verification, especially bootstrap/failure transitions. |
| `controller_command`, `controller_mode`, `command_applied` | `_current_action`, filtered `_cmd_v/_cmd_w`, backup/wait flags, and wheel action call are observable. Periodic robot-state JSON contains some commands/poses but no prediction association. |
| `clock_domain`, `benchmark_frame`, `timeline_time`, `monotonic_ns` | Frame exists; timeline time/API and monotonic clocks would require new observation. Record separate domains and any mapping. |
| `upstream_git_sha`, `local_git_sha`, `dirty_diff_hash` | Available through Git; setup wrapper records run provenance. Upstream behavior does not automatically attach these to predictions. |
| `checkpoint_id`, `base_model_revision`, `environment_id` | Available from setup manifest/version snapshot; not automatically prediction-linked. |

The harness's raw records validate execution and preserve future analysis inputs. They do not establish an OLD/FRESH correspondence or demonstrate a hard case. Upstream periodic robot logs alone do not constitute such a dataset either.
