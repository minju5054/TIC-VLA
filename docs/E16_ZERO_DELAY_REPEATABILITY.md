# Hospital E16 zero-delay navigation repeatability

## Question and frozen protocol

This study asks whether the Hospital E16 corner failure occurs in ordinary continuous TIC-VLA execution without the measurement hook, and whether three zero-hook trials differ descriptively from three no-hook trials. It does not implement graph optimization, correction, smoothing or a positive-delay sweep.

Starting local/fetched origin SHA: `6339bd24d17a9de12bb7089c53bdc332686bec43` on `hardcase-probe`; upstream/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. All existing source is unchanged. Additions are an experiment runner, protocol JSON, saved-only analyzer/plotter and tests. In particular, `research/continuous.py`, `research/late_handoff.py`, `research/control.py`, inference service and `DynaNav/` remain byte-identical to the starting revision.

The exact successful config is copied from `outputs/nova-e16-hospital-lights-baseline-20261010-01`. Recursive config difference is **[]** between every reference/new run and both conditions, including instruction, seed36, controller, start/goal, camera/lighting, scene, model and preprocessing. Simulator: Isaac6.0.1 controlled Hospital wrapper, Nova Carter, continuous OLD execution,60Hz physics/control,2Hz target inference,48 predictions. Model: existing separate Isaac5 Python process, no Isaac5 SimulationApp. This is not an unchanged official DynaNav benchmark. No cart, human, new obstacle, package, driver or global environment change.

Prespecified new-run order: **NO_HOOK, ZERO_HOOK, ZERO_HOOK, NO_HOOK, NO_HOOK, ZERO_HOOK**. Each run creates a fresh SimulationApp and a fresh inference service/model, with empty initial history/reasoning state and the original start pose. Seed36 is preserved, not represented as deterministic replay. Runtime/inference/contract failure stops later trials; navigation failure does not. Six exclusive IDs, no retries or replacement runs. Historical A/B and previous zero1/zero2 are references only, excluded from condition counts.

`research/e16_zero_repeatability.py` passes `application_hook=None` for NO_HOOK and the unmodified `ApplicationHold(...,0.0,...)` for ZERO_HOOK. No instrumentation is added to model, controller or loop. Both conditions use the same saved-data validation after navigation. Per-run host records identify GPU UUID/model/driver/kernel and simulator PID; the post-navigation child-process snapshot records the independent inference service PID while it is still alive. Bootstrap request checks include tick0, sampled IDs `[1]`, no delay reference and first reasoning start step0.

Plan: `outputs/e16-zero-repeatability-plan-20261011-01/receipt.json`. It binds config, exact runtime sources, user instruction, four reference directories, checkpoint weights, startup/test gates and fixed analysis rules. Setup: `outputs/e16-zero-repeatability-setup-20261011-01/`. Simulator-only startup PASS;22 tests PASS before any model execution. Original collider/no-actor, wheel/camera/config checks run before each trial's inference.

## Measurement conventions

The frozen spatial selection is unchanged: first non-bootstrap north-facing/north-moving observation east of the original exit plane, within0.75m south of original corner-entry Y, before any measured entry at that observation. The original bounds are `2.5521036832844546 < X <= 7.986982542195387` and `8.707836118855571 <= Y < 9.457836118855571` metres. NO_HOOK selection is saved-only; it does not invoke a runtime hook. ZERO_HOOK selection must match that independent saved-only rule, with unchanged native tensor/command and zero additional physics ticks. Missing selection remains explicit.

Original actual-left-motion, corner entry/passage, nonfloor PhysX contact and strict2m goal-region gates are reused. Fixed48 predictions do not validate stopping at the goal. Northward overshoot uses the previous frozen corner window: first dense crossing of trigger-south Y through passage, first contact or end, whichever is earliest; `max(0,maxY-original center_y_max)`. Overshoot alone is not a navigation failure. Contact prims/points are actual PhysX evidence, not a geometry proxy.

Native predictions remain30 forward/left positions in metres at their own observation body origin, with no native yaw/timestamps. Own-observation world projection is `p_W = t_obs + R(yaw_obs)p_native`; yaw is CCW from world+X, Z up, saved quaternions wxyz. World predictions and measured execution remain separate. Same-index native curves across different runs are not frame-invariant revision metrics.

All request records export native/RGB source paths and hashes, observation/application poses, original lookahead/index, reconstructed(v,w)/filter, next-tick measured velocity response, previous motion prompt, sampled image IDs and reasoning fields. The raw CSV application tick still carries the previously active command; the next physics tick measures FRESH response.

Timing separates observation, submit, inference start, action ready, response detection and application using same-host monotonic differences. **Detection-to-application wall duration is an upper-bound diagnostic, not exact hook cost**: it includes response consumption, validation, controller computation, optional hook, wheel target application and state capture. Selected ZERO_HOOK received/hold logging is included, but this interval does not measure all per-tick callback overhead or background reasoning timing. Main distributions contain47 nonbootstrap handoffs per run; bootstrap is exported separately. Action wall latency, simulation pending time and measured transport are distinct.

Saved reasoning generation-start step, completion-*poll* step, KV availability, text, sampled image IDs, delay reference and model state inputs are preserved. Poll steps are mapped to the observing request when exact. A new generation-start field is not the identity of the KV cache consumed by that action. Returned reasoning text/KV availability does not establish exact KV tensor equality. Worker-ready wall time remains null; no background completion clock is fabricated or equated with action-ready time.

## Divergence definitions fixed before the runs

Progress comparisons use precontact northbound observations east of the original exit in1m world-Y bins `[2,3),…,[12,13)`. Each bin reports request IDs, median angular command and lookahead-left. The representative observation is nearest the bin midpoint, ties by request ID; actual XY/yaw separation is recorded. Bins do not guarantee identical images, states or histories. No waypoint matching or automatic correspondence is performed.

The descriptive persistent steering-divergence rule is the first **two adjacent occupied common bins** with differing signs of both median w and median lookahead-left, using existing Hospital numerical sign tolerances. This is neither a hard-case threshold nor a causal/statistically significant onset. The first numerical RGB/native inequality is reported separately. Initial native RMSE is called same-pose only when initial observation SE(2) poses are exactly equal. C12–C18 is a separately labeled descriptive appendix; equal request IDs are never assumed to be identical locations.

All15 primary-run pairs plus comparisons with four references are retained; no successful or failed run is discarded. The displayed primary success/failure example, when one exists, is the earliest defined two-bin disagreement (ties by run order), not the largest error. If no pair meets the rule, the diagnostic remains undefined. Original RGBs and native files remain traceable through request IDs and exact source paths.

## Results: CASE 3 — all six succeed, paths differ

Exactly six fresh runs were executed, in ABBAAB order; no retry, extra run or positive delay. Every run has48 finite predictions and passes the runtime/inference/continuous contract. **NO_HOOK:3 successful corners/0 failures. ZERO_HOOK:3/0.** All six enter the goal region and have no nonfloor contact. These are observed counts, not population success probabilities. The previous failed zero trial is still a failed reference, not included in either count.

Times below are simulation seconds. Overshoot is relative to the unchanged original corner north bound11.741243471m; it is zero in all six despite differing maximumY. A corner-passage time and a goal-entry time do not imply the robot stopped there.

| Run | Condition | Corner | Entry s | Actual-left start s | Passage s | Max corner Y m | Overshoot m | Goal entry s |
|---|---|---|---:|---:|---:|---:|---:|---:|
| R1 | NO_HOOK | PASS | 7.700000 | 7.900000 | 10.500001 | 11.212600 | 0.000000 | 18.516668 |
| R2 | ZERO_HOOK | PASS | 7.683334 | 7.333334 | 10.933334 | 10.983150 | 0.000000 | 18.483334 |
| R3 | ZERO_HOOK | PASS | 7.533334 | 8.833334 | 10.933334 | 11.625122 | 0.000000 | 19.083334 |
| R4 | NO_HOOK | PASS | 7.633334 | 7.850000 | 10.933334 | 11.330015 | 0.000000 | 18.766668 |
| R5 | NO_HOOK | PASS | 7.683334 | 7.833334 | 10.433334 | 10.893250 | 0.000000 | 18.400001 |
| R6 | ZERO_HOOK | PASS | 7.566667 | 8.316667 | 10.916667 | 11.345761 | 0.000000 | 18.750001 |

| Run | Corner-entry world XY m | Actual-left-start world XY m | Final world XY m | Final yaw rad |
|---|---|---|---|---:|
| R1 | (5.153223, 9.458896) | (4.964316, 9.691821) | (-20.638845, 10.378934) | 2.976215 |
| R2 | (5.480891, 9.464518) | (5.857618, 9.099455) | (-20.519405, 10.048765) | 2.938389 |
| R3 | (5.736916, 9.466520) | (4.615348, 11.038898) | (-19.703835, 10.218426) | 2.978050 |
| R4 | (5.628850, 9.472148) | (5.412070, 9.714183) | (-20.199997, 10.193282) | 2.936522 |
| R5 | (5.239832, 9.460056) | (5.083923, 9.622350) | (-20.694775, 10.204474) | 2.957107 |
| R6 | (5.629695, 9.461810) | (4.896840, 10.309686) | (-20.138983, 10.476637) | 2.938532 |

The six new runs have **no contact position or colliding prim**. For context only, previous zero1 contacted `/Root/Geo_M2_DoorWall2Corner2_656/Geo_M2_DoorWall2Corner/Geo_M2_DoorWall2Corner/Section1` against `/World/Robots/Nova_Carter/wheel_right/Cylinder_03` at9.550000498s. RobotXY=(4.095304,12.124000)m, yaw2.065245rad; PhysX reported point=(4.219769,12.343957,.140450)m. It never passed the original west-hallway gate or goal. Its .382756m overshoot remains a historical observation.

## Hook and timing results

All six select **C13** by the unchanged spatial rule. For NO_HOOK this selection is derived after execution; no hook record/directory exists. All three ZERO_HOOK runs actually trigger once, preserve the received native array and original command, and realize **0 additional ticks**. Detection/application ticks are equal for every one of288 requests. This does not remove genuine observation-to-application transport under continuous OLD execution.

| Run | Action wall median / P95 ms | Observation→application wall median / P95 ms | Detection→application median ms | Selected C13 detection→application ms |
|---|---:|---:|---:|---:|
| R1_NO_HOOK | 56.826 / 114.823 | 531.344 / 601.496 | 0.279948 | 0.271211 |
| R2_ZERO_HOOK | 57.460 / 121.208 | 531.660 / 593.864 | 0.288716 | 1.668034 |
| R3_ZERO_HOOK | 56.020 / 114.895 | 536.210 / 575.570 | 0.308584 | 1.431941 |
| R4_NO_HOOK | 54.783 / 88.532 | 532.721 / 571.950 | 0.287415 | 0.264872 |
| R5_NO_HOOK | 56.222 / 108.641 | 531.783 / 564.999 | 0.293387 | 0.628813 |
| R6_ZERO_HOOK | 55.514 / 120.019 | 533.470 / 587.814 | 0.290000 | 1.527250 |

Selected-point NO_HOOK wall durations are .264872–.628813ms, versus1.431941–1.668034ms for ZERO_HOOK. The medians across three selected observations are .271211 and1.527250ms (difference1.256039ms). This is **descriptive evidence of a larger measured processing interval at the logging hook**, consistent with extra hook/logging work; it is not isolated hook cost, a randomized same-input estimate, or evidence that it causes failure. The previous failed zero1 had9.656738ms at this interval, versus1.449495ms in previous successful zero2; those two references cannot identify a causal effect either.

Per-run nonbootstrap action medians54.783–57.460ms and observation→application medians531.344–536.210ms overlap between conditions. Every run's median pending simulation duration is.100000005s and median measured XY transport is approximately.15010m. P95/max and per-request values, including submit/start/ready/detection clocks, are exported. Bootstrap is slower: action1.787–1.863wall s, observation→application2.257–2.349wall s, application at1.816667–1.900000sim s. Later observation schedules therefore differ before the C13 trigger. Wall time is not simulation time.

## Native prediction, controller and measured response

Every new request's original controller was reconstructed from native data and the prior yaw-filter state, without a source change; all v=1.5m/s. No approach request in the frozen precontact northbound Y∈[2,13)m window has **both negative native lookahead-left and negative applied w**. In particular, all C12–C18 w values are positive. Small isolated early native-left negatives can still produce positive commands through the original filter; neither negative components elsewhere nor later hallway corrections are classified as corner failure.

The following is a **same-index descriptive appendix only**. Entries are applied w[rad/s]; physical positions, native lookahead points, filter states and velocities for all42 rows are in `c12_c18_descriptive.csv` alongside clearly labeled references.

| Run | C12 | C13 | C14 | C15 | C16 | C17 | C18 |
|---|---:|---:|---:|---:|---:|---:|---:|
| R1_NO_HOOK | +0.212224 | +0.370598 | +0.324718 | +0.320267 | +0.349222 | +0.279705 | +0.155496 |
| R2_ZERO_HOOK | +0.407008 | +0.349133 | +0.221254 | +0.207238 | +0.235526 | +0.166994 | +0.101660 |
| R3_ZERO_HOOK | +0.123486 | +0.081920 | +0.321249 | +0.664848 | +0.477998 | +0.385826 | +0.283954 |
| R4_NO_HOOK | +0.203523 | +0.343550 | +0.315018 | +0.265617 | +0.239413 | +0.228703 | +0.188047 |
| R5_NO_HOOK | +0.201625 | +0.256608 | +0.448626 | +0.356567 | +0.215692 | +0.178177 | +0.152065 |
| R6_ZERO_HOOK | +0.155573 | +0.287812 | +0.460459 | +0.394340 | +0.263420 | +0.241963 | +0.191822 |

Physical-progress comparison below uses the frozen nearest-bin-midpoint representative, without changing bin/selection rules after outcomes. Cells show `C#, observationY, native lookahead-left, applied w` in metres/rad/s. The Y∈[9,10)m representative of R3 is C13, while others are C14; in[10,11)m it spans C15–C17. These remain different XY/yaw/history states, not identical observations. Full XY/yaw separation is in `divergence.json`.

| Run | Y∈[9,10)m | Y∈[10,11)m |
|---|---|---|
| R1_NO_HOOK | C14, 9.6918, +0.170898, +0.324718 | C16, 10.6256, +0.185547, +0.349222 |
| R2_ZERO_HOOK | C14, 9.6055, +0.098633, +0.221254 | C17, 10.6079, +0.072754, +0.166994 |
| R3_ZERO_HOOK | C13, 9.1971, +0.036377, +0.081920 | C15, 10.5040, +0.365234, +0.664848 |
| R4_NO_HOOK | C14, 9.7142, +0.170898, +0.315018 | C16, 10.6028, +0.120117, +0.239413 |
| R5_NO_HOOK | C14, 9.6223, +0.227539, +0.448626 | C16, 10.4352, +0.087402, +0.215692 |
| R6_ZERO_HOOK | C14, 9.7673, +0.347656, +0.460459 | C15, 10.3097, +0.193359, +0.394340 |
| Prev_zero_1 | C14, 9.8130, -0.052246, -0.048176 | C15, 10.4611, -0.103516, -0.137362 |

The prior failure's C14/C15 remain the first adverse corner sequence: native lookahead-left−.052246/−.103516m, applied w−.048176/−.137362rad/s at7.9/8.4s. Observations were Y9.813034/10.461102m, not the same states as new C14/C15. After C14, next-tick yaw rate is still+.006128rad/s; the first measured negative sample is7.933334s,−.039710rad/s. During its active interval median yaw rate is−.071332rad/s. C15 interval median is−.149234rad/s. This is a genuine executed rightward response to the original native/controller output, not merely a plotting sign or modified controller. It is absent from the new matched-progress corner approach. No proof of which upstream input caused the old native output follows from this sequence.

## Earliest divergence and reasoning/history

All15 pairs of new runs have **exactly identical C1 observation SE(2)**, empty/equal initial motion history and equal saved initial model-state inputs. Nevertheless, their saved C1 RGB pixels differ: MAE2.101438–2.262112 on a0–255 scale;85.74–87.36% of pixels have at least one unequal channel. This is small-amplitude pixel inequality across a large image fraction, not a major scene change. Native C1 is already nonidentical in every pair: same-pose pointwise RMSE.013134–.063258m. All six C1 reasoning texts are distinct; C1 w spans+.028439 to+.204098rad/s. These differences occur before any application of the selected C13 hold. Within NO_HOOK alone, C1 RGB/native/text differences also exist.

Saved observation poses first become numerically unequal at C2 for12 pairs, C3 for3 pairs. This is a same-index descriptive clock/state check, not a cross-run trajectory correspondence. Bootstrap application time differs; motion/history/observations subsequently diverge. The archived PNG comparison does not establish byte equality of downstream preprocessed tensors, exact KV state, deterministic GPU arithmetic, or a causal renderer→prediction→failure chain.

**Meaningful primary success/failure divergence is undefined**: there is no new failed run, and no new pair satisfies the frozen two-adjacent-bin directional rule. The diagnostic figure keeps that result and shows C1 numeric differences for the first two runs by protocol order. Even the previous failure does not satisfy the two-bin median rule: positive C13 and negative C14 share[9,10)m, whose median stays positive; only[10,11)m has a negative median. The raw C14/C15 sequence is retained explicitly. We did not relax the rule to manufacture an onset. Undefined under this coarse rule does not imply identical paths or absence of the historical local failure sequence.

All288 new actions report KV available; exact worker-ready timestamps remain null. Six fresh process histories begin with sampled IDs[1], reasoning start step0 and no delay reference. Reasoning starts follow C1,C2,C4,C6,… through the corner in all six and in the old failure. C14 completion-poll sim ticks differ (R1 474, R2/R3/R5 470, R4 471, R6 469; previous failure468), reflecting differing schedules; this is not a measured VLM completion wall clock. Within each run C14/C15 share the most recently polled text, and sampled image IDs are[2,8,14]/[3,9,15], but the corresponding images and previous-motion/state inputs differ. New whole-run distinct-text counts are24,23,24,23,24,23; R4's late lifecycle slips to a new start atC47. These later differences do not establish a precontact cause.

The previous failed C14/C15 text says it has completed the left turn and should “align slightly right”; its numeric reasoning answer also bends right. This is consistent with the native rightward output but **does not identify reasoning as the cause**. Successful new R2 and R5 also say “align slightly right” at C14 while producing positive native lookahead/controller output, and successful R1/R4 likewise claim the turn is complete. Text wording alone is therefore not a failure classifier or reliable causal explanation. Those are model-generated descriptions, not measured scene/robot facts; the untouched Hospital contains built-in beds/carts even though no experiment cart/human was added.

## Interpretation and next single task

Observed classification: **CASE 3**. Ordinary NO_HOOK execution has measurable RGB/native/reasoning/path variation even before the late hook's selected event, but **no-hook corner failure was not reproduced in these three trials**. ZERO_HOOK also has no new failure. The selected wall interval is longer with ZERO_HOOK, while additional physics ticks/native/controller changes are zero and the navigation outcomes are the same. A causal zero-hook contribution to the previous failure remains **unresolved**, not excluded or established.

Six runs, one scene and one host cannot determine condition-specific failure probability, statistical hook effects, deterministic model behavior or navigation-performance generalization. Progress bins retain differences in X/yaw/images/history. KV contents and exact reasoning-ready clocks are unavailable. An observed replanning/navigation error is not evidence that graph reconciliation is necessary or effective. No smoothing, graph optimization, correction, new obstacle, tuning, additional sampling or delay sweep was implemented.

**Recommended next single research task (not executed): fixed saved-C1 input repeatability.** Reuse one byte-identical saved C1 RGB with the same instruction, initial state/history and seed across fresh inference processes, with no simulator or application hook, to test whether bootstrap native/reasoning variation persists when rendered-input variation is removed. This would isolate an input/model-repeatability question; it would not replay later unknown KV state or by itself establish the origin of the corner failure. Keep the late-delay sweep paused.

## Artifacts, validation and reproduction

Raw directories (six independent process runs, exactly48 requests each):

- `outputs/nova-e16-zero-repeat-20261011-01-r01-no_hook`
- `outputs/nova-e16-zero-repeat-20261011-01-r02-zero_hook`
- `outputs/nova-e16-zero-repeat-20261011-01-r03-zero_hook`
- `outputs/nova-e16-zero-repeat-20261011-01-r04-no_hook`
- `outputs/nova-e16-zero-repeat-20261011-01-r05-no_hook`
- `outputs/nova-e16-zero-repeat-20261011-01-r06-zero_hook`

Final derived directory: `outputs/e16-zero-repeatability-analysis-20261011-02/`; initial`…-01` preserved. Both contain identical numeric summaries. -02 replaces the blank undefined-onset panel with traceable initial RGB/native diagnostics, while retaining the undefined primary onset and frozen rules. Raw data was not rerun or altered.

The derived directory contains `summary.json`, `run_metrics.csv`, `divergence.json` (all45 pairs:15 primary and30 involving references), `earliest_divergence.json`, `c12_c18_descriptive.csv`, full per-run request JSON/CSV, physical-progress bins, navigation JSON and dense measured robot CSVs. Every native/RGB reference points to an original request with SHA256. `metadata.json` binds source runs, plan, config and analyzer/plotter hashes and verifies source immutability.

Required figures under `figures/`:

- `six_run_world_xy.png`
- `corner_comparison.png`
- `angular_command_vs_progress.png`
- `measured_yaw_and_velocity.png`
- `fresh_native_progress_comparison.png` (new successes versus explicitly labeled **previous** failed reference; no new failure invented)
- `timing_distributions.png`
- `navigation_outcomes.png`
- `earliest_divergence_diagnostic.png`

Supplemental: `bootstrap_rgb.png`, `reasoning_lifecycle.png`. All10 figures visually reviewed. No generated figure/raw output is committed.

Before execution: simulator-only startup PASS;22 focused tests PASS, exact configs/checkpoint hashes and prior saved contracts PASS. Before the final two trials:24 focused tests PASS including two saved-analysis diagnostic tests. No model-time policy/code change was made. Independent post-run checks validate all288 finite native arrays, original controller/filter/index values,576 observation/application pose matches,288 full-HD RGBs, six distinct simulator PIDs and six distinct inference launcher PIDs. Every run has an unchanged no-actor scene/collider inventory and exact runtime config diff[]. All reference/weight/frozen code/analysis-source hashes remain unchanged; receipts are in `outputs/e16-zero-repeatability-setup-20261011-01/` (`final-verification.json`, startup/tests/per-run logs and reviews).

Host in all six: NVIDIA GeForce RTX5060Ti, UUID`GPU-2adc3788-846a-f8fe-3a44-e5542c653286`, driver595.99.02, kernel7.0.0-38-generic. This is a common recorded host fingerprint, not identical GPU utilization/frequency/scheduling. Isaac6 Python3.12.13/NumPy2.3.1; separate Isaac5 Python3.11.13/NumPy1.26.0, torch2.7.0+cu128, torchvision0.22.0+cu128, transformers4.57.6. Base SHA256`a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`; checkpoint`376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`. Both artifact hashes match in every run; checkpoint loading is strict. Full original command/config/provenance is stored per run; `.git` HEAD during execution is the starting SHA, with new files and exact source hashes recorded as dirty work.

Executed run command (index0…5 separately, in order; existing IDs now reject reruns):

```bash
bash scripts/isaac6_python.sh research/e16_zero_repeatability.py --phase run --receipt-dir outputs/e16-zero-repeatability-plan-20261011-01 --index 0
```

Saved-only analysis (choose an unused output directory):

```bash
MPLCONFIGDIR=/tmp/ticvla-mpl python3 research/analyze_e16_repeatability.py --plan outputs/e16-zero-repeatability-plan-20261011-01 --output-dir outputs/e16-zero-repeatability-analysis-NEW
python3 -m unittest tests.test_e16_zero_repeatability tests.test_e16_repeatability_analysis tests.test_late_handoff tests.test_continuous -v
```

Delivery: focused code/config/tests/report plus append-only work-log commit, normal push to`origin hardcase-probe`; exact commit/remote/clean status recorded after commit in setup`git-finalization.json`. No force push or history rewrite.
