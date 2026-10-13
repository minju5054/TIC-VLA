**TIC-VLA H05 offline RGB-sequence comparison — 2026-10-13**

**NO_CLEAR_DIRECTIONAL_RESPONSE / PREVISIBLE_DIVERGENCE / REPLAY_VARIABILITY_UNRESOLVED.** The paired experiment completed: two independently initialized model services, 14 requests each, 28 new inferences, zero navigation trials, physics steps, command applications or new renders. At the primary C12–C14 observations, both ABSENT and PRESENT predict LEFT native lookahead and LEFT counterfactual angular commands. The previous failed H05 trial’s left-to-right transition was not reproduced at these successful-baseline poses. Numerical curve differences remain; this does not establish an absence of RGB influence.

The first raw native-left sign disagreement is **C5**: A −0.00753784 m, B +0.02819824 m. A is within the preregistered ±0.02 m near-zero band; no opposing LEFT/RIGHT pair outside the native ±0.02 m or angular ±0.03 rad/s deadbands occurs anywhere C1–C14. The first operational difference is **C1**, before any human pixels (angular B−A +0.067018 rad/s). The frozen operational threshold is native point XY RMSE ≥0.05 m, absolute lookahead-left difference ≥0.05 m, or absolute angular-command difference ≥0.05 rad/s. These descriptive thresholds are not significance tests.

A is the exact original successful baseline PNG sequence; B is the exact saved stopped-stage H05 preflight sequence. Both use the same baseline observation IDs, simulation times, full XYZ/wxyz poses, velocities, instruction and saved motion-history text. All 112 paired source files match historical manifests; all 28 images are uint8 RGB 1920×1080 with valid brightness. Camera intrinsics/clipping match; the largest full camera matrix error is 2.76823003e-07 (<1e−5). B’s saved render-completion/stopped-timeline evidence was checked. C1–C10 have zero human pixels, C11 is MARGINAL, C12–C14 CLEAR. Here C12 is the preflight pose at 6.800000355 historical simulation seconds, **not** failed closed-loop C12 at 7.000000365 seconds.

The table compares the two **new** inference sequences. Native left is metres; w is rad/s, positive = yaw-CCW LEFT. N/W indicates raw native/angular sign agreement (Y/N); a raw sign mismatch near zero is not a qualified direction reversal. State/δ parity means actual float32 six-value state, delay and delay-reference equality. G/P are generation-start / latest completion-detection poll **historical tick indices**, equal between A/B; “—” means no new generation start. KV availability is true for both on every row.

| C | Visibility | RGB MAE | Native left A/B | w A/B | N/W agree | Native XY RMSE m | Sampled IDs (both) | State/δ; delay s | KV; G/P (both) |
|---|---|---:|---:|---:|:---:|---:|---|---|---|
| C1 | HIDDEN | 13.995 | +0.02307/+0.05249 | +0.05042/+0.11744 | Y/Y | 0.04746 | 1 | equal; 0.0 | yes; 0/0 |
| C2 | HIDDEN | 3.775 | +0.02161/+0.02905 | +0.04899/+0.08044 | Y/Y | 0.03871 | 2 | equal; 1.8 | yes; 108/0 |
| C3 | HIDDEN | 3.527 | +0.01819/+0.02551 | +0.03925/+0.06290 | Y/Y | 0.03109 | 3 | equal; 2.3 | yes; —/0 |
| C4 | HIDDEN | 4.530 | +0.01807/+0.03784 | +0.03671/+0.06891 | Y/Y | 0.04740 | 4 | equal; 2.8 | yes; —/0 |
| C5 | HIDDEN | 4.518 | -0.00754/+0.02820 | -0.00066/+0.06086 | N/N | 0.10978 | 1,5 | equal; 3.3 | yes; 198/198 |
| C6 | HIDDEN | 4.781 | -0.00279/+0.03516 | +0.00134/+0.06827 | N/Y | 0.11406 | 1,6 | equal; 2.0 | yes; —/198 |
| C7 | HIDDEN | 4.395 | +0.01105/+0.07178 | +0.01916/+0.11179 | Y/Y | 0.12168 | 1,7 | equal; 2.5 | yes; —/198 |
| C8 | HIDDEN | 4.142 | +0.09033/+0.06494 | +0.14105/+0.11593 | Y/Y | 0.12755 | 2,8 | equal; 3.0 | yes; 288/288 |
| C9 | HIDDEN | 4.477 | +0.06738/+0.05835 | +0.11891/+0.11043 | Y/Y | 0.11625 | 3,9 | equal; 2.0 | yes; —/288 |
| C10 | HIDDEN | 5.066 | +0.09277/+0.04248 | +0.14394/+0.09058 | Y/Y | 0.10432 | 4,10 | equal; 2.5 | yes; —/288 |
| C11 | MARGINAL | 5.733 | +0.10547/+0.12305 | +0.18690/+0.20519 | Y/Y | 0.03957 | 1,5,11 | equal; 3.0 | yes; 378/378 |
| C12 | CLEAR | 6.922 | +0.16309/+0.16895 | +0.27781/+0.28238 | Y/Y | 0.03012 | 1,6,12 | equal; 2.0 | yes; —/378 |
| C13 | CLEAR | 7.192 | +0.21484/+0.19434 | +0.36822/+0.34002 | Y/Y | 0.05372 | 1,7,13 | equal; 2.5 | yes; —/378 |
| C14 | CLEAR | 7.766 | +0.17578/+0.21191 | +0.36055/+0.37115 | Y/Y | 0.03791 | 2,8,14 | equal; 3.0 | yes; —/378 |

At C12/C13/C14, native point XY RMSE is 0.03012/0.05372/0.03791 m; world XY RMSE is 0.030122040/0.053717038/0.037907487 m. All 28 counterfactual forward speeds are 1.5 m/s. Lookahead forward/left differences, angular differences, full native B−A arrays and corresponding world XY arrays are retained in the compact result and derived sidecar. Old baseline native predictions are not used as either paired control.

The saved-only RGB audit below preceded inference. Pixel difference is the mean absolute difference across three RGB channels, in 0–255 units; q50/q95/q99 show its whole-image distribution. “Changed” means any channel differs by >10. Mask in/out are region RGB MAEs, with “—” for an empty human mask. Luminance is Rec.709 on archived 8-bit RGB; brightness validity is PASS for both conditions on all rows. The exact q0/q25/q50/q75/q90/q95/q99/q100 distributions, inside/outside distributions and changed fractions are in `rgb-audit.json`.

| C | Human pixels | q50/q95/q99 | Changed % | Mask in/out MAE | Luminance A/B |
|---|---:|---:|---:|---:|---:|
| C1 | 0 | 9.00/36.67/48.00 | 47.86 | —/13.995 | 192.468/206.179 |
| C2 | 0 | 2.00/11.67/26.00 | 7.17 | —/3.775 | 204.572/207.611 |
| C3 | 0 | 2.67/9.33/20.67 | 4.53 | —/3.527 | 203.136/206.009 |
| C4 | 0 | 4.00/10.33/17.67 | 6.00 | —/4.530 | 200.156/204.250 |
| C5 | 0 | 4.00/10.67/17.00 | 6.25 | —/4.518 | 198.478/202.575 |
| C6 | 0 | 4.33/10.33/18.00 | 5.94 | —/4.781 | 196.540/200.856 |
| C7 | 0 | 4.00/9.67/18.67 | 4.67 | —/4.395 | 194.257/198.130 |
| C8 | 0 | 3.33/9.67/18.00 | 4.61 | —/4.142 | 190.650/194.213 |
| C9 | 0 | 3.67/10.67/19.00 | 5.98 | —/4.477 | 184.242/188.228 |
| C10 | 0 | 4.00/12.00/21.33 | 8.67 | —/5.066 | 175.983/180.589 |
| C11 | 264 | 4.67/14.00/22.00 | 11.97 | 41.609/5.728 | 164.106/169.410 |
| C12 | 5358 | 5.67/15.67/28.33 | 16.95 | 73.070/6.751 | 145.137/151.374 |
| C13 | 5849 | 5.67/16.67/35.67 | 20.08 | 67.401/7.022 | 128.579/135.048 |
| C14 | 6464 | 6.33/17.67/26.33 | 26.19 | 84.095/7.527 | 121.572/128.381 |

**Rendering confound:** previsible C1–C10 RGB MAE is 3.527–13.995/255 despite zero human semantic pixels. A comes from navigation captures and B from a separate stopped-stage rendering pipeline. At C12–C14, outside-mask MAE remains 6.751/7.022/7.527. Thus this archived sequence intervention includes background/render-history differences, and is not a human-only pixel intervention. Inputs passed the frozen validity checks, so the explicitly requested exploratory pair proceeded without replacement rendering or a third condition.

**Observed internal context:** actual sampled IDs, float32 state inputs, delay values/reference poses, generation-start ticks, latest completion-poll ticks and KV availability match at all 14 paired requests. No measured lifecycle mismatch triggers ASYNC_CONTEXT_CONFOUNDED. Nevertheless, returned reasoning text differs at all 14 requests; availability and matching lifecycle metadata do not prove equal KV contents. The exact consumed-KV identity and worker-ready wall time are unavailable. Original asynchronous closed-loop state is not reproduced. The unchanged VLM uses `do_sample=True, temperature=0.1, top_p=0.1, top_k=10`; seed 36 alone does not imply deterministic reasoning. With one fixed-order A→B pair, rendering/history and stochastic variation remain inseparable, and the previous navigation failure’s cause remains unresolved.

**Replay and coordinate contract:** the exact successful 2 Hz config was copied; its `predictions:48` field is retained but this offline wrapper bounds execution to 14 requests per condition. Original inference service, client, JPEG preprocessing, 9/6/3/0-second historical image sampling, strict checkpoint load, `predict_async`, action expert and controller are unchanged. Both receive copied historical metadata; no new prediction updates pose, instruction or history. The archived metadata’s `type:prediction` tag is preserved; the existing service routes it through its ordinary prediction branch. A service is closed before B is created, and the controller yaw filter resets to None for each condition.

Native arrays are (30,2) forward/left metres at the observation-time robot body origin; no native yaw, intrinsic timestamps, cumulative displacements or SE(2) poses are introduced. With the original Hospital world origin, +Z up, saved XYZ metres and normalized quaternion ordering w,x,y,z representing body→world, `p_world = p_observation + R_observation @ [forward,left,0]`; only then project to world XY. No application-time re-anchoring occurs. For state displacement, the unchanged service uses the inverse rotation of its recorded delay-reference body pose: `R_reference.T @ (p_observation − p_reference)` in metres. State is `[body vx, −body vy, body vz, body yaw-rate, reference-body dx, reference-body dy]` (m/s, rad/s, m as appropriate); `predict_async` receives float32, casts to model BF16 and omits vz before adding delay seconds. The lateral-velocity negation is preserved, not inferred from field names.

The nominal monotonic dispatch schedule starts after each model service is ready and uses historical offsets 0, 1.800000094, then 0.500000026 s increments. Each request waits until both its nominal deadline and previous actual dispatch plus the reference interval; there is one outstanding request and no catch-up burst. Maximum measured drift is A 11.416 ms / B 3.239 ms. Action-call wall latency median is A 54.927 ms / B 56.557 ms, maximum A 1.697298 s / B 1.633376 s (bootstrap). Request dispatch, action inference start, CUDA-synchronized ready and received timestamps are stored separately. Old observation clocks/ticks remain historical inputs; they are not the replay wall clock. Reasoning completion is a poll event, not exact worker-ready time. No execution/switch clock exists because no commands are applied.

**Provenance:** clean starting local and fetched `origin/hardcase-probe` were `7474461f0c72118cf86e314f44ed79c18e4cbb9b`; upstream was `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. At execution only the two additive scripts and their test were untracked; the tracked diff was empty. All three new file hashes and 127 other source paths were frozen before inference. No official `DynaNav/`, existing runtime/controller, environment, previous config or historical output changed.

- Original baseline config SHA256: `47dfec04f4fbee560e61389347e132765cd0135b360de91010575ccac675c0bb`.
- Base weight SHA256: `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`.
- Strict checkpoint SHA256: `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`.
- Frozen protocol SHA256: `4bd25fdd583c2b1cac66889d757c360a2961cdbf3548212a7aa3c0dd135132ab`.
- Raw manifest SHA256: `23a5d0d29de436fc2f09e6df265a40e3ed55dddce9be9dd6c894617e910d9e5f`.

Inference used Isaac 5 Python 3.11.13 (`5.0.0-rc.45+release.23960.184afb15.gl`), torch 2.7.0+cu128, torchvision 0.22.0+cu128, NumPy 1.26.0, transformers 4.57.6, RTX 5060 Ti; both service receipts explicitly record `simulation_app_started:false`. No Isaac 6 process was started. This offline test is not an official simulator reproduction. Host saved-only analysis used Python 3.12.3, NumPy 1.26.4, Pillow 10.2.0, matplotlib 3.6.3. No dependency installation occurred.

**Validation PASS:** 10 focused tests, including two existing controller tests with official AST parity and native shape/nonfinite validation. Synthetic mock replay tests verify isolation and 14-request bounds; they are infrastructure checks, not scientific evidence. Actual saved-only checks validate 28 finite native arrays, exact inputs and state/delay equality, independent service PIDs and shutdown ordering, sequential controller state, clocks/no catch-up, zero applications and unchanged raw hashes. Independent scalar maximum errors: controller 5.56e−17, full quaternion world transform 1.78e−15 m, state and native-delta 0. Postrun verification preserves all 112 paired files, 130 frozen sources and 4,787 protected historical/official/model files; all 128 new raw files match the frozen raw manifest. Two figures were visually reviewed. Final diff checks and the focused normal commit/push are bound by the separate publication receipt; raw output remains ignored.

Exact experiment/check commands (the run phase was executed once; existing output/claim paths reject reuse):

```bash
python3 -m unittest tests.test_h05_offline_rgb_pair tests.test_research.ControllerTests tests.test_research.RecordsTests.test_shape_finite -v
python3 -m py_compile research/h05_offline_rgb_pair.py research/analyze_h05_offline_rgb_pair.py
python3 research/h05_offline_rgb_pair.py --phase prepare --output-dir outputs/h05-offline-rgb-pair-20261013-01
set -o noclobber
python3 research/h05_offline_rgb_pair.py --phase run --output-dir outputs/h05-offline-rgb-pair-20261013-01 > outputs/h05-offline-rgb-pair-20261013-01/run.log 2>&1
MPLCONFIGDIR=/tmp/h05-offline-mpl python3 research/analyze_h05_offline_rgb_pair.py --output-dir outputs/h05-offline-rgb-pair-20261013-01
python3 research/h05_offline_rgb_pair.py --phase verify --output-dir outputs/h05-offline-rgb-pair-20261013-01
python3 /tmp/verify_h05_pair.py
git diff --check
```

Artifacts: [compact result](../results/ticvla-h05-offline-rgb-pair.json), [protocol](../outputs/h05-offline-rgb-pair-20261013-01/protocol.json), [RGB audit](../outputs/h05-offline-rgb-pair-20261013-01/rgb-audit.json), [derived comparison](../outputs/h05-offline-rgb-pair-20261013-01/derived/comparison.json), [native/world differences](../outputs/h05-offline-rgb-pair-20261013-01/derived/native-and-world-differences.json), [validation](../outputs/h05-offline-rgb-pair-20261013-01/validation.json), [publication SHA receipt](../outputs/h05-offline-rgb-pair-20261013-01/git-finalization.json). The temporary service JPEG buffers were cleaned by the unchanged model lifecycle and are not an archival dataset; original input PNGs and new native output arrays remain available locally.

Plots: [C1–C14 native lookahead and angular commands](../outputs/h05-offline-rgb-pair-20261013-01/derived/steering-comparison.png); [C12–C14 paired native curves](../outputs/h05-offline-rgb-pair-20261013-01/derived/primary-native-curves.png).

**One next uncertainty:** would a matched-render A/B control remove the previsible steering differences? This requires a separate decision; no repeated-input, rendering or navigation experiment was started.
