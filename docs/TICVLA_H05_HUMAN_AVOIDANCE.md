# TIC-VLA 2 Hz H05 stationary human avoidance — one trial

**`TICVLA_H05_NOT_QUALIFIED`. Runtime PASS; navigation FAIL.** The robot failed to complete the original left turn, travelled north/east, contacted Hospital structures, and never approached or passed H05. This is an upstream route-following failure before a close human encounter. It does not demonstrate successful avoidance or a human collision. Exactly **one** scientific trial, **48** predictions, no retry or tuning.

| Metric | Measured result |
|---|---|
| First human pixels / first CLEAR | Both **C12**, simulation **7.000000 s**; C13 also CLEAR |
| First CLEAR mask | **5,649 pixels**, fraction **0.00272425**; bbox `[10,335,70,555]`, **60 × 220 px** |
| First CLEAR RGB | Rec.709 mean **143.928**, std **43.061**, brightness PASS; center distance **8.630668 m** |
| CLEAR before conflict / ≥5 m useful lead | **Undefined:** no actual human conflict or upstream safety-plane approach. The 8.63 m center distance is not a measured lead-to-conflict distance |
| Complete safe FRESH | **0/48** pre-passage curves; **0/2** current-CLEAR/brightness-PASS curves |
| Native classifications | Collision/low clearance **0**, horizon insufficient **48**, safe prefix **0**, complete safe bypass **0** |
| Passing side / downstream boundary / additional 1 m | **NOT_REACHED / no / no** |
| Minimum center / conservative surface clearance | **7.569587 / 6.712604 m**, continuous closest at **9.044487 s**; no margin violation |
| Human PhysX contact | **None**; the large clearance results from not reaching the person |
| Structural PhysX contact | **Yes**, 14 collider-pair spans; first at **16.266668 s** |
| Original strict `<2 m` goal entry | **No**; closest goal distance **17.300141 m**, final **20.817667 m** |
| Primary / full run | Both end at **25.500001 s**, tick **1530**; C48 observed at **25.000001 s**, normal final period retained; no post-goal segment |
| Qualification | `HUMAN_NAVIGATION_FAIL / SAFE_FRESH_INSUFFICIENT_EVIDENCE` |

The native curves all fail to reach H05's upstream safety plane. Their `HORIZON_INSUFFICIENT` label therefore does **not** establish that prediction length alone caused failure: the robot and curves are going elsewhere. The frozen evaluator additionally emits `VISIBILITY_NOT_ESTABLISHED` because its *preinteraction* qualification cannot be established without an interaction. **Actual CLEAR visibility is established**, and the compact result marks conflict-relative visibility metrics as undefined. Direct observed failures are `STRUCTURAL_CONTACT`, `FAILED_TO_BYPASS`, and `GOAL_NOT_REACHED`; there is no `HUMAN_CONTACT`, `LOW_CLEARANCE`, or `IMPLEMENTATION_FAILURE`.

The clearest failure evidence is the native/controller direction at the junction. C12 uses lookahead `(1.14844, +0.05615)` body forward/left metres and command `(1.5, +0.09202)` m/s, rad/s. C13, while the human is still CLEAR, changes to `(1.125, −0.10254)` and `(1.5, −0.12757)`; C14 commands `w=−0.22061`. The successful saved baseline's C13/C14 commands were positive `+0.285/+0.314 rad/s`. The unchanged direct controller follows the new curves into a rightward correction and the robot fails to finish the required left turn. This is measured local evidence, not proof that the person caused the directional change. Logged background reasoning at C15 mentions a pedestrian and claims the left turn is complete, while measured yaw is only about **104°**, rather than westward 180°. Reasoning text/its answer triples are not the native `(30,2)` action and are not treated as native yaw or current-frame perception.

The first structural contact involves `/Root/Geo_W_TrimStrateBase1_453/...` and the right wheel; robot center is `(10.89214,18.43283)`. Subsequent wall/chassis and stair contacts are recorded. Near-horizontal-normal structural reports include nonzero impulses (maximum **60.8253 N·s**) and minimum reported separation **−0.002781 m**, so this is not inferred from a disk proxy. The human capsule excludes outstretched arms. No full-body human safety claim is made.

The frozen matched-progress corridor lateral metric is **undefined**, because the robot never enters the reference west-corridor X range (`x≤2.55210 m`; actual minimum x **4.87872 m**). Descriptive saved-data departures are maximum distance to the successful baseline polyline **10.462481 m**, and maximum absolute Y offset from corridor center `y=10.59954 m` after first reaching its entrance Y band **8.027919 m**. These are not human bypass excursions.

Action-call wall latency min/median/p95/max is **0.04881 / 0.05475 / 0.11505 / 1.94371 s**; the maximum is bootstrap C1. Observation-to-application wall delay is **0.24006 / 0.50826 / 0.58875 / 2.42617 s**; simulation delay is **0.06667 / 0.10000 / 0.15000 / 2.00000 s**. Applied command ranges are `v=0.00417…1.5 m/s`, `w=−1.2…+0.54717 rad/s`. Background reasoning start steps, completion **poll** steps, sampled image IDs and delay inputs (**0…2.5 simulation s**) remain in each request. Exact worker-ready time is unavailable; these clocks are not substituted for action latency. Brightness PASS is **29/48** observations; the first failure is C21, after route departure. C12/C13 both pass.

## Frozen setup and implementation

Run: `outputs/nova-e16-h05-stationary-20261013-01`. Authorization and expanded configuration: `outputs/h05-avoidance-setup-20261013-01/`. Started with local and fetched origin both **`e7b7abe11e8f05409c208364c7ec192d4a6f9bea`**, branch `hardcase-probe`; upstream **`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`**. At execution, the three additive Python files were untracked; the receipt binds their bytes, all relevant runtime sources, dirty status/diff identity, exact run ID and evaluation rules before execution. Final publication SHA is recorded in `git-finalization.json` in the authorization directory.

| Frozen identity | SHA256 |
|---|---|
| H05 selected configuration | `1b9d2beec527fdede71f3b33984cfd3e11a0d75dcf01da1ac4b1544d57da434e` |
| Successful baseline runtime configuration | `47dfec04f4fbee560e61389347e132765cd0135b360de91010575ccac675c0bb` |
| Human root USD | `2e5d7fc72d6e2ed67f0ab86e2f3aaa91066019a4c71266361735956f232ccff0` |
| Base `model.safetensors` | `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484` |
| `TIC-VLA-model.ckpt` | `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036` |
| Unchanged `research/continuous.py` | `d25f5c7a308024dae45b8ca2b2488cb1f205ae4bbdc140206a388ca5b0014715` |

H05 is exactly `(-2.4478963167155454,10.349539794921895,0)` m, root yaw **0 rad**, prim `/World/PreflightHuman`, the original Isaac 6 `People/Characters/male_adult_construction_03/male_adult_construction_03.usd` URL from the selected file. The human asset hash comes from the prior compact result/physical receipt, rather than being embedded in the selected file. Radius **0.25 m**, capsule cylinder height **1.2 m**, robot radius **0.6069825421953869 m**, additional margin **0.05 m**, required center separation **0.9069825421953869 m**.

The runner deep-copies the selected `baseline_config`, verifies it against the original runtime file, and adds only `stationary_human` and the existing visibility thresholds. `human.start_position` is explicitly copied into `position`; historical `experiment_authorized:false` is unchanged. Original E16 start/goal/instruction, seed36, Hawk camera, 153 fixture lights, 60 Hz physics/control, 2 Hz inference target, 48 predictions, direct controller (`1.5/1.2/1.0/0.8/0.35`), history and asynchronous reasoning/KV lifecycle are preserved. The runner reuses `HospitalLightsSimulation`, `StationaryHuman`, `run_model_loop`, `NovaRecords`, and `validate_continuous`. **No existing runtime or official source was edited.** No precomputed bypass was imposed.

All **17** actual pre-model checks passed: source/asset identities, actual PhysX and authored human pose, capsule/semantic label, floor support, no Hospital/robot overlap, camera/wheels/spawn, identical lighting readback and collision inventory, brightness, and completed render at unchanged physics tick0. One extra initial model-free capture is archived as `preflight.png`; it is not added to model history. The gate then exclusively created the single scientific claim before the only model load. There were **no failed infrastructure launches** and no scientific retry.

This is the adapted **Isaac 6.0.1-rc.7 / Python3.12.13 / NumPy2.3.1** simulator and separate **Isaac5 Python3.11.13 / torch2.7.0+cu128 / torchvision0.22.0+cu128 / NumPy1.26.0 / transformers4.57.6** inference process, without a second SimulationApp. It is not an unchanged official Isaac5 simulation reproduction. Full version receipts are retained in the run directory.

World coordinates use original Hospital origin, metres, +Z up. West is `(-1,0)`; LEFT is `(0,−1)`. Native forward/left points are embedded as `[dx,dy,0]`, rotated body→world with the **observation-time wxyz quaternion**, translated by that observation's XYZ, then projected to XY. A separately identified origin-to-first-point connector is checked continuously. No cumulative sum, application-time reanchoring, invented native yaw/timestamps, or chunk stitching is used. Complete H05 prediction requires surface clearance ≥0.05 m and endpoint at least **1 m beyond the downstream safety plane**; actual structural contacts are evaluated separately. Primary evaluation ends at first strict goal entry or the existing C48 period remainder, without extension.

The baseline and this trial have **identical C1 measured XYZ/quaternion**, but C1 RGB differs by mean absolute channel value **12.7924/255**, and commands differ before H05 becomes visible. The added initial render changes render history; its contribution and asynchronous context effects are not isolated. Therefore one trial cannot attribute the failed turn specifically to the human or establish general avoidance reliability.

## Saved artifacts, commands and checks

[Compact result](../results/ticvla-h05-human-avoidance.json), [equal-aspect execution plot](../outputs/nova-e16-h05-stationary-20261013-01/derived/h05/execution-xy.png), [individual FRESH comparison](../outputs/nova-e16-h05-stationary-20261013-01/derived/h05/fresh-curves.png), [original first CLEAR RGB C12](../outputs/nova-e16-h05-stationary-20261013-01/diagnostics/rgb_000012.png), [original closest-approach RGB C16](../outputs/nova-e16-h05-stationary-20261013-01/diagnostics/rgb_000016.png). C16 is at 9.000000 s, **44.49 ms before** continuous closest approach; no exact-time image was synthesized. All 48 native metrics/anchors/lookaheads/commands are in `derived/h05/predictions.json`; original requests and arrays remain under `raw/requests/`.

The existing occlusion GUI requires pair/request-metric files absent from this run's evidence format, so there is no directly supported launch command. No new GUI or replay framework was built. Only the two requested plots were finalized; display-only corrections mark the first structural contact and compare C11–C16, with unchanged frozen evaluator/rules and raw data.

Exact commands from repository root (the executed run ID is consumed and cannot be reused):

```bash
python3 research/h05_stationary_run.py --phase freeze --receipt-dir outputs/h05-avoidance-setup-20261013-01 --run-id nova-e16-h05-stationary-20261013-01
bash scripts/isaac6_python.sh research/h05_stationary_run.py --phase run --receipt-dir outputs/h05-avoidance-setup-20261013-01
python3 research/h05_stationary_analysis.py --run-dir outputs/nova-e16-h05-stationary-20261013-01
MPLCONFIGDIR=/tmp/h05-mpl python3 outputs/h05-avoidance-setup-20261013-01/finalize.py
python3 -m unittest tests.test_h05 -v
python3 research/h05_stationary_run.py --phase verify --receipt-dir outputs/h05-avoidance-setup-20261013-01
git diff --check
```

**Validation:** six focused synthetic adapter/geometry tests PASS; 48-prediction real-model/OLD-retention contract PASS; independent mask, dense-human clock/pose, raw immutability and protected-source checks PASS. The before/after manifest covers **4,319** prior artifact/official source/model files, including all six repeatability runs and the prior H05 preflight, plus **130** frozen source paths. Raw images, arrays, logs and plots remain ignored. A normal focused commit/push preserves history.

**Next uncertainty:** why does the policy fail to finish the left turn, with divergence already present before H05 is visible? Initial render/history and asynchronous context contributions remain unisolated. No follow-up experiment was started.
