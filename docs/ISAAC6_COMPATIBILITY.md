# Isaac Sim 6.0.1 compatibility investigation

Date: **2026-10-09, Asia/Seoul**. Outcome: **PARTIAL — core simulation works, but the official DynaNav model/navigation path remains blocked. NOT READY FOR CHUNK COLLECTION.**

Isaac 6 RTX startup was recovered using process-local environment isolation. The remaining differences include replacement of the human behavior system and Replicator Agent configuration/lifecycle, not just import renames. A small patch with demonstrated preservation of the official human scenario was not established. No scene, robot, RGB observation, native prediction, applied command, or robot movement was observed in this task. No single episode was launched because its prerequisites failed.

## Repository and environment

| Item | Observed value |
|---|---|
| Workspace | `/home/gpuadmin/Workspace/TIC-VLA` |
| Upstream | `https://github.com/ucla-mobility/TIC-VLA.git` |
| Origin | `https://github.com/minju5054/TIC-VLA.git` |
| Branch | `hardcase-probe` |
| Upstream/main SHA, locally and remotely verified | `9fa6f8b66b9e121d5df5df071297bba8e5353ebb` |
| Starting research SHA, locally and remotely verified | `c6d26552f74aeda00e38c799e86aa6fb3884a392` |
| Experiment revision | Starting SHA plus the additive probe scripts; no DynaNav source diff. Per-probe file hashes are in the local evidence directory. |
| Final delivery SHA | Exact SHA and remote verification are in `outputs/isaac6-compat-20261009/git-finalization.txt` and the final task report; a commit cannot contain its own SHA. |
| GPU | NVIDIA GeForce RTX 5060 Ti, 16,311 MiB, device 0 |
| Driver | `595.84`; NVIDIA-SMI reports CUDA driver capability `13.2` |
| System CUDA toolkit | `/usr/local/cuda` resolves to `/usr/local/cuda-12.6`; unchanged |
| Isaac 5 baseline | `/home/gpuadmin/isaacsim-ticvla-5.0.0`, `5.0.0-rc.45+release.23960.184afb15.gl` |
| Isaac 6 under test | `/home/gpuadmin/isaacsim`, `6.0.1-rc.7+release.42383.32955d8d.gl` |
| Isaac 6 Python | `3.12.13`, `/home/gpuadmin/isaacsim/kit/python/bin/python3` |

GitHub confirmed that origin is a fork of the official repository. Both main refs remained at the upstream SHA; no main-branch changes, force push, or history rewrite. The previous LightNav research repository was not modified.

Both Python launchers resolve to the `python.sh` file inside their own installations. These SHA-256 values match before/after; Isaac 6 also matches the previous setup receipt:

| File | SHA-256 |
|---|---|
| Isaac 5 `VERSION` | `9a386be0545b0c802b4508e4b87e155df519ec10c634e88aae0c3c0ff771c3f5` |
| Isaac 5 `python.sh` | `927fd6844fd8289bd6218849c8209ea706ea33193058a208387c89d4ac45a899` |
| Isaac 6 `VERSION` | `dd69799842a727b6a272edfaaa11b471be6ca19a2d2806973eb5ddb89a2aac0d` |
| Isaac 6 `python.sh` | `37f7e3717f1fce1800d0b15b55fcd754c909978bd2455b9d6de3d22d77a92bc3` |
| Both `requirements.txt` (empty) | `e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855` |

## Startup failure and recovery

The first test executed the requested simulator-only `SimulationApp` configuration: headless, `RayTracedLighting`, 640 × 480, then a ready marker and `app.close()`. It imported no TIC-VLA and loaded no episode.

**First failure:** after GPU detection, a deprecated Core dependency attempted to import existing bundled torch and failed with:

```text
libcusparse.so.12: undefined symbol: __nvJitLinkCreate_12_8, version libnvJitLink.so.12
```

The process exited **0**, but never printed `ISAAC6_SIMULATION_APP_READY`. Installed `isaacsim.core.deprecation_manager/api.py` logs a torch import error and calls Kit shutdown; this explains why exit 0 alone was insufficient.

The inherited `LD_LIBRARY_PATH` contained ROS paths and `/usr/local/cuda/lib64`. `ldd` showed the bundled cuSPARSE resolving `libnvJitLink.so.12` to system CUDA **12.6**. `readelf` confirmed that library lacks the requested `_12_8` symbol while the existing Isaac ML archive's **12.8** library provides it. The inherited `PYTHONPATH` also added ROS packages. No missing CUDA library was installed.

A second, otherwise identical simulator-only process removed inherited `LD_LIBRARY_PATH` and `PYTHONPATH`. It reached RTX startup completion, printed the ready marker, called `app.close()`, and exited 0 in **10.19 s**. GPU detection identified the RTX 5060 Ti. There was no native SIGSEGV or OOM. The 1-second sampled device-wide peak was **2,293 MiB**, including the desktop. Isaac 6 defaults to `fast_shutdown=True`, so successful `app.close()` terminates the process before a post-close print; startup-ready + shutdown log + exit status establish this test's result.

The two environment variables were removed together. The CUDA symbol inspection directly implicates the library search path, but a separate PYTHONPATH-only A/B test was not performed. The wrapper intentionally isolates this non-ROS diagnostic from both inherited paths. It does not alter shell profiles, global environment, installed libraries, or driver. Isaac 5 was not rerun or modified; its earlier failure evidence remains historical, and this Isaac 6 result does not prove an Isaac 5 fix.

## Dependencies and integrity

No packages were added, removed, or changed. No installation was proposed after the independent actor-system blocker was established, so no pip install or installation dry-run was needed. A future installation still requires a dry-run against the **actual extension-enabled** package set. Do not apply the Isaac 5 requirements/NumPy constraints wholesale to this runtime.

| Module | Before SimulationApp, `python.sh` | After clean-environment SimulationApp |
|---|---|---|
| numpy | PASS `2.3.1` | PASS `2.3.1` |
| torch | Not on initial Python path | PASS `2.11.0+cu128` |
| torchvision | Not on initial Python path | PASS `0.26.0+cu128` |
| transformers | Missing | Missing |
| huggingface_hub | Missing | Missing |
| accelerate | Missing | Missing |
| einops | Missing | Missing |
| timm | Missing | Missing |
| cv2 | PASS `4.13.0` | PASS `4.13.0` (`opencv-python-headless==4.13.0.90`) |
| matplotlib | PASS `3.10.8` | PASS `3.10.8` |
| yaml | PASS `6.0.3` | PASS `6.0.3` |

Isaac's base experience enables `omni.isaac.ml_archive`, exposing existing torch/torchvision from `extsDeprecated/omni.isaac.ml_archive/pip_prebundle`. Their absence in initial `pip freeze` does **not** justify installing/replacing them. An actual CUDA 16 × 16 matrix product passed after startup, with the expected value 16.0 on the RTX 5060 Ti.

Pre/post `python.sh -m pip freeze` text is byte-identical, SHA-256:

```text
5eabcde29d898ca9743569f4d5b8a2d801ba500b13fa7302bd03bffe274bb855
```

Package inventory diff: empty. **Binary stack changed: NO** — no torch, torchvision, torchaudio, CUDA, USD, Isaac/Kit, NumPy, or system-package installation/replacement. Recorded VERSION/launcher/requirements hashes are unchanged; these are selected integrity checks, not a claim of hashing every binary. Normal simulator logs/caches may be generated by startup.

Existing model identities remain those in [artifacts.json](artifacts.json): base model `/home/gpuadmin/Workspace/models/tic-vla/InternVL3-1B`, checkpoint `/home/gpuadmin/Workspace/models/tic-vla/checkpoints/TIC-VLA-model.ckpt`. Base weights SHA-256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`; checkpoint SHA-256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`. No replacement or new model download; no model load was attempted under Isaac 6. Earlier Isaac 5 strict-load success is not an Isaac 6 validation result.

## Source/API audit

Classification describes API availability, not end-to-end equivalence. `COMPATIBLE` below means the existing call surface remains available, sometimes through deprecated packages; it does not certify physics, rendering, or actor behavior.

| Source location / category | Isaac 5 official dependency | Installed Isaac 6 finding | Classification / decision |
|---|---|---|---|
| `run_benchmark.sh:5–25` / launcher | `ISAAC_SIM_PYTHON`, cwd DynaNav, GPU env (default 1) | Same launcher protocol; shell CUDA path can poison startup | UNCHANGED interface; additive process-isolation wrapper only. Any future episode must explicitly use GPU 0. |
| `benchmark.py:1428` / SimulationApp | `from isaacsim import SimulationApp` | Same constructor and tested RTX configuration | COMPATIBLE; startup PASS after environment isolation. |
| World / simulation context | No direct `World(...)` construction in this DynaNav path; robot classes depend on Core | Legacy `isaacsim.core.api` import works in `extsDeprecated` | COMPATIBLE import; reset/physics lifecycle UNKNOWN until a scene test. |
| `benchmark.py:518–567` / timeline | `omni.timeline`, play/stop, timecodes, `commit_silently` | Same module present; Kit/physics versions differ | COMPATIBLE surface; frame cadence and physical timing UNKNOWN. |
| `benchmark.py:1011`, behavior pose reads / stage and prim utilities | `omni.usd`, `pxr.UsdGeom/Gf`, `add_reference_to_stage` | USD initialized; `isaacsim.core.utils.stage/prims` imports pass from deprecated Core | COMPATIBLE surface; robot asset not loaded. |
| `behavior/nova_carter_test_ticvla.py:35`, `benchmark.py:1112` / scripting | `omni.kit.scripting.BehaviorScript`, `ApplyScriptingAPICommand` | Old extension absent locally; `omni.behavior.scripting.core.BehaviorScript` and same command exist; lifecycle/event implementation differs | RENAMED provider. Candidate small import adaptation, not applied while human semantics are blocked. |
| `benchmark.py:952` / extension enablement | 14 extensions including scripting, people, sensor alias | Missing locally: `omni.kit.scripting`, `omni.anim.people`, `omni.isaac.sensor`; other named extensions present | REMOVED/RENAMED dependencies; no silent skipping in an episode. |
| `behavior/...:133–147` / robot articulation | `WheeledRobot`, `WheelBasePoseController` | Original modules remain under `extsDeprecated/isaacsim.robot.wheeled_robots`; relevant signatures preserved | COMPATIBLE call surface. Do not substitute new experimental robot classes without validation. |
| `benchmark.py:1025` / Nova Carter | Original Isaac 4.5 `nova_carter_sensors.usd`, original wheel joints/start transform | Same URL retained in unchanged source; not loaded | UNKNOWN asset/physics compatibility; no asset substitution. |
| `behavior/...:1047` / DifferentialController | `forward(command=[v,w]) -> ArticulationAction`, radius .152, base .413 | Deprecated controller retains formula and observed signature | COMPATIBLE API; no model-derived command or wheel result tested. |
| `behavior/...:1048` / wheel action | `WheeledRobot.apply_wheel_actions(actions)` | Same observed signature | COMPATIBLE API; application and actual actuation UNKNOWN. |
| `behavior/...:491–552` / camera, RGB | Fixed camera prim; Replicator render products 1920 × 1080, RGB annotators | `omni.replicator.core` imports, render-product/annotator API remains in installed source | COMPATIBLE surface; actual camera content/RGB UNKNOWN. |
| `benchmark.py:952` / viewport | `omni.kit.viewport.window`, manipulator and property extensions | Same extension identifiers installed; base viewport starts | COMPATIBLE startup; headless empty-stage test is not scene visual QA. |
| `benchmark.py:1002–1009`, behavior `on_update` / physics update | Kit `next_update_async`, PhysX, callback-driven control | Kit and PhysX startup pass; physics packages remain | COMPATIBLE surface; callback ordering and episode physical time UNKNOWN. |
| `benchmark.py:451–508` / scene config | IRA `0.7.19`: `global`, `scene.asset_path`, `character.command_file`, writer parameters | IRA `1.6.8`: typed config, `environment.base_stage_asset_path`, actor groups/routines/triggers, writers list | SIGNATURE_CHANGED/schema redesign; unchanged hospital template fails with 13 validation errors. |
| `benchmark.py:998–1005` / scene setup lifecycle | `register_set_up_simulation_done_callback`, `set_up_simulation_from_config_file` | Both absent; asynchronous `setup_simulation` and new events exist | REMOVED old entry points; lifecycle adaptation alone cannot restore old actor system. |
| behavior `:36,446–454` / people settings and registration | `PeopleSettings`, `AgentEvent.AgentRegistered` | `omni.anim.people` absent; new `omni.anim.behavior.core` | REMOVED old API; no demonstrated equivalent registration/avoidance semantics. |
| `robot_navigation_manager.py:12,87–98` / vector helpers | `CarbUtil.scale3/sub3/add3` class methods | `carb_util` has equivalent-looking module functions; class import fails | RENAMED surface; small helper adaptation possible in isolation, not applied. |
| `robot_navigation_manager.py:13–14,93–98` / human avoidance | `GlobalCharacterPositionManager`, `Utils`, publish robot current/future positions and radius | Old manager path absent; replacement behavior/agent registry architecture | REMOVED old API; equivalent human response to robot publication UNKNOWN. |
| `ticvla.py:5–24,891+` / model | Torch, torchvision, transformers, sibling VLM, unchanged `(B,30,2)` head | Torch/vision work after startup; transformers and inference extras absent | UNKNOWN model compatibility; first runtime import fails at transformers. No head/checkpoint modification. |
| Spot behavior | Policy robot + same old scripting/people imports | Shares missing dependencies; not selected for this task | UNKNOWN beyond shared static blockers; no Spot runtime claim. |

The IRA migration is confirmed by NVIDIA's [Replicator Agent migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ext_isaacsim_replicator_agent_migration_guide.html): old command files are replaced by routines/triggers and the animation backend changes. Installed IRA 1.6.8 source confirms the schema and removed methods. NVIDIA's [wheeled-robot migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/robot_control_extensions_to_experimental.html) documents the deprecation/split; this installation still provides deprecated Python classes, so wholesale controller migration is unnecessary for an import check.

**Why no DynaNav patch:** changing two import names would leave command files, actor configuration, registration, and human avoidance unresolved. Omitting people, bypassing position publication, replacing commands with random routines, or swapping scenes could alter the research scenario. Copying Isaac 5 compiled extensions into the Python 3.12/Kit 110 installation is not a supported minimal bridge. The investigation does not prove that a carefully validated larger port is impossible; it establishes that the requested small, semantics-preserving port is not currently justified.

## Validation matrix and first failures

| Stage | Result | Evidence / first failure |
|---|---|---|
| A: Isaac 6 SimulationApp | PASS after environment isolation | First inherited-environment run failed before ready; clean-environment repeat reached ready. |
| RTX renderer / GPU | PASS (startup only) | RTX startup completed; RTX 5060 Ti active in Vulkan device table. |
| Shutdown | PASS for simulator-only test | `app.close()`, shutdown log, exit 0; fast shutdown exits the process. |
| B: DynaNav runtime imports | FAIL | `ModuleNotFoundError: transformers` from unchanged sibling `DynaNav/ticvla.py`. |
| C: benchmark module import | PASS | Exact workspace `DynaNav/benchmark.py`, without invoking episode main. |
| D: Nova Carter behavior import | FAIL | `ModuleNotFoundError: omni.kit.scripting`. |
| Independent navigation-manager import | FAIL | `CarbUtil` class missing; people manager also absent in source audit. |
| Scene-config preflight | FAIL | Official `_create_episode_config` template passed directly to new `RootConfig.model_validate`: 13 errors. Temporary diagnostic output paths only; no config migration. |
| E: DynaNav scene load | BLOCKED | Config/lifecycle/actor prerequisites fail; no scene loaded. |
| F: robot spawn | BLOCKED | No scene/behavior readiness. |
| G: camera/RGB | BLOCKED | No robot/camera render product. |
| H: TIC-VLA construction | NOT ATTEMPTED | Runtime dependency failure and blocked observation path. |
| I: checkpoint strict load | NOT ATTEMPTED | Earlier Isaac 5 PASS is not reused as Isaac 6 evidence. |
| J: native action prediction | BLOCKED | No current simulator RGB reaches TIC-VLA. |
| Prediction shape / finite values | NOT ATTEMPTED | `(1,30,2)` remains source/checkpoint knowledge, not an observed Isaac 6 tensor. |
| K: controller command | BLOCKED | No prediction; signature inspection is not a command test. |
| Wheel action | BLOCKED | No applied action. |
| Robot motion | BLOCKED | No pose/displacement samples. |
| Single hospital episode | NOT ATTEMPTED | Required preflight gates did not pass. No full benchmark. |

Observed runtime prediction shape, finiteness, `(v,w)`, wheel action, and pose displacement are all **unavailable**, not zero. There is no navigation-performance, hard-case, or reconciliation evidence.

## Additions and semantic impact

| File | Change and rationale | Impact / uncertainty |
|---|---|---|
| `scripts/isaac6_python.sh` | Version-checked launcher; clears inherited LD_LIBRARY_PATH/PYTHONPATH/PYTHONEXE only for its child; uses selected installation's own setup | Resolves observed CUDA library shadowing; no model/controller/scenario change. Non-ROS diagnostics only; do not assume suitability for an unrelated ROS workflow. |
| `scripts/probe_isaac6.py` | Default simulator-only probe; optional independent unchanged-source imports, API/signature checks, CUDA operation, and schema validation | No robot/scene/model construction, no shim, no missing-extension download, no chunk logger. Reports failures explicitly and preserves a nonzero audit exit code. |
| `AGENTS.md`, `docs/RESEARCH_CONTEXT.md` | This fork hosts future TIC-VLA-based research; distinguish prior LightNav repository and gated future work | Research scope correction only. |
| `docs/SETUP.md` | Link historical Isaac 5 setup to this later authorized investigation | Preserves original evidence. |
| This report and `docs/WORK_LOG.md` | Provenance, limitations, API matrix and delivery record | Documentation only. |

Official `DynaNav/`, model/controller implementation, episode configurations, requirements, and `.env.testing.local` remain unchanged. No unresolved rename-only shim is presented as a working port.

## Reproduction and local evidence

From the repository root, use fresh output paths. Do not source the Isaac 5 `.env.testing.local` to select this probe's runtime:

```bash
mkdir -p outputs
probe_dir=$(mktemp -d "$(pwd)/outputs/isaac6-probe-XXXXXX")
TICVLA_ISAAC6_ROOT=/home/gpuadmin/isaacsim \
  timeout --signal=INT --kill-after=30s 180s \
  bash scripts/isaac6_python.sh scripts/probe_isaac6.py > "$probe_dir/startup.log" 2>&1
# Run only after checking startup-ready, shutdown and exit status above.
TICVLA_ISAAC6_ROOT=/home/gpuadmin/isaacsim \
  timeout --signal=INT --kill-after=30s 180s \
  bash scripts/isaac6_python.sh scripts/probe_isaac6.py --audit > "$probe_dir/audit.log" 2>&1
# Expected audit exit: 1 on this installation; inspect ISAAC6_PROBE records.
```

Ignored evidence directory: `outputs/isaac6-compat-20261009/`.

- `installations-before.json`, `installations-after.json`: separated runtime identity/hash snapshots.
- `pip-list-before.txt`, `pip-freeze-before.txt`, `pip-freeze-after.txt`, `imports-before.txt`: initial launcher inventory, post inventory and import results.
- `startup.log`, `startup-result.json`, `startup-gpu.csv`: original environment failure, exit 0 without ready.
- `startup-clean-env.log`, `startup-clean-env-result.json`, `startup-clean-env-gpu.csv`: successful simulator-only repeat.
- `api-probe.log`, `api-config-probe.log`, corresponding exit-code and source-provenance files: actual import/signature/CUDA/schema evidence. The second probe added CUDA/schema checks; it did not patch DynaNav.
- `final-api-probe.log`, `final-api-probe-exit-code.txt`, `final-probe-source-provenance.json`: final script validation reproduces expected import/schema failures and returns 1; no unexpected traceback. The launcher's Isaac 5 rejection also returned the expected status 2.
- `final-integrity.json`: selected installation hashes and initial pip inventory still match after all probes.
- `official-extensions.json`: installed paths for every extension requested by the official benchmark.
- `git-finalization.txt`: final SHA, normal push and remote verification.

Before chunk logging, resolve and validate preservation of original human command sequencing, registration/avoidance and scene lifecycle on the selected runtime; then plan inference dependencies without binary replacement, and prove a single original hospital episode's RGB → prediction → controller → wheel action → motion chain. Axis, timing and reasoning-state caveats in [ACTION_SEMANTICS.md](ACTION_SEMANTICS.md) still apply. A successful empty-stage startup alone does not open the collection gate.
