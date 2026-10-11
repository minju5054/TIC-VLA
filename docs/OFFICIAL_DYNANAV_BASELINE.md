# Official DynaNav baseline reproduction — 2026-10-11

**Latest decision: BLOCKED — HOST MAINTENANCE REQUIRED.** The follow-up below confirms that the NVIDIA mismatch persists. No fresh simulator startup or official navigation was attempted in this follow-up. The earlier one-attempt startup failure is retained later in this document as historical evidence.

## Follow-up: driver diagnosis, 2026-10-11 10:54–10:55 KST

Started clean on local/fetched `origin/hardcase-probe` **55f6d14888ac10dc3048de939bc0bc68688fe3b4**, fetched `upstream/main` **9fa6f8b66b9e121d5df5df071297bba8e5353ebb**. Official `DynaNav/` remains identical to upstream. Read the current rules, work log, this report, SETUP and ISAAC6_COMPATIBILITY; the optional misspelled `docs/OFFICIAL_DYNAV_BASELINE.md` does not exist.

Fresh evidence: [`outputs/official-dynanav-runtime-20261011-01`](../outputs/official-dynanav-runtime-20261011-01). `stage-a-host-diagnosis.json` contains exact command arguments, exit status/stdout/stderr, reboot-status files and relevant apt/dpkg history. `stage-a-followup.json` contains kernel-journal confirmation, package audit and installed-module checks. Each host subprocess had a 25-second bound; none timed out. `stage-a-summary.json` records the gate decision. These are diagnosis artifacts, not navigation data.

| Item | Current read-only evidence |
| --- | --- |
| `nvidia-smi` | **UNRESOLVED**, exit 18, driver/library version mismatch, on both checks |
| Running Linux kernel (`uname -r`) | `7.0.0-34-generic` |
| Loaded NVIDIA module (`/sys/module/nvidia/version`, `/proc/driver/nvidia/version`) | **595.84** |
| On-disk module (`modinfo nvidia`) | **595.99.02**, `/lib/modules/7.0.0-34-generic/updates/dkms/nvidia.ko.zst` |
| DKMS | `nvidia/595.99.02` installed for **7.0.0-34-generic** and **7.0.0-38-generic**; explicit `modinfo -k 7.0.0-38-generic -F version nvidia` also returns 595.99.02 |
| Installed driver/DKMS/compute/GL packages | `595.99.02-0ubuntu0.24.04.1` |
| NVML symlink target | `/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.595.99.02` |
| CUDA driver-library symlink target | `/usr/lib/x86_64-linux-gnu/libcuda.so.595.99.02` |
| Last boot (`uptime -s`) | **2026-10-08 09:20:03 KST** |
| Package transaction | `/usr/bin/unattended-upgrade`, **2026-10-11 06:23:11–06:24:00 KST**, 595.84 → 595.99.02 |
| Package state (`dpkg --audit`) | Exit 0, empty output; no incomplete package configuration reported |
| `/run/reboot-required` | Present: `*** System restart required ***` |
| `/run/reboot-required.pkgs` | `linux-image-7.0.0-38-generic`, `linux-base` |

The apt transaction and dpkg log establish an automatic driver update after the last boot. The running kernel still holds the old module while NVML/libcuda and installed modules are newer. The current boot's kernel journal explicitly reports client version 595.99.02 versus kernel-module version 595.84. **The most likely cause is the completed automatic update awaiting reboot**, not a TIC-VLA checkpoint or controller failure. The reboot marker itself names Linux kernel packages, not NVIDIA; the loaded-versus-installed NVIDIA evidence independently supports the diagnosis. No driver downgrade/reinstall is indicated by these checks.

Stage A's healthy-GPU gate fails, so **Stage B: NOT RUN; Stage C E16: NOT RUN**. Fresh Isaac 5 attempts **0**, model calls **0**, native predictions **0**. Strict model load, RGB, controller application, measured robot movement, collisions/recovery, latency/cadence and goal outcome are **UNKNOWN**. No GUI, RGB, XY/yaw plot or replay was created. The old startup failure is not counted as a new attempt. E17 and Isaac 6 porting are outside this follow-up; the older port discussion below is historical only.

### Specific maintenance plan — approval required, not performed

The current task explicitly says **“Do not reboot automatically”** and requires approval before host maintenance. Stop at this gate. No reboot, module unload/reload, package installation/removal, driver/CUDA/Python/Isaac/GPU/sudo configuration change, or service restart was performed.

1. Have the user save open work and confirm that the desktop, remote sessions and any running jobs may be interrupted. Obtain explicit approval for **one normal host reboot** to activate the already-installed matching driver. No driver installation is proposed.
2. Only after that approval, restart through the normal desktop restart action (equivalent administrative command: `sudo systemctl reboot`). This command has **not** been run. Leave bootloader and GPU settings unchanged; matching NVIDIA modules are installed for both listed kernels.
3. After reconnecting, run `uname -r`, `cat /sys/module/nvidia/version`, `modinfo -F version nvidia`, and `nvidia-smi`. Require a healthy NVIDIA-SMI device listing and matching loaded/on-disk/userspace driver versions before any Isaac launch. If the mismatch persists, stop and inspect the boot/module logs; do not automatically reinstall or hot-reload modules.
4. Only with that gate healthy, use the existing Isaac 5 installation for the authorized **one bounded RTX simulator-only test**, requiring constructor return, recognized GPU, ten updates and clean shutdown. Only a PASS permits **one original E16**, with bounded wall time and fresh evidence. No E17. Fixing this mismatch does not prove that the older Isaac 5 RTX SIGSEGV is resolved.

Validation for this follow-up is limited to the actually executed host checks, cross-checking the diagnosis records, protected-file hashes, append-only work log, unchanged upstream runtime, and Git diff/whitespace review. The prior two controller tests below were **not rerun** and are not a current simulator/model/navigation PASS. All **563 protected files** are checked before delivery, allowing only this report update and the work-log append; this includes both weights and every file in the preceding official-baseline evidence directory. The checkpoint identity remains the SHA256 in the table below. Final validation and normal commit/push receipts are saved in this follow-up directory; generated evidence is excluded from Git.

## Earlier investigation — historical record

## Source identity and evidence

- Starting clean local/fetched `origin/hardcase-probe`: `524489aa692b7537e16fc39a6b82a073fb1f9b8b`.
- Fetched official `upstream/main`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
- `git diff upstream/main -- DynaNav` is empty. All official code/configs and existing research code remain unchanged.
- Fresh evidence directory: [`outputs/official-dynanav-baseline-20261011-01`](../outputs/official-dynanav-baseline-20261011-01). `provenance.json`, `protected-before.json`, `host-before.json`, `host-after.json`, `isaac5-process-result.json`, `outcome.json`, and `source-audit.json` distinguish execution from static inspection. Generated evidence is not committed.
- Installed Isaac 5: `/home/gpuadmin/isaacsim-ticvla-5.0.0`, `5.0.0-rc.45+release.23960.184afb15.gl`; Isaac 6: `/home/gpuadmin/isaacsim`, `6.0.1-rc.7+release.42383.32955d8d.gl`.

Model identities, verified against the before/after file hashes:

| File | SHA256 |
| --- | --- |
| `/home/gpuadmin/Workspace/models/tic-vla/InternVL3-1B/model.safetensors` | `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484` |
| `/home/gpuadmin/Workspace/models/tic-vla/checkpoints/TIC-VLA-model.ckpt` | `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036` |

File integrity does not establish a successful checkpoint load in an official run. Earlier model-only/research successes are historical evidence, not this task's official navigation validation.

## One startup attempt: FAIL

Before starting, the three historical failures were inspected: `outputs/smoke-20261009-hospital-01/console.log`, `outputs/setup-20261009/isaac-only-headless.log`, and `outputs/setup-20261009/isaac-only-no-cuda-mask.log`. They report RTX startup SIGSEGV with `librtx.scenedb.plugin.so`/RTX renderer frames on the RTX 5060 Ti, before model loading. They remain unchanged.

The current host has a different, independently observed blocker:

| Check | Current result |
| --- | --- |
| Loaded NVIDIA kernel module | `595.84` |
| NVML and CUDA user-space library targets | `libnvidia-ml.so.595.99.02`, `libcuda.so.595.99.02` |
| Installed driver/compute/GL packages | `595.99.02-0ubuntu0.24.04.1` |
| `nvidia-smi` before and after | Exit 18, `Driver/library version mismatch` |

Exactly one headless RayTracedLighting `SimulationApp` startup was attempted, with no scene/model/navigation. Child-only environment isolation removed `LD_LIBRARY_PATH`, `PYTHONPATH`, and `PYTHONEXE`; CUDA device 0 was selected. The probe would print constructor completion, perform ten updates, then close.

[`isaac5-startup.log`](../outputs/official-dynanav-baseline-20261011-01/isaac5-startup.log) instead contains `NVML_ERROR_LIB_RM_VERSION_MISMATCH`, no usable GPU, and `HydraEngine rtx failed creating scene renderer`. The 120-second supervisor limit interrupted the owned process group; elapsed wall time was **121.015 s**. Constructor-returned, ten-updates-completed, and requested-shutdown markers are all absent. Kit's internal `app ready` line and process exit code **0** do not constitute startup success. No probe process remains.

No SIGSEGV was observed in this attempt. The old SIGSEGV is neither reproduced nor proven resolved: the current GPU initialization failure prevents a clean comparison. No second startup, driver/CUDA/Python change, reboot, install, or dependency modification was performed. Host checks are identical before and after.

## Intended official episodes, not executed

The source is [`DynaNav/configs/benchmark_full.yaml`](../DynaNav/configs/benchmark_full.yaml), seed 666 and benchmark success threshold 1.5 m. There are 85 configured episodes; none ran.

| Field | First gate: episode_16 | Conditional second run: episode_17 |
| --- | --- | --- |
| Scene | Official Isaac 5 Hospital `hospital.usd` | Same |
| Robot | Nova Carter, original asset and behavior | Same |
| Start world XYZ, yaw | `(7.38, 1.49, 0.01)`, 90 degrees | Same |
| Goal world XYZ | `(-12.23, 10.19, 0.01)` | Same |
| People | 0 | 10, existing official scenario |
| Timeout | 85 s | 95 s |

Original instruction: “Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.” E16 means no added human/obstacle; existing Hospital objects remain. The official loader's Nova start-height adjustment and original scene lighting must also remain. The research lights-on E16 run is not this baseline. E17 can run only after E16 validates; no new cart or pedestrian scenario is proposed.

## Full official behavior: source audit, not execution evidence

References: [`benchmark.py`](../DynaNav/benchmark.py), [`nova_carter_test_ticvla.py`](../DynaNav/behavior/nova_carter_test_ticvla.py), [`ticvla.py`](../DynaNav/ticvla.py), and [`robot_navigation_manager.py`](../DynaNav/robot_navigation_manager.py).

- **Model:** original InternVL3-1B, released checkpoint loaded with `strict=True` after prefix handling, BF16 action expert with 30 two-component waypoints. Current-image preprocessing uses `load_image(input_size=448, max_num=1)`. Native points have no yaw/timestamp channel. No model/controller substitute was used.
- **Scheduling:** executable `on_update` calls action inference synchronously when its sensor-frame counter reaches every third update and inference gates permit; the code assumes 30 benchmark frames/s and nominal 10 Hz sampling. Only VLM reasoning/KV generation uses the background worker. Header comments about an action queue/background action thread are not the executable contract. Actual cadence and physics advancement during inference remain UNKNOWN.
- **Reasoning clocks:** generation submission records frame and pose; completion is registered when polled. Delay uses benchmark-frame differences divided by 30, not measured wall latency. Bootstrap can wait for KV; subsequent actions reuse the available cache while reasoning runs asynchronously.
- **RGB/history:** original front Hawk left camera and third-person Replicator products are 1920×1080. Latest annotator data is saved as JPEG; history retains up to 190 entries and samples offsets 90/60/30/current, nominally −9/−6/−3/0 seconds at the assumed rate. Actual exposure/pose alignment is not proven by source inspection. Temporary image eviction is not archival logging.
- **State:** original world velocity is rotated into body coordinates and its lateral component negated; delayed displacement uses its original separate convention. Preserve exact state construction, one-second motion history, reasoning-start pose references, and delay input; variable names alone do not establish equal frames.
- **Lookahead:** original common-origin native points, 1 m accumulated segment-length lookahead, index clipped to `[2, T−3]`, bearing filter gain 0.35, curvature `2*y/L²`, `v=min(1.5, 1.2/(abs(curvature)+0.001))`, angular command clipped to ±1.2 rad/s. No invented initial waypoint or timed waypoint queue.
- **Slew:** linear acceleration/deceleration limits 2.0/2.5 m/s²; angular 3.0/3.5 rad/s²; original deadbands and `_slew` branch logic. The helper chooses the limit by the sign of `target-current`, so replacing it with a speed-magnitude rule would change behavior.
- **Recovery:** after initial inference, displacement under 0.1 m for 150 benchmark frames triggers backup; 150/30 is nominally **5 s**, despite the stale “3 seconds” comment. Backup is 300 frames, nominally 10 s, at −0.5 m/s with zero yaw rate, bypassing normal slew and skipping action inference. It then waits at zero command for qualifying post-backup fresh reasoning/inference. Preserve all freshness gates, frame counters, and command priority.
- **Wheels:** original `DifferentialController`, radius 0.152 m, wheel base 0.413 m, original joints and `apply_wheel_actions`. Their physical execution is unvalidated here.
- **Evaluation:** benchmark goal threshold 1.5 m, XY path length and straight-line-reference SPL. Human collision uses XY distance below 0.2 m. Physical collision requires original chassis sensor contact force ≥10 N for ≥100 consecutive benchmark frames. The stored contact count tracks consecutive contact frames, not incident count. These are not the research harness's raw nonfloor-contact definitions.
- **People integration:** `RobotNavigationManager` publishes robot current/future positions and radius for people to avoid it. It is not an additional robot obstacle-avoidance planner. Its old people-manager dependencies are initialized even in the no-human route.

## Comparison with current research harness

This table compares source/config behavior only. All official measured velocities, actual command timing, trajectory, contacts, recovery outcomes, goal progress, and success remain **UNKNOWN**.

| Aspect | Official DynaNav source | Current continuous Nova research path |
| --- | --- | --- |
| Runtime | Isaac 5 original behavior in benchmark | Isaac 6 physics + separate Isaac 5 model Python IPC |
| Action scheduling | Inline action call in `on_update`; nominal 10 Hz | Target 2 Hz requests; one outstanding future, OLD command retained |
| RGB/history | Latest annotator JPEG; history by entry offsets | Explicit observation render barrier; history by simulation times |
| State/delay | Original pose/state and benchmark-frame clock | Wrapper state/pose and simulation-time clock; not full timing parity |
| Native action/model | Original 30×2/model/KV pipeline | Original model reused; different request/history timing can change FRESH |
| Lookahead target | Original formula/filter | Formula parity tested; does not establish whole-behavior parity |
| Applied `(v,w)` | Recovery/wait priority and slew after target calculation | Direct target commands, without original slew/recovery/wait machine |
| Wheel geometry | Radius/base 0.152/0.413 m | Audited research Nova 0.14/0.4132 m |
| Scene | Original Hospital illumination/objects | Existing E16 lights-on run adds fixture lighting; cart run adds cart |
| Collision evaluation | Chassis sustained-force + human proximity rules | Raw nonfloor PhysX contacts and experiment-specific diagnostics |
| Actual velocities/path/success | UNKNOWN in this task | Prior saved research evidence only; no official comparison possible |

Thus the cart result cannot establish the paper's official TIC-VLA avoidance performance. Source differences can affect execution, but do not prove that original recovery would avoid the cart or that either implementation navigates better.

## Isaac 6 compatibility: substantial port required, not implemented

This audit reads the **installed** Isaac 6 sources without starting it. Paths, SHA256 hashes, classes, methods, and schema fields are retained in `source-audit.json`. Historical Isaac 6 startup/model-harness successes in [ISAAC6_COMPATIBILITY.md](ISAAC6_COMPATIBILITY.md) are not a current runtime PASS under the driver mismatch.

| Area | Installed-source finding | Required validation |
| --- | --- | --- |
| BehaviorScript | `omni.kit.scripting` absent; `omni.behavior.scripting.core` 110.1.0 has `BehaviorScript`, lifecycle methods, and scripting attachment command | Import/extension changes may be small; callback order, `delta_time`, reset/play/update semantics must match |
| Benchmark setup | IRA changed from 0.7.19 to 1.6.8; original `register_set_up_simulation_done_callback` and `set_up_simulation_from_config_file` absent | Use new async setup/event lifecycle without changing benchmark frame accounting |
| Config | New typed root `isaacsim.replicator.agent`, required `environment`, `simulation_duration` in seconds, forbidden extra fields | Original `global/scene/sensor/character/robot/replicator` schema is not accepted unchanged; preserve seed, units, timeouts, assets |
| People/navigation | `omni.anim.people` absent; new character groups/routines/triggers; old global position manager unavailable | Preserve command semantics, spawning/seed, people avoidance and robot publication; cannot replace with scripted research humans |
| Utilities | New `omni.metropolis.utils.carb_util` has module functions, no `CarbUtil` class | Narrow equivalent math adapter is possible, but does not repair people integration |
| Articulation/wheels | Legacy wheeled robot/controller APIs still present in `extsDeprecated` | Original asset/joints/radius/base, lifecycle and measured wheel response must be verified |
| Camera/contact | Replicator available; old `omni.isaac.sensor` absent, newer physics sensor package is under `extsDeprecated` | Preserve camera asset/annotator behavior and force/consecutive-frame collision semantics |
| Model/history/control | No reason to replace original algorithms | Preserve image sequence, state signs, clocks, synchronous action scheduling, full slew/recovery/wait logic; runtime equivalence UNKNOWN |

The benchmark unconditionally sets up IRA and the behavior imports people APIs even for E16's zero people. Bypassing these with the research harness would not satisfy reproduction. Renaming a BehaviorScript import alone is insufficient.

**Proposed files and ordered port gates, for a separate task:**

1. Establish a healthy host GPU and simulator-only startup before any runtime validation. Dependency inspection/dry-run must precede any necessary future install; no environment changes were made here.
2. `DynaNav/benchmark.py`: narrow version-dependent extension/setup/config/event/sensor adapters, preserving the single-episode worker, original asset/camera/lighting, frame counters and evaluation. New proposed `DynaNav/compat/isaac6_agent.py`: explicit old-config-to-new-schema and lifecycle adapter; keep original YAML files unchanged.
3. `DynaNav/behavior/nova_carter_test_ticvla.py`: only required imports/API hooks, preserve entire executable model/history/controller/recovery pipeline. `DynaNav/robot_navigation_manager.py`: utility and equivalent people-registration/publication interfaces. Do not substitute a no-op manager just to make E16 launch.
4. Preserve `DynaNav/generate_commands.py` and the seed/command sequence. An additional proposed `DynaNav/compat/isaac6_people.py` would translate existing commands to the new actor API **only if** timing, navigation and avoidance equivalence can be established. Otherwise stop and report the unresolved semantic difference. This is the main substantial-port risk.
5. Add targeted adapter/lifecycle/state/recovery/evaluation checks, then original E16; only after it passes, existing E17. Archive current RGB/actions, target and applied commands, measured state, reasoning clocks and contacts without altering scheduling. Label any validated Isaac 6 result **ADAPTED DYNANAV BASELINE**, not an unchanged Isaac 5 reproduction.

No listed port file was created or changed. `DynaNav/ticvla.py`, checkpoint, training and model architecture need no proposed algorithm change. Port implementation stopped at the user's substantial-port boundary.

## Results and conditional GUI command

| Requested runtime result | Status |
| --- | --- |
| Hospital load / Nova initialization / original behavior | NOT EXECUTED; UNKNOWN |
| Strict released checkpoint load in official navigation | NOT EXECUTED; UNKNOWN |
| Actual RGB / native action / controller / recovery | NOT EXECUTED; UNKNOWN |
| Measured path, yaw, collisions, goal progress/success | UNKNOWN, not zero or a navigation failure |
| World XY/yaw figures, GUI movement/replay | Not generated; no official trajectory exists |

The following is a **future conditional original Isaac 5 GUI command, not executed**. Use only after host/startup gates are resolved in a new authorized task. It selects exactly one child episode and refuses an existing output/log ID. The original application config is non-headless. Upstream temporary image buffers still require an archival plan before claiming complete saved observations.

```bash
(
  set -euo pipefail
  cd /home/gpuadmin/Workspace/TIC-VLA
  source .env.testing.local
  unset LD_LIBRARY_PATH PYTHONPATH PYTHONEXE
  export ISAAC_SIM_ROOT=/home/gpuadmin/isaacsim-ticvla-5.0.0
  export ISAAC_SIM_PYTHON="$ISAAC_SIM_ROOT/python.sh"
  run_id="official-e16-$(date -u +%Y%m%dT%H%M%S)-$$"
  result_dir="$PWD/outputs/$run_id"
  test ! -e "$result_dir"
  test ! -e "$PWD/DynaNav/logs/$run_id"
  mkdir "$result_dir"
  bash DynaNav/run_benchmark.sh DynaNav/configs/benchmark_full.yaml \
    --child --episode_name episode_16 --run_id "$run_id" \
    --result_json "$result_dir/result.json" \
    2>&1 | tee "$result_dir/console.log"
)
```

## Validation and decision

- Existing two `tests.test_research.ControllerTests` pass. They test only the lookahead formula slice/common-origin semantics, not full official behavior or navigation.
- Static Python parsing, official/upstream directory equality, E16/E17 config identity, installed API inventory, and startup failure-marker checks pass. No simulator/model tests were fabricated.
- Before documentation edits, **552 protected files** matched the starting hashes, including model weights, checkpoint, existing tracked sources, local environment file, installation launch/version files and the three old crash logs. Final verification permits only the append to `WORK_LOG.md`; the new report is additive. Host driver checks remain unchanged. Final receipts are in the fresh evidence directory.
- Diff/whitespace and append-only checks precede one focused documentation commit and normal push to `origin hardcase-probe`; final SHA/remote identity is recorded in `git-finalization.json` and the final response. No generated evidence is committed.

**Next recommendation:** arrange separately authorized host maintenance to make the loaded NVIDIA kernel driver and user-space libraries consistent, then verify `nvidia-smi` before scheduling a new bounded startup task. This task does not authorize changing the driver; resolving the mismatch also would not prove that the older Isaac 5 RTX SIGSEGV is resolved. Official obstacle avoidance and navigation performance remain unvalidated.
