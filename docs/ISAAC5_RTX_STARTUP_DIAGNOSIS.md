**Isaac Sim 5 RTX startup diagnosis — 2026-10-13, read-only**

**The established failure is native Isaac 5 RTX initialization before TIC-VLA.** The leading explanation is an interaction between the older Kit 107.3.1 RTX binaries and the current Blackwell/R595 stack (**moderate confidence; SUPPORTED_BUT_UNPROVEN**). The exact defective operation is unknown. A remaining kernel/userspace version mismatch, checkpoint failure and CUDA OOM do not explain the post-reboot evidence. No documented, low-risk fix for this exact Linux / RTX 5060 Ti / 595.99.02 / Isaac 5.0.0 combination was established.

For an official DynaNav reproduction, **qualify a separate Isaac 5 host first (Option B)**. This best preserves the current working Isaac 6 workstation and original human-scenario implementation. Continuing Isaac 5 on this PC without a newly justified change is presently blocked, not proven impossible. No simulator, compatibility checker, model, episode or navigation was launched in this task; no packages, settings or installations were changed.

**Current identities and installation checks**

| Item | Read-only observation |
|---|---|
| Git | Clean `hardcase-probe`; local and live `origin` both `f169c0ab30b9d85afda7d851d49689eaacdcfc69`; upstream `9fa6f8b66b9e121d5df5df071297bba8e5353ebb` |
| Host | Ubuntu 24.04.4, x86_64, kernel `7.0.0-38-generic`; current boot `2026-10-12 15:35:51` KST, later than the archived crash |
| GPU | NVIDIA GeForce RTX 5060 Ti, PCI device `2D04`, compute capability 12.0, 16,311 MiB; 1,012 MiB used at inspection |
| NVIDIA stack | Loaded **open** kernel module, on-disk module and NVIDIA-SMI all **595.99.02**; NVML, libcuda, NVIDIA GLX/EGL targets also 595.99.02 |
| CUDA / Python | Driver API 13020; `cuInit`, count/device/name/version queries all return 0, one GPU, no context created. This is distinct from installed `nvcc` **12.6.20**; system Python **3.12.3** |
| Isaac 5 | `/home/gpuadmin/isaacsim-ticvla-5.0.0`, **5.0.0-rc.45+release.23960.184afb15.gl**; archived runtime Kit **107.3.1+production.206797.8131b85d.gl** |
| Isaac 6 | `/home/gpuadmin/isaacsim`, **6.0.1-rc.7+release.42383.32955d8d.gl**; retained, not started |
| Launchers | Both `python.sh` paths resolve within their respective installation. No alternate Python override is currently set |

The selected VERSION/launcher identities still match the earlier compatibility audit:

| File | SHA256 |
|---|---|
| Isaac 5 VERSION | `9a386be0545b0c802b4508e4b87e155df519ec10c634e88aae0c3c0ff771c3f5` |
| Isaac 5 python.sh | `927fd6844fd8289bd6218849c8209ea706ea33193058a208387c89d4ac45a899` |
| Isaac 6 VERSION | `dd69799842a727b6a272edfaaa11b471be6ca19a2d2806973eb5ddb89a2aac0d` |
| Isaac 6 python.sh | `37f7e3717f1fce1800d0b15b55fcd754c909978bd2455b9d6de3d22d77a92bc3` |

**Archived failure sequence, independently rechecked**

| Attempt | Driver condition | Actual result |
|---|---|---|
| 2026-10-09 Hospital smoke; simulator-only headless; simulator-only without CUDA mask | 595.84 | Three native RTX SIGSEGVs, before scene/model/navigation. Headless and removing the mask did not remove the crash |
| 2026-10-11 before reboot | Kernel 595.84, userspace/on-disk 595.99.02 | Separate `NVML_ERROR_LIB_RM_VERSION_MISMATCH` failure; no usable GPU; 120 s limit, 121.015 s elapsed; interrupted exit 0, no constructor return, no SIGSEGV |
| 2026-10-11 after user reboot | Matching 595.99.02; GPU active in Vulkan | Same native RTX SIGSEGV; launcher exit 1 after **10.038287 s**, before the 180 s limit; no constructor return, ten updates or requested shutdown |

All five source-log hashes match `crash-comparison.json`. The post-reboot original Kit log, 3,265,792-byte minidump, TOML and Python stack each match their archived copy and recorded SHA256. No artifacts were copied again or overwritten. **Native frames 001–009 exactly match across all four SIGSEGV logs.**

Post-reboot chronology: Kit starts at `2026-10-11T02:15:20Z`; its internal `[8.074s] app ready` precedes the fatal event at `02:15:28Z` (11:15:28 KST). TOML uptime is 8 s; the supervisor’s 10.038287 s includes launcher/reporting time. Crash-reporter `[24ms]` and later counters are not seconds since application startup. Python remains inside `SimulationApp.__init__` line269 / `_prepare_ui` line530, called by `probe_isaac5.py:5`. `ISAAC5_CONSTRUCTOR_RETURNED` is absent. Internal app-ready is **not** constructor success.

The first non-signal native frame is **`librtx.scenedb.plugin.so`**. The saved symbol labels include frame001 `std::vector<...>::_M_realloc_insert...+0x123ef`, frame004 `carbOnPluginStartup+0x3b2fe`, frames006–008 `libcarb.scenerenderer-rtx.plugin.so`, and frame009 `libomni.hydra.rtx.plugin.so+0x4459`. These stripped-binary labels localize the failure; they do not prove a vector-allocation bug, memory exhaustion or a Python menu defect. No Hospital asset, robot or TIC-VLA model is loaded by this probe. Four saved GPU samples peak at **1,221 MiB**; this is a sampled device-wide maximum, not an instantaneous bound. No CUDA OOM appears; TOML reports about23.18 GiB available RAM.

Installed `gdb` was used in batch mode with initialization files and debuginfod disabled. It rejects the Breakpad minidump format; `minidump_stackwalk`, `minidump_dump`, `dump_syms` and LLDB were not found. `file` recognizes a 13-stream minidump. No debugger was installed and no live process was attached. Source-line/private-symbol diagnosis remains unavailable. Warp’s `cuDeviceGetUuid` warning recurs, but its causal connection to the fatal native thread is unestablished.

**Installed binaries and loader inspection**

The archived Kit log identifies Hydra’s extension as `extscache/omni.hydra.rtx-1.0.0+8131b85d.lx64.r`. Under the Isaac 5 root, its `bin/deps/librtx.scenedb.plugin.so` has SHA256 **`ceb181bade19f30774e48a738b834ddc473e35daaad21b9b984f33edd4187564`**, ELF build ID **`4646295e3a6bee06ad8540662e51110635fb6530`**. Related identities:

| Plugin within that extension | SHA256 | ELF build ID |
|---|---|---|
| `bin/deps/libcarb.scenerenderer-rtx.plugin.so` | `22f2c9a900c8708b55f4e4ed3aa4636e05d84aa90fb09c9b78122b22e4a59ea2` | `e34288d58fed8d2e955244f42d46fa8f1ba10e78` |
| `bin/libomni.hydra.rtx.plugin.so` | `aa9d9dfc21c68a02b786f3d132190aaa33b1a51fbc3d1e3edc14479779ce57fd` | `764286afe199fc7f661a2ee4fc204bd24305e71f` |

All are stripped x86_64 ELF libraries. The SceneDB/scene-renderer copies in `omni.kit.usd.mdl` are byte-identical, not conflicting versions. The original local release ZIP was streamed and rehashed: **`3de5ca513b6e71f325bccca5301c2835f2e257dfddd6787f02a24ab139bda89f`**, matching the initial download receipt. Eight selected installed members (VERSION, two launcher/setup scripts and five RTX plugin files) match their ZIP bytes. This supports an intact selected installation, not certification of every installed file or runtime success. The `rc.45` string is the recorded official 5.0.0 release artifact identity, not evidence by itself of an accidental preview install.

`readelf` records DT_NEEDED, RPATH and symbol versions. Bare `ldd`, including the base Python-launcher search path, leaves private USD/Boost/Hydra libraries unresolved because it does not initialize Kit’s extension loader; the working Isaac 6 installation shows the same limitation. Those files are present in each installation. Supplying their **own installed extension provider directories only to the ldd subprocess** resolves all inspected dependencies and version requirements across five Isaac 5 and three Isaac 6 plugin files. Python/USD stay within the selected runtime; libc/libstdc++ and libcuda resolve to expected host providers. No missing required version or obvious ABI conflict was found. This static resolution is not a fresh live Kit load map and cannot exclude a private driver/plugin ABI or native logic defect.

Current inherited `LD_LIBRARY_PATH` includes ROS and `/usr/local/cuda/lib64`; `PYTHONPATH` contains ROS Python3.12. The earlier Isaac 6 cuSPARSE/nvJitLink shadowing fault is documented separately. **The post-reboot Isaac 5 crash already occurred with inherited LD_LIBRARY_PATH, PYTHONPATH and PYTHONEXE cleared**, so repeating that isolation is not a new solution. No global paths were changed. The archived Kit log contains no `Failed to load` or `undefined symbol` entry. The package query’s exit1 is only “no packages found matching cuda-*”; nvcc exists and its version query succeeds. This does not establish a broken CUDA installation.

**Version-specific external evidence** — checked 2026-10-13. Documentation facts and forum hypotheses have different weight; none establishes this exact root cause.

| Source and publication/update date | Actual scope, finding and applicability | Documented fix/workaround and limits |
|---|---|---|
| [NVIDIA 5.0.0 release](https://github.com/isaac-sim/IsaacSim/releases/tag/v5.0.0), published2025-08-08; [versioned README](https://github.com/isaac-sim/IsaacSim/blob/v5.0.0/README.md) | Official GA release. The v5.0.0 source README includes RTX5080/RTX PRO Blackwell hardware, so “Isaac5 cannot support any Blackwell” is too broad. RTX5060Ti is not a listed reference GPU. Ubuntu24.04 caveat is in source-build instructions; it does not alone diagnose this prebuilt crash | No exact crash fix. Release-linked [5.0 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html) and [release notes](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/overview/release_notes.html) return HTTP404. An exact historical 5.0 driver matrix could not be recovered from these pages |
| [NVIDIA forum: same GPU](https://forums.developer.nvidia.com/t/isaac-sim-5-1-crashes-on-startup-with-rtx-5060-ti-blackwell-sm-120-rtx-scenedb-plugin-crash/366252), 2026-04-11; [staff reply](https://forums.developer.nvidia.com/t/isaac-sim-5-1-crashes-on-startup-with-rtx-5060-ti-blackwell-sm-120-rtx-scenedb-plugin-crash/366252/2), 04-13 | RTX5060Ti, **Windows11, Isaac5.1 rc.19, 595.79**; SceneDB DLL startup crash. Staff identifies an R595/Blackwell issue. Corroborates the compatibility hypothesis, but OS, binary and driver patch differ | Staff cites another user’s apparent downgrade success; no exact Linux5.0/595.99.02 fix or source-level defect |
| [NVIDIA forum: Linux5.0 crash](https://forums.developer.nvidia.com/t/isaac-sim-crash/369783), 2026-05-12 onward; [staff reply](https://forums.developer.nvidia.com/t/isaac-sim-crash/369783/2), 05-12 | **5.0.0-rc.45, Ubuntu24.04.4, RTX5050 Laptop, 595.58.03, kernel6.17**. Posted SceneDB offsets `0x123ef`, `0x12766`, `0x14a14`, `carbOnPluginStartup+0x3b2fe`, and Hydra `0x4459` match our signature. Different GPU/driver/kernel/RAM; strong symptom corroboration, not causal proof | Staff hypothesizes stacking/ABI changes and suggests 5.1/R580. The “5.0 validated580” link actually opens **5.1** requirements; it is not a verified5.0 matrix. Follow-up reports failed GPU initialization after downgrade, not a confirmed cure |
| [Isaac5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html), updated2026-06-04 | Official tested Linux x86_64 baseline **580.65.06** for **5.1**, not5.0. The aarch64 entry is a separate platform | Version-specific baseline only; not authority to downgrade this5.0/6.0.1 workstation |
| [Omniverse technical requirements](https://docs.omniverse.nvidia.com/utilities/latest/common/technical-requirements.html), updated2026-10-12 | Latest Omniverse validates Linux Blackwell drivers570.169/580.95.05/595.58.03. An old-Kit restriction for595.58.03 is explicitly scoped to **Azure RTX Pro6000 Preview**; Windows multi-GPU, ARM and ≤570.144 bugs have other scopes. None directly covers this host | Documented version fixes apply to those scopes. Latest compatibility does not retroactively certify Kit107.3.1, nor justify a universal “595 is unsupported” claim |
| [Isaac6.0.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/installation/requirements.html), updated2026-06-22 | Official tested Linux x86_64 driver **595.58.03**; 580.159.03 is **aarch64/DGX Spark**. Current595.99.02 differs, but this workstation already has empirical6.0.1 success | Replacing the shared driver with R580 would move the working x86_64 runtime away from its tested baseline; its effect would need revalidation, not an assumed improvement |
| [NVIDIA IRA migration guide](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ext_isaacsim_replicator_agent_migration_guide.html), updated2026-09-18 | Isaac6 replaces command files with routines/triggers and `omni.anim.people` with `omni.anim.behavior.core`; confirms a scenario-system redesign | A documented migration route, not proof of equivalent official DynaNav pedestrian behavior |

**Hypothesis assessment** — status describes the post-reboot RTX SIGSEGV unless explicitly marked historical.

| Hypothesis | Supporting evidence | Contrary evidence / limitation | Status |
|---|---|---|---|
| NVIDIA kernel/userspace version mismatch | Directly confirmed in the pre-reboot failed attempt | Post-reboot and current module/on-disk/NVML/CUDA versions match; GPU/API checks pass | **NOT_SUPPORTED** for current RTX cause; historical mismatch **CONFIRMED** |
| Isaac5 RTX binary and RTX5060Ti incompatibility | Repeated same binary signature on this GPU; related NVIDIA forum cases; newer Isaac6 works | No controlled GPU/driver A/B; Blackwell is not categorically excluded | **SUPPORTED_BUT_UNPROVEN** |
| Unsupported GPU architecture or driver combination | Exact5060Ti/595.99.02/Kit107.3.1 combination lacks verified validation | Versioned5.0 README includes Blackwell; latest docs includeR595. No exact prohibition found | **UNRESOLVED**; architecture-wide rejection not supported |
| Shared-library resolution or ABI mismatch | Inherited ROS/CUDA paths can affect generic launches; private ABI interaction remains possible | Crash survived clean child environment; own providers resolve DT_NEEDED/version requirements; no undefined-symbol error | **UNRESOLVED** for private ABI; no positive missing/shadowed-library finding |
| CUDA OOM / VRAM exhaustion | No supporting startup evidence | No model/scene; sampled1221/16311MiB; no CUDA OOM; abundant recorded RAM | **NOT_SUPPORTED** |
| TIC-VLA checkpoint or Python dependency failure | No supporting startup evidence | Simulator-only probe contains no model import/load; crash precedes constructor return. Separate archived model inference works | **NOT_SUPPORTED** |
| Other native RTX initialization defect | Fatal native SceneDB/renderer stack is established | Unsymbolized internals cannot distinguish logic bug, driver interaction, cache/settings interaction or another native defect | **SUPPORTED_BUT_UNPROVEN** as an alternative cause |

Explicit answers: **(1)** before TIC-VLA initialization; **(2)** selected installation/release bytes are intact but startup is blocked, not a certified fully working installation; **(3)** no remaining driver/library version mismatch detected; **(4)** older RTX binary × Blackwell/R595 compatibility is the leading moderate-confidence hypothesis, not a proven5060Ti hardware defect; **(5)** precise native root cause cannot be identified from available symbols/logs. The stack proves *where/when*, not the failing invariant. Shader/cache state and private driver interfaces remain unisolated; no cache was deleted or edited.

**Execution choices and next gate**

| Option | Effort and feasibility evidence | Main risk; effect on current Isaac6 | Original behavior and next verification gate |
|---|---|---|---|
| **A — retain Isaac5 on this PC** | Uncertain diagnosis effort; selected installation intact, repeated RTX failure. No exact documented host-preserving fix found | A driver change affects the shared GPU stack, display, CUDA clients and working6.0.1; regression/rollback cost unknown. Neither downgrading nor cache deletion is recommended from this evidence | Original5.0 behavior could be preserved if a verified fix exists. First obtain exact5.0/Kit107.3.1 compatibility or symbolized vendor diagnosis; only separately authorize a discriminating startup after that. Identical retry adds little |
| **B — separate compatible Isaac5 computer (recommended)** | Moderate setup/qualification effort, conditional on access. Original release and original human stack can remain together. Prefer an already demonstrated5.0 machine; an Ada RTX4080-class or higher16GB device is a candidate, not a guarantee | Candidate host compatibility is unproven until checked. **No changes to this workstation or its working6 installation**; transfer assets/checkpoints with hashes | Best preservation route. First read-only inventory of candidate GPU, OS/kernel, loaded/on-disk/userspace driver and exact5.0 build, backed by version-specific vendor requirements or a recorded same-build success. Then separately authorize bounded simulator-only constructor/updates/shutdown gate before any original E16 |
| **C — official DynaNav adaptation to existing Isaac6** | Substantial port and equivalence-validation effort. Existing controlled harness proves6RTX and separate5Python inference, not an official DynaNav port | BehaviorScript provider/lifecycle, IRA schema/setup events, human registration and robot-to-human avoidance publication differ. Environment risk can stay low with additive code and no binary changes, but scenario-fidelity risk is high | Unchanged official behavior is **not established**. First specify/test equivalent actor lifecycle, command semantics and avoidance coupling; then original robot/camera/control timing. No port performed |

Option C’s concrete blockers remain those in [ISAAC6_COMPATIBILITY.md](ISAAC6_COMPATIBILITY.md): installed IRA0.7.19→1.6.8, old configuration’s13 validation errors, removal of `register_set_up_simulation_done_callback` / `set_up_simulation_from_config_file`, `omni.kit.scripting.BehaviorScript`→`omni.behavior.scripting.core.BehaviorScript`, and absent `PeopleSettings`, old registration/position-manager APIs. Disabling people or replacing their commands with a scripted actor would alter the research scenario. Successful custom-harness runs do not remove these blockers.

**Recommended next step:** identify one separate, already working Isaac5 candidate host and verify its exact GPU/driver/build evidence before allocating a reproduction run. Do not change this workstation’s driver to follow a different-version forum recommendation. A container on this PC still uses its host GPU driver, so it does not by itself isolate that variable.

**Separate authorization required:** any new Isaac/compatibility-checker startup or episode; any installation, driver/kernel/cache/settings change or reboot; any DynaNav6 port; and any external submission of local crash artifacts. These requirements come from this task’s explicit restrictions. No approval is needed to finish this report, and none of these actions was performed. **No new diagnostic startup test is proposed here**: no sufficiently specific low-risk single-variable intervention was established from the available evidence. A future proposal must define the variable, discriminating outcomes, wall/process limits and rollback before launch.

**Evidence and validation**

Fresh ignored directory: [`outputs/isaac5-rtx-diagnosis-20261013-01`](../outputs/isaac5-rtx-diagnosis-20261013-01). `current-host.json` records exact bounded commands/stdout/stderr and selected paths; `binary-inspection.json`, `extension-library-resolution.json`, `resolved-native-dependencies.json`, `archive-binary-integrity.json`, `historical-evidence-audit.json`, `dump-reader-check.json`, `external-fetch-status.json` and `external-findings.json` separate measured facts from interpretation. Existing crash files are referenced in place. Nothing was uploaded.

**Validation PASS:** full before/after path/type/size/mtime/ctime/symlink inventories of **116,997 Isaac5 entries**, **187,322 Isaac6 entries**, and **35,696 prior-output entries** are unchanged; access times are excluded because reads can update them. Selected critical files and all files in the five requested historical evidence directories additionally pass SHA256 checks (116 protected paths), including official DynaNav Python sources. Nine selected host files, including driver libraries, the loaded-module version record and package database, also retain their hashes. These are full metadata inventories plus selected content checks, **not an all-binary content checksum**. Results are in `final-validation.json`; `report-validation.json` binds this final wording, and final local/remote commit equality is in `git-finalization.json`. Only this report and an append-only WORK_LOG entry are committed. No simulator/model tests were rerun; saved-evidence checks and `git diff --check` are the applicable validation.

**ISAAC5_DIAGNOSIS_COMPLETE — NO ENVIRONMENT CHANGES**
