# Reproducible TIC-VLA setup

Inspection date: **2026-10-09 (Asia/Seoul)**. This document records this machine, not a generic claim that all upstream environments work.

For the current working Isaac 6 four-wheel platform with separate model inference, see [MINIMAL_CLOSED_LOOP.md](MINIMAL_CLOSED_LOOP.md). The installation/reproduction attempts below are retained as historical evidence.

This is the historical **Isaac 5 official reproduction** record. A subsequent task explicitly authorized probing existing Isaac 6.0.1; its results and process-local launcher are in [ISAAC6_COMPATIBILITY.md](ISAAC6_COMPATIBILITY.md). Earlier statements below that Isaac 6 was out of scope describe the original setup task. Neither installation nor the original Isaac 5 local configuration was replaced. This fork now hosts the user's TIC-VLA-based research, as defined in [RESEARCH_CONTEXT.md](RESEARCH_CONTEXT.md).

## Repository

| Item | Value |
|---|---|
| Workspace | `/home/gpuadmin/Workspace/TIC-VLA` (clone directly into `.`; no nested workspace) |
| Upstream | `https://github.com/ucla-mobility/TIC-VLA.git` |
| Verified upstream main | `9fa6f8b66b9e121d5df5df071297bba8e5353ebb` (`git ls-remote` and clone HEAD agree) |
| Origin | `https://github.com/minju5054/TIC-VLA.git` (GitHub confirms fork parent `ucla-mobility/TIC-VLA`) |
| Research branch | `hardcase-probe` |
| Local revision | Run `git rev-parse HEAD`; smoke runs record exact SHA plus dirty status/diff. A commit cannot embed its own SHA in its contents. |

The host workspace was empty. The tool sandbox initially exposed read-only `.git/.agents/.codex/.aws` placeholders; an authorized host inspection established that they were sandbox mounts, not user files. Git operations used host access. The initial sandbox GPU/auth errors were not host failures. Host `gh auth status` succeeded as `minju5054`; no credentials were changed. The fork did not exist (authenticated API 404), so `gh repo fork ... --clone=false --remote=false` created it.

## Machine before installation

| Item | Observed value |
|---|---|
| GPU | One NVIDIA GeForce RTX 5060 Ti, physical device 0 |
| VRAM | 16,311 MiB reported by NVIDIA-SMI; initial use 1,113 MiB |
| Driver | 595.84 |
| Driver CUDA capability | 13.2 shown by NVIDIA-SMI; this is not the installed compiler version |
| Installed CUDA compiler | `nvcc` 12.6, V12.6.20; unchanged |
| OS / architecture | Ubuntu 24.04.4 LTS / x86_64 |
| CPU / RAM | AMD Ryzen 7 9800X3D, 8 cores/16 threads; about 30 GiB RAM usable |
| Initial disk available | About 1.6 TiB on `/dev/nvme0n1p1` |
| System Python | `/usr/bin/python3`, 3.12.3; unchanged |
| Tools | `uv 0.12.9`, `gh 2.45.0`; conda not on PATH |
| Existing Isaac | `/home/gpuadmin/isaacsim`, `6.0.1-rc.7+release.42383.32955d8d.gl` |
| Existing Python launcher | `/home/gpuadmin/isaacsim/python.sh`; not used for TIC-VLA and not modified |

Pre-install inspection and hashes of the existing Isaac VERSION/launcher/requirements are kept locally under `outputs/setup-20261009/`. No system Python/CUDA/driver, LightNav, or main-research environment is changed.

## Isaac Sim 5.0.0

Official references: [TIC-VLA setup](https://github.com/ucla-mobility/TIC-VLA/tree/9fa6f8b66b9e121d5df5df071297bba8e5353ebb#setup), [NVIDIA 5.0.0 download page](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html), [NVIDIA workstation installation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_workstation.html).

The versioned NVIDIA documentation URLs returned 404 through the web fetch service during this task; NVIDIA's indexed 5.0.0 download record and the exact upstream README identify the standalone package below. The actual package endpoint returned HTTP 200 with 8,774,227,758 bytes and Last-Modified `2025-08-07`. No login/download gate was encountered. Do not substitute a newer Isaac release or bypass a future license/access prompt.

Selected isolated installation: `/home/gpuadmin/isaacsim-ticvla-5.0.0`; launcher: `/home/gpuadmin/isaacsim-ticvla-5.0.0/python.sh`.

```bash
mkdir -p "$HOME/Workspace/models/tic-vla/downloads"
curl -fL --retry 3 \
  https://download.isaacsim.omniverse.nvidia.com/isaac-sim-standalone-5.0.0-linux-x86_64.zip \
  -o "$HOME/Workspace/models/tic-vla/downloads/isaac-sim-standalone-5.0.0-linux-x86_64.zip"
# Extract only to a NEW directory; do not overlay an existing installation.
test ! -e "$HOME/isaacsim-ticvla-5.0.0"
mkdir "$HOME/isaacsim-ticvla-5.0.0"
unzip -q "$HOME/Workspace/models/tic-vla/downloads/isaac-sim-standalone-5.0.0-linux-x86_64.zip" \
  -d "$HOME/isaacsim-ticvla-5.0.0"
```

The ZIP's internal version is `5.0.0-rc.45+release.23960.184afb15.gl` (the official 5.0.0 release artifact), with 102,131 entries / 17,662,195,747 expanded bytes. Archive SHA-256: `3de5ca513b6e71f325bccca5301c2835f2e257dfddd6787f02a24ab139bda89f`.

The archive's `post_install.sh` only creates the examples symlink and installs a desktop icon. Its icon installer unconditionally overwrites `~/.local/share/applications/IsaacSim.desktop`. To preserve the existing 6.x desktop entry, only the examples-symlink step was applied to the new runtime; the icon step was omitted. This does not change simulator/model behavior.

```bash
cd "$HOME/isaacsim-ticvla-5.0.0"
test -e extension_examples || \
  ln -s exts/isaacsim.examples.interactive/isaacsim/examples/interactive extension_examples
```

## Inference artifacts

No training dataset is needed or downloaded. The checkpoint happens to be hosted in a Hugging Face **dataset repository**; only the checkpoint file is selected from it.

| Artifact | Source / immutable revision | Local path |
|---|---|---|
| Base VLM | `OpenGVLab/InternVL3-1B` / `4415a3b810e636d11dfa86b0e9ba40bb00535aa8` | `/home/gpuadmin/Workspace/models/tic-vla/InternVL3-1B` |
| TIC-VLA checkpoint | `handsomeYun/TIC-VLA` (dataset) / `671de608f39ccae41b66cbb24ab923c0275f8fb4`, `TIC-VLA-model.ckpt` | `/home/gpuadmin/Workspace/models/tic-vla/checkpoints/TIC-VLA-model.ckpt` |

The existing HF cache contained LightNav, not this base model; no duplicate TIC-VLA cache was found. Files are downloaded from revision-qualified HTTPS resolve URLs and verified against API LFS SHA-256 where available. The local `artifact-manifest.json` outside Git contains source URL, revision, filename, path, byte size, and SHA-256 for every downloaded file. Source metadata JSON is retained beside it. The committed `docs/artifacts.json` records portable provenance, not weight contents.

Equivalent download commands after inference dependencies are installed:

```bash
source .env.testing.local
"${ISAAC_SIM_PYTHON}" - <<'PY'
import os
from huggingface_hub import snapshot_download, hf_hub_download
snapshot_download(
    repo_id="OpenGVLab/InternVL3-1B",
    revision="4415a3b810e636d11dfa86b0e9ba40bb00535aa8",
    local_dir=os.environ["TICVLA_BASE_MODEL_PATH"],
    ignore_patterns=["examples/*", ".gitattributes"],
)
hf_hub_download(
    repo_id="handsomeYun/TIC-VLA", repo_type="dataset",
    revision="671de608f39ccae41b66cbb24ab923c0275f8fb4",
    filename="TIC-VLA-model.ckpt",
    local_dir=os.path.dirname(os.environ["TICVLA_CHECKPOINT_PATH"]),
)
PY
```

## Machine-local configuration

The tracked upstream `.env.testing` is unchanged. Use ignored `.env.testing.local` from the repository root; do not source upstream `.env.testing` afterward because it overwrites the selected paths.

```bash
export ISAAC_SIM_ROOT="$HOME/isaacsim-ticvla-5.0.0"
export ISAAC_SIM_PYTHON="${ISAAC_SIM_ROOT}/python.sh"
export TICVLA_DYNANAV_ROOT="$(pwd)/DynaNav"
export TICVLA_BASE_MODEL_PATH="$HOME/Workspace/models/tic-vla/InternVL3-1B"
export TICVLA_CHECKPOINT_PATH="$HOME/Workspace/models/tic-vla/checkpoints/TIC-VLA-model.ckpt"
export TICVLA_OUTPUT_DIR="$(pwd)/outputs"
export CUDA_VISIBLE_DEVICES=0
export PYTHONNOUSERSITE=1
export HF_HOME="$HOME/Workspace/models/tic-vla/hf-cache"
export PYTHONUNBUFFERED=1
```

`DynaNav/run_benchmark.sh:8` defaults to GPU **1**, but this host has only device **0**. Explicit `CUDA_VISIBLE_DEVICES=0` is necessary; the upstream script respects this override. `TICVLA_OUTPUT_DIR` does **not** relocate all upstream DynaNav logs: behavior writes `DynaNav/logs/<run_id>` and benchmark writes temporary data under the config's `tmp/<run_id>` directory. The wrapper records these locations and rejects reused IDs.

## Dependencies and commands

Use only the selected Isaac Python. The package metadata has no automatic runtime dependencies. The official `requirements-test.txt` pins NumPy `<2` and excludes training requirements to preserve Isaac's PyTorch/CUDA binaries.

```bash
source .env.testing.local
# This exact combination was installed after reviewing a pip --dry-run report.
"${ISAAC_SIM_PYTHON}" -m pip install -e . -r requirements-test.txt \
  -c requirements-test-isaac5.constraints.txt
"${ISAAC_SIM_PYTHON}" scripts/validate_runtime.py
"${ISAAC_SIM_PYTHON}" scripts/validate_runtime.py --construct
bash scripts/research_smoke.sh episode_hospital_smoke
```

The smoke wrapper invokes the unchanged official launcher with `--child --episode_name episode_hospital_smoke`; it does not run the four-episode example or full benchmark. Default wall limit is 900 s (override `TICVLA_SMOKE_WALL_SECONDS`), with INT then a 30 s kill grace. Hospital parameters remain official: 20 people, Nova Carter, original scene/start/goal/instruction/70-second frame-count timeout. GUI mode is upstream default; this task does not add a headless patch.

Inspect stdout/result and existing robot logs for actual construction, strict checkpoint load, image input, successful reasoning/action output, and navigation commands/motion. Upstream can return exit 0 even when episode runtime fails, so exit 0 is not a smoke pass. Frame-based benchmark scores from this setup are not performance claims.

Every run must have a fresh ID. Upstream `_empty_dir` clears the per-run behavior directory on play; image history is also pruned/deleted. These temporary images are not an archival collection. The wrapper is setup/provenance support, not a chunk logger.

## Validation results and remaining blockers

**Outcome: repository, artifacts, Isaac Python imports, CUDA kernels, model construction, and strict checkpoint load work. The official scene smoke fails during Isaac RTX startup before scene loading. No scene inference or navigation execution was observed.**

| Check | Result | Evidence |
|---|---|---|
| Official repo setup | PASS | Correct root, remotes, fork parent, research branch and upstream SHA |
| Isaac Sim 5.0.0 import | PASS | Callable `SimulationApp` from the selected runtime |
| DynaNav TIC-VLA imports | PASS | Loaded `DynaNav/ticvla.py`, `ticvla_vlm.py`, and inference extras |
| Package `import ticvla` | FAIL | Eagerly imports training data modules requiring absent `pytorch_lightning`; not required by the sibling DynaNav runtime |
| CUDA visibility/kernel | PASS | One RTX 5060 Ti; actual torch CUDA matrix multiplication passed |
| Config load | PASS | All eight YAML files parse; four example scenes expand correctly, local asset paths exist |
| Base model available | PASS | 1,876,463,472-byte weights, verified LFS SHA-256 |
| Checkpoint available | PASS | 1,938,501,547-byte checkpoint, verified LFS SHA-256 |
| Checkpoint load | PASS | Exact Nova Carter key-prefix stripping and `load_state_dict(..., strict=True)` |
| Model construction | PASS | Official base VLM and DynaNav action head, unchanged bfloat16/no-flash-attention settings |
| DynaNav scene launch | FAIL | Native SIGSEGV inside RTX startup, before scene setup |
| TIC-VLA inference | BLOCKED | No simulator RGB observation reached the model |
| Robot/navigation execution start | BLOCKED | Simulator bootstrap did not finish |
| Minimal smoke test | FAIL | Official hospital child run exits 1 through `python.sh` after native SIGSEGV |

Verified checkpoint dimensions: `action_expert.action_chunk_embed.weight = (30,512)` and `action_expert.mlp.2.weight = (2,256)`. This validates the horizon/output structure, not an observed scene prediction.

Selected installed versions: Python 3.11.13; torch `2.7.0+cu128`, torchvision `0.22.0+cu128`, torchaudio `2.7.0+cu128`; NumPy 1.26.0; transformers 4.57.6; huggingface-hub 0.36.2; accelerate 1.15.0; timm 1.0.30; einops 0.8.2; OpenCV 4.11.0.86; matplotlib 3.10.3; PyYAML 6.0.2; Pillow 11.2.1. The constraints file also pins the newly installed transitive dependencies. No torch-family or NumPy replacement occurred.

`pip check` reports bundled metadata conflicts/missing optional packages, including NVIDIA SRL's `usd-core`, docstring-parser, plotly/tenacity, selenium/trio, msal/pyjwt, and lxml/scipy/s3transfer/cryptography version constraints. All implicated distributions are outside the newly installed inference package set; the raw result is preserved locally. They were not “fixed” by replacing Kit's USD or unrelated bundled packages. Runtime imports and strict model load passed despite this packaging check failure. The training-package import failure is separately preserved; no Lightning or training requirements were installed just to make that optional import pass.

### Memory measurements

| Stage | Measurement |
|---|---|
| Initial desktop baseline | 1,113 MiB device-wide |
| Model-only process | 1,820.21 MiB allocated; 1,856 MiB reserved; 1,840.17 MiB peak allocated by torch |
| Model-only device-wide peak | 3,100 MiB, sampled every 1 s (includes desktop) |
| Official smoke startup peak | 1,490 MiB device-wide, sampled every 1 s; model never loaded in this process |
| Isaac-only headless startup peak | 1,484 MiB device-wide, sampled every 1 s; startup crashed |
| Scene + model / pre-inference / inference peak | Not reached; unavailable |

No CUDA OOM was observed. Sampled NVIDIA-SMI peaks are not exact instantaneous peaks. A fully initialized Isaac-only memory value is unavailable because startup crashed.

### Classified blocker and diagnosis

`FAIL: Isaac Sim 5.0.0 RTX startup native crash; likely driver/runtime incompatibility.`

The official hospital attempt (`smoke-20261009-hospital-01`) crashed at about 12.5 s during `SimulationApp(...)`, before `BenchmarkRunner` scene loading or TIC-VLA behavior construction. Backtrace includes `librtx.scenedb.plugin.so`, `libcarb.scenerenderer-rtx.plugin.so`, and `libomni.hydra.rtx.plugin.so`. Warp also reports `cuDeviceGetUuid` driver-entry/API errors. These messages do not establish a model, checkpoint, or OOM problem.

Two independent simulator-only diagnostics reproduced the same crash: headless with `CUDA_VISIBLE_DEVICES=0`, then headless without the CUDA mask to isolate the enumeration warning. Neither diagnostic imports TIC-VLA, loads a scene, or performs inference. Removing the window or mask did not resolve it. There is no broad upstream patch or simulator version substitution.

**Inference, not a locally proven driver A/B result:** the current 595.84 driver is the leading compatibility suspect. NVIDIA staff identify 595.xx/Blackwell RTX startup failures in a [same-GPU report](https://forums.developer.nvidia.com/t/isaac-sim-5-1-crashes-on-startup-with-rtx-5060-ti-blackwell-sm-120-rtx-scenedb-plugin-crash/366252/2) and discuss the 580-series validation baseline in a [5.0.0 crash report](https://forums.developer.nvidia.com/t/isaac-sim-crash/369783/2). The former is Isaac 5.1, so it is corroboration, not proof for this installation. Host driver changes and using existing Isaac 6.x are explicitly out of scope and were not performed.

Before implementing a successive chunk logger, obtain a working **Isaac Sim 5.0.0 RTX runtime on a compatible host/driver stack**, then rerun the same single-episode wrapper and verify scene, RGB, action output, and applied navigation commands. A separate compatible machine is an option that preserves this host's existing research environment. A container alone does not replace the host GPU driver. No model access/login/license step is currently blocking, and no reauthentication is needed.

The source-audited axis, timing, and reasoning-state association caveats remain collection-design requirements; this failed startup provides no experimental hard-case evidence. If a host driver change is considered later, it needs a separate explicit task because this task forbids it.

### Local evidence and reproduction

Raw evidence is ignored, intentionally not committed:

```text
outputs/setup-20261009/preinstall-inspection.txt
outputs/setup-20261009/isaac-bundled-runtime.txt
outputs/setup-20261009/pip-plan.json
outputs/setup-20261009/pip-install.log
outputs/setup-20261009/pip-freeze.txt
outputs/setup-20261009/pip-check.txt
outputs/setup-20261009/model-construction.log             # initial package-import failure
outputs/setup-20261009/model-construction-attempt2.log    # actual DynaNav strict load PASS
outputs/setup-20261009/model-gpu-attempt2.csv
outputs/smoke-20261009-hospital-01/console.log
outputs/smoke-20261009-hospital-01/provenance.txt
outputs/smoke-20261009-hospital-01/working-tree.patch
outputs/smoke-20261009-hospital-01/untracked-files.sha256
outputs/setup-20261009/smoke-hospital-01-gpu.csv
outputs/setup-20261009/isaac_only_probe.py
outputs/setup-20261009/isaac-only-headless.log
outputs/setup-20261009/isaac-only-no-cuda-mask.log
```

All runtime attempts used upstream/local HEAD `9fa6f8b66b9e121d5df5df071297bba8e5353ebb` with the additive setup working tree. Upstream inference/benchmark source was unchanged. The official smoke manifest records dirty state and setup-file hashes. Repeat runs after the setup commit automatically record their new local HEAD.

Minimal native-crash reproducer from the repository root (preserve each new log rather than overwriting old evidence):

```bash
source .env.testing.local
"${ISAAC_SIM_PYTHON}" -c 'from isaacsim import SimulationApp; app = SimulationApp({"renderer":"RayTracedLighting", "headless":True, "width":640, "height":480}); print("launch ready"); app.close()'
```

The simulator minidumps and Kit logs are under the **new** runtime's `kit/data/Kit/Isaac-Sim Python/5.0/` and `kit/logs/Kit/Isaac-Sim Python/5.0/`. Existing 6.x VERSION, Python launcher, and requirements file hashes still match their pre-install values. System Python 3.12.3 and nvcc 12.6 remain unchanged; no other research environment was targeted.
