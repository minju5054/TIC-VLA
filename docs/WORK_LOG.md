# Work log (append only)

## 2026-10-09 — Official reproduction and source audit

- Read the full user request and checked the current path, directory, Git status/remotes before cloning. Host folder was empty; sandbox protection mounts were not treated as user files or removed.
- Recorded host GPU, driver, compiler, Python, OS, storage, CLI/auth state, and existing Isaac version before installing a runtime. Saved local inspection under `outputs/setup-20261009/`.
- Verified upstream `main` via `git ls-remote`: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Cloned official repository directly into the workspace, created `hardcase-probe`, renamed official origin to upstream.
- Authenticated GitHub query showed no existing `minju5054/TIC-VLA`; created the fork, added origin, confirmed parent and matching main SHA. No credentials changed, force push, or history rewrite.
- Read official README/DynaNav README, launcher, benchmark, both behavior implementations, both TICVLA implementations, VLM loader, test/train requirements, conda YAML, `.env.testing`, train/benchmark/data-collection configs, and data-generation/action-target code.
- Confirmed one physical GPU; upstream default device 1 is invalid here. Local environment overrides device 0 without changing the official launcher.
- Found existing Isaac `6.0.1-rc.7` and left it untouched. Selected separate `~/isaacsim-ticvla-5.0.0`; official 5.0.0 ZIP download is ungated and accessible. Started download outside Git.
- Resolved immutable Hugging Face revisions and selected only base model files and released checkpoint; no dataset archives. Downloads/hash verification are in progress at this entry.
- Added `AGENTS.md`, research context/schema and source semantics docs, ignored local environment configuration, single-episode smoke wrapper and optional model-construction validation utility. No upstream algorithm/controller changes; no chunk logger, metric, SE(2) conversion, or reconciliation.
- Static AST checks passed for benchmark, runtime/package models, behavior files, and validation utility; shell syntax passed. Ignore checks passed for local environment, generated output, caches, and weights. Fork/main and exact upstream SHA checks passed.
- Source findings: real head is 2D despite stale 3D comments; nominal 30 offsets at 10 Hz in current ego frame; synchronous action decoding with asynchronous reasoning; controller converts polyline to held/smoothed velocity commands rather than a timed waypoint queue. Frame/delay association and axis caveats are documented, not patched.
- Further installation/runtime results, final diff review, commit, and push will be appended below.

### Artifact installation and runtime baseline

- Completed official Isaac ZIP download (8,774,227,758 bytes, SHA-256 in SETUP) and extraction into new `~/isaacsim-ticvla-5.0.0`. Internal release version is `5.0.0-rc.45+release.23960.184afb15.gl`. Skipped the official desktop-icon overwrite step to preserve existing 6.x integration; created only the new runtime's examples symlink.
- A checkpoint transfer stalled; stopped only the task-owned downloader, preserved partial files, and resumed both weight transfers with bounded curl retry/low-speed timeout. Both final weight hashes match Hugging Face LFS metadata. Base weights: 1,876,463,472 bytes; checkpoint: 1,938,501,547 bytes. All 17 selected files total 3,827,163,803 bytes. No training dataset downloaded.
- Wrote portable `docs/artifacts.json` and external detailed manifest/source metadata. Model/cache files remain outside Git.
- New Isaac Python is 3.11.13. Bundled stack: torch `2.7.0+cu128`, torchvision `0.22.0+cu128`, torchaudio `2.7.0+cu128`, NumPy `1.26.0`, Pillow `11.2.1`. CUDA is visible; compiled architectures include `sm_120` for this GPU. `from isaacsim import SimulationApp` returned a callable API: PASS.
- Generated local constraints to retain the bundled torch family before resolving official inference extras. No conda/training/RL setup installed.
- This repository had no Git author configured; copied the established main-research repository's author identity into this repository's local Git config only. Global config and credentials remain unchanged.

### Dependency installation and model validation

- Reviewed the pip dry-run report before installation; it proposed no replacement of torch/torchvision/torchaudio/NumPy. Installed editable TIC-VLA and only official testing extras under the new Isaac runtime, with `requirements-test-isaac5.constraints.txt` preserving bundled binaries and pinning resolved inference versions.
- `pip check`: FAIL for bundled optional-package metadata conflicts/missing dependencies. Preserved the report; did not replace Kit USD or install unrelated packages to silence it. Required inference-module imports pass.
- First construction-check attempt stopped at package `ticvla` import because its VLM imports training data modules requiring `pytorch_lightning`. This is an upstream package-path dependency issue, not the runtime module used by DynaNav. Kept the failed log and changed only the additive validation utility to report this optional package failure while continuing the independent DynaNav check. No Lightning/training dependencies or upstream patches added.
- DynaNav runtime imports, selected Isaac interpreter/version, CUDA visibility and real CUDA matrix operation, config expansion, and artifact-path checks: PASS.
- Official base VLM plus TIC-VLA model construction: PASS. Released checkpoint with Nova Carter's exact prefix handling and `strict=True`: PASS. Chunk embedding `(30,512)`, head output weight `(2,256)`.
- Model-only torch peak allocated: 1,840.17 MiB; allocated after load: 1,820.21 MiB; reserved: 1,856 MiB. Device-wide 1-second sampled peak: 3,100 MiB including desktop. Evidence: `outputs/setup-20261009/model-construction-attempt2.log` and corresponding GPU CSV.

### Official smoke attempt and simulator diagnosis

- Ran the unmodified official launcher/config in child mode for `episode_hospital_smoke` only, run ID `smoke-20261009-hospital-01`, GPU 0, 900-second wall cap. No full benchmark ran.
- FAIL: native segmentation fault during `SimulationApp` RTX startup, before scene setup, behavior/model loading in that process, RGB input, prediction, or navigation. `python.sh` returned 1. Backtrace traverses `librtx.scenedb.plugin.so`, `libcarb.scenerenderer-rtx.plugin.so`, and `libomni.hydra.rtx.plugin.so`.
- A model-free, scene-free headless Isaac probe reproduced the same failure. A second headless probe without `CUDA_VISIBLE_DEVICES` also reproduced it; the mask warning was not sufficient to explain/fix the crash. Probes and logs remain under ignored setup output.
- Sampled device-wide peaks: official startup 1,490 MiB; Isaac-only startup 1,484 MiB. No CUDA OOM observed. Full Isaac-only ready state, scene+model memory, and inference memory were never reached.
- NVIDIA staff reports match the 595.xx/RTX startup failure family; documented exact links and treated driver/runtime incompatibility as the leading hypothesis, not a proven local driver A/B result. Did not alter driver/CUDA, substitute Isaac 6.x, change model precision/architecture, or patch upstream semantics.
- BLOCKED: simulator RGB -> action prediction -> navigation execution. This is not navigation-performance evidence or hard-case evidence. Before logger implementation, resolve compatible Isaac 5.0.0 RTX execution and repeat the single-episode smoke.

### Final review and Git delivery

- Existing Isaac VERSION, `python.sh`, and requirements SHA-256 checks: PASS, unchanged. System Python remains 3.12.3, nvcc remains 12.6, NVIDIA driver remains 595.84. Other research environments were not modified.
- Final shell syntax, Python AST, 17-file artifact-manifest sizes/hashes/portable paths, ignore rules, and `git diff --check`: PASS. Reusing the existing smoke run ID is rejected with exit 2; the previous console log checksum is unchanged. GPU returned to desktop baseline after diagnosis.
- Reviewed upstream diff: only `.gitignore` gains artifact/local-config exclusions. No changes to DynaNav, model, controller, configs, original environment files, or official requirements.
- Added files: `AGENTS.md`; `docs/RESEARCH_CONTEXT.md`; `docs/ACTION_SEMANTICS.md`; `docs/SETUP.md`; this work log; `docs/artifacts.json`; `requirements-test-isaac5.constraints.txt`; `scripts/research_smoke.sh`; `scripts/validate_runtime.py`. Ignored local file: `.env.testing.local`. Raw logs, simulator files, caches, and weights are excluded from the commit.
- Delivery is one focused commit, `Set up reproducible TIC-VLA hard-case research environment`, followed by normal `git push -u origin hardcase-probe`. No force push/history rewrite. The exact resulting commit and remote verification are recorded in the final task report and local `outputs/setup-20261009/git-finalization.txt`; the log uses this receipt to avoid a self-referential commit SHA or an extra bookkeeping commit.
