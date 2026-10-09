# TIC-VLA research workspace rules

This fork is the home of the user's TIC-VLA-based research: official reproduction, DynaNav experiments, future successive chunk collection, hard-case analysis, and the future reconciliation method. `https://github.com/minju5054/se-3-reconciliation` contains prior LightNav-based research; do not modify it as part of this workspace's tasks. Keep official upstream code distinguishable from research additions.

- Preserve official TIC-VLA behavior. Prefer environment configuration, ignored local environment files, and additive wrappers before minimal upstream patches.
- Document every upstream source change: file, reason, behavior difference, and possible research impact. Do not refactor unrelated code.
- Work on `hardcase-probe`, not `main`. Never force push or rewrite history.
- Never commit model weights, checkpoints, Hugging Face caches, large datasets, Isaac caches, recordings, or generated experiment outputs.
- Never overwrite previous raw experiment output. Use a new run ID and reject existing run directories. Keep raw and derived artifacts separate. Upstream image buffers are temporary and are deleted; do not present them as an archival dataset.
- Do not infer coordinate frames from variable names. State axes, origins, quaternion ordering, transform direction, units, and timing explicitly for every coordinate transform.
- Distinguish observation time, action inference start/ready time, reasoning generation start/ready/poll time, and command switch/execution time. Simulation frame counts and wall-clock timestamps are different clocks.
- Do not call native TIC-VLA `(dx, dy)` output an SE(2) trajectory. Do not invent yaw and present it as native data.
- Distinguish navigation failure from OLD/FRESH handoff inconsistency, TIC-VLA performance validation from reconciliation mechanism validation, and simulation from real-world evidence.
- Do not describe synthetic data or static source inspection as experimental evidence.
- The minimal closed-loop harness and append-only request/state records in `research/` are explicitly authorized. Do not expand them into graph optimization, reconciliation, correspondence, hard-case metrics/ranking, waypoint yaw, or trajectory conversion until requested.
- Do not train or fine-tune TIC-VLA in this reproduction stage. Do not download training datasets for inference.
- Record exact upstream Git SHA and local Git SHA for every experiment, plus dirty status/diff identity when applicable, checkpoint identity, dependency versions, and commands.
- Preserve Isaac Sim **5.0.0** as the official reproduction baseline. The user's Isaac **6.0.1** compatibility investigation is authorized separately; do not represent it as an unchanged Isaac 5 reproduction. Preserve both installations and other research environments. Do not modify system Python, global CUDA, NVIDIA drivers, or LightNav environments.
- Before using Isaac 6, read `docs/ISAAC6_COMPATIBILITY.md`. Gate DynaNav work on a successful simulator-only startup. Small API adaptations require explicit evidence of equivalent behavior; do not silently replace the human scenario system or controller.
- Current execution uses the separately authorized controlled harness in `docs/MINIMAL_CLOSED_LOOP.md`, not a DynaNav port. Run Isaac 6 simulation separately from Isaac 5 Python model inference (without a second SimulationApp); preserve the official model and document wrapper/state/timing differences. Gate real inference on wheel/camera checks, static control on inference, and the single scripted human on static closed-loop success.
- The additive continuous path in `docs/CONTINUOUS_HANDOFF.md` uses one outstanding IPC future and main-thread physics with OLD command retention and real-time pacing. Preserve the default frozen config/path and prior geometry conclusions. Gate continuous dynamic execution on continuous static PASS; distinguish measured action-call latency from background reasoning age and keep FRESH anchored to its observation pose.
- Capture current RGB with explicit render completion and no physics advance. A nonempty image alone does not prove alignment with the observation pose. Pedestrian motion evidence must distinguish authored targets from measured PhysX poses.
- Install only necessary inference dependencies after inspecting the selected runtime and a pip dry-run. Preserve the existing torch/torchvision/torchaudio, CUDA, USD, and Isaac/Kit binaries and NumPy major version. Isaac 5's NumPy `<2` requirement must not be applied blindly to Isaac 6. Never install `requirements-train.txt` in either inference runtime.
- Follow each task with appropriate validation, diff review, an append-only `docs/WORK_LOG.md` entry, and a focused commit. Report PASS/FAIL/BLOCKED honestly; a process exit code alone does not prove inference or movement.

See `docs/RESEARCH_CONTEXT.md`, `docs/ACTION_SEMANTICS.md`, `docs/SETUP.md`, and `docs/MINIMAL_CLOSED_LOOP.md` before changing this workspace.
