# TIC-VLA research workspace rules

This fork reproduces official TIC-VLA as a candidate source of successive navigation action chunks. The reconciliation method lives separately in `https://github.com/minju5054/se-3-reconciliation`.

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
- Do not implement graph optimization, reconciliation, correspondence, hard-case metrics, a successive chunk logger, or trajectory conversion until explicitly requested.
- Do not train or fine-tune TIC-VLA in this reproduction stage. Do not download training datasets for inference.
- Record exact upstream Git SHA and local Git SHA for every experiment, plus dirty status/diff identity when applicable, checkpoint identity, dependency versions, and commands.
- Use Isaac Sim **5.0.0** for this reproduction. Preserve existing Isaac Sim installations and other research environments. Do not modify system Python, global CUDA, NVIDIA drivers, or LightNav environments.
- Install only inference dependencies into the separate TIC-VLA Isaac runtime. Preserve NumPy `<2` and the Isaac PyTorch/CUDA stack; never install `requirements-train.txt` there.
- Follow each task with appropriate validation, diff review, an append-only `docs/WORK_LOG.md` entry, and a focused commit. Report PASS/FAIL/BLOCKED honestly; a process exit code alone does not prove inference or movement.

See `docs/RESEARCH_CONTEXT.md`, `docs/ACTION_SEMANTICS.md`, and `docs/SETUP.md` before changing this workspace.
