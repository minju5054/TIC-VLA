# Modified-copy H05 runtime smoke

Copy this entire folder to `tools/h05_api_smoke/` in a TIC-VLA copy, then **prepare there**. Do not copy old authorization receipts as new permissions or results. Existing research code, configuration, historical outputs, model files and Isaac installations are not modified. No Git checkout or Git command is required by this folder.

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
python3 -B tools/h05_api_smoke/runner.py prepare
```

Preparation prints a fresh `outputs/h05-api-smoke-…-setup` directory. It contains `config.json`, `receipt.json`, `verify.sh`, `run.sh`, and `analyze.sh`. Preparation and verification read source files, reference data and model bytes; they do not import Isaac, load model weights, render or advance physics. Use `prepare --gui` to explicitly select a GUI run; the default preserves historical headless mode.

```bash
# Substitute the exact setup directory printed by prepare.
bash outputs/h05-api-smoke-NEW-setup/verify.sh
bash outputs/h05-api-smoke-NEW-setup/run.sh && bash outputs/h05-api-smoke-NEW-setup/analyze.sh
```

`run.sh` starts a bounded 900-second Isaac 6 process with the modified copy's renderer selection and separate Isaac 5 model inference. It runs the existing 48-prediction, 2 Hz H05 controlled harness, not the DynaNav IRA entry point. Source and model identities, episode semantics, robot/camera/wheel geometry, human placement, lighting, completed render and measured PhysX pose checks remain mandatory. It preserves the model, controller and scheduling implementations. The original H05 pre-model checks are retained, with historical source-byte equality replaced by a check against the newly frozen current source; geometric fields still compare to the saved baseline.

The new configuration refreshes the episode's source hashes without changing its actual scene, start, goal, instruction, yaw or people count. A semantic change is rejected. Current renderer selection is recorded explicitly, and equivalence with historical rendering is **not established** by preparation. Git identity is not invented for a directory without `.git`; a content manifest identifies the copy.

`receipt.json` uses repository-relative paths for local code/reference files. Existing external model and Isaac paths remain absolute and are hashed. Copying this tool folder alone is supported; its underlying research modules and historical geometry reference files must also exist. If the prepared session and repository move together with identical contents, verification resolves local paths in the new repository. Source edits require a new preparation. The historical source files and selection are never rewritten to disguise their provenance.

An attempted run cannot reuse its output directory or console log. Both the launcher exit status and a completed runtime result are required for launcher success. Native crashes may leave only the session's `console.log` and `process-result.json`; inspect those before analysis. Analysis never starts the simulator. It requires completed real inference and checks saved native request/clock/OLD-retention contracts, raw hashes and measured motion. It writes `derived/api-smoke/summary.json` once. Runtime success and goal entry are separate fields; this is not an avoidance qualification or an unchanged Isaac 5 reproduction.

No historical `outputs/` directory should be deleted to make a test run. Prepare a fresh session instead.
