# Separate stationary obstacles near C24

Two independent scenes: one added stationary human OR one added static Hospital
cart. They are never instantiated together. The user requested this separation
and fixed actors; only the robot moves. Original Hospital props remain unchanged.

Both footprint centres use previous successful modified-copy C24 X and C24
world Y minus 0.65 m: `(-1.541121244430542, 10.704311943054199, 0)` metres.
This leaves a local bypass on the north side. Cart long axis is parallel to
world X, pointing west; the human retains yaw zero. World origin is the original
Hospital origin, +Z up, yaw CCW from +X, recorded robot quaternion w,x,y,z.
USD local-to-world matrices act on row vectors. These are scene transforms,
not invented native action yaw. The existing native `(forward,left)` outputs,
controller, weights, 2 Hz target and 60 Hz physics/48-prediction loop are unchanged.

This is the TEST copy's Isaac6 renderer/custom wrapper with a separate Isaac5
Python model service. It is not unchanged Isaac5 reproduction. The previous H05
path had a human at another position; it only supplies descriptive reference
geometry, not a matched obstacle-free or deterministic causal control.

From a copy containing the prior successful H05 run and reference artifacts:

```bash
python3 -B tools/h05_stationary_obstacles/runner.py prepare --scenario human
python3 -B tools/h05_stationary_obstacles/runner.py prepare --scenario cart
```

Each command prints its exclusive setup directory. For each setup independently:

```bash
bash outputs/NEW-SETUP/verify.sh
bash outputs/NEW-SETUP/run.sh
bash outputs/NEW-SETUP/analyze.sh
```

Use fresh IDs; do not reuse or edit receipts. `--gui` is available during prepare.
No Git command is required. Source/weights/reference/config hashes are frozen.
Robot/camera/light/render gates and actual scene collision/floor queries run
before inference. Each scene must have a sampled connected local bypass and
intersect the previous path's conservative robot disk. Queries do not control
the robot. A failed gate prevents model loading.

Human position is checked from PhysX every tick. Cart static USD world transform
is checked every tick; its original collision mesh is retained without a dynamic
rigid body. Saved actor/robot ticks must match. Contact-free passage, goal entry,
and mere runtime completion are separate. Conservative bounds/disk overlaps do
not prove penetration. The loop continues after goal entry; evaluation reports
contacts through first goal entry and totals separately. Saved RGB GIFs hold
actual observations at their simulation timestamps; they are not full videos.
