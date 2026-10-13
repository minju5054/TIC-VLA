# Exact original C10/C24, two grounded people in one scene

The user selected **one scene with two stationary humans** and the original
no-added-obstacle run `nova-e16-hospital-lights-baseline-20261010-01` as the
coordinate reference. This is not the later H05 graph's C10/C24.

- C10 root XY = `(6.824887275695801, 7.096357822418213)` m.
- C24 root XY = `(-1.7567275762557983, 10.988419532775879)` m.
- No horizontal offset, third H05 human, or added cart. Both people remain fixed.

Original Hospital world origin, XYZ metres, +Z up. Human yaw zero. Robot
quaternions w,x,y,z; logged human PhysX quaternions explicitly x,y,z,w.
USD skinning evaluates the default rest pose in skeleton space and applies the
skeleton-to-world row-vector matrix. The actual workboot sole vertices set
visual Z; root XY remains exact. Capsule radius .25 m and cylinder height1.2 m
are unchanged; its centre is .85 m above the floor so its bottom touches it.

The previous helper used a broad authored root bound of Z=-.1574196815 m,
lifting the visual by that amount. Model-free asset inspection found the
actual skinned shoe soles at world Z≈.157419658 m under that transform.
The new additive actor lowers only visual Z and the capsule centre. Historical
code/evidence/assets are preserved. No packages, weights or controllers change.

The portable runner requires a completed model-free preflight before a model
run can be prepared. Preflight checks exact authored XY, measured XY within
1 micrometre, both soles within1 mm of actual floor rays, source hashes,
stationarity, spawn/wheel/camera/light/render guards and local bypasses. It
also renders feet/body diagnostic views at frozen t=0 without model inference.
Review these before starting navigation. Physics settling is part of setup,
not navigation; no robot commands are issued during the diagnostic views.

In the TEST repository containing all reference data:

```bash
python3 -B tools/exact_two_humans/runner.py prepare --preflight
bash outputs/NEW-PREFLIGHT-SETUP/run.sh
python3 -B tools/exact_two_humans/runner.py prepare --prerequisite outputs/COMPLETED-PREFLIGHT-RUN
bash outputs/NEW-RUN-SETUP/run.sh
bash outputs/NEW-RUN-SETUP/analyze.sh
```

Use the fresh paths printed by prepare. `--gui` is available. No Git commands
are required. Every receipt freezes current sources, reference data, config,
model/runtime identities and prerequisite evidence. Existing paths are rejected.

Model execution remains the current modified-copy Isaac6.0.1 simulation plus
separate Isaac5 Python inference, 48 predictions, 2 Hz target, 60 Hz physics,
OLD retention and native forward/left outputs. It is not unchanged Isaac5
reproduction. Goal entry, passing each person, person/structure contacts and
runtime completion are separate results. The fixed loop continues after goal
entry. A single run and comparison to a historical path do not isolate causality.

`raw/humans_state.csv` has two measured people per physics tick. Legacy
`human_at_*` request fields explicitly contain a `humans` dictionary keyed by
C10/C24. Individual semantic masks are saved without affecting model RGB.
The model still receives the unchanged full camera RGB and official instruction.
