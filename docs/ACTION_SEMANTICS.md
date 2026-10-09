# Native action semantics: source audit

Source baseline: `ucla-mobility/TIC-VLA@9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Line references below refer to this exact revision. Runtime evidence is separately reported in [SETUP.md](SETUP.md). Released-checkpoint inspection and strict DynaNav loading passed: chunk embedding `(30,512)`, final output weight `(2,256)`, consistent with the default `(1,30,2)` prediction contract.

## Confirmed from executable source

| Property | Finding and source |
|---|---|
| Runtime model | DynaNav imports the sibling `DynaNav/ticvla.py`, via the behavior's parent-path insertion (Nova Carter lines 53–63). It is a different implementation from the package `ticvla/models/ticvla.py`. Check `module.__file__`; a package-only import does not validate the simulator path. |
| Tensor shape | `ActionExpert.mlp` ends in `Linear(..., action_dim)`; runtime constructor explicitly passes `action_dim=2`, default `num_action_chunks=30` (`DynaNav/ticvla.py:83–88, 211–215, 263–293`). General shape `(B,T,2)`; `predict_async` prepares batch 1, hence default `(1,30,2)`. |
| Components | Two position-offset components `(dx,dy)`. No action-head yaw, z, velocity, or timestamp channel. Runtime comments claiming `(B,T,3)` / `dtheta` are stale and contradicted by the actual layers and package implementation. |
| Origin | All future targets share the current observation/body frame origin. They are not incremental displacements from the preceding predicted waypoint. `policy_data.py:174–202` copies each future `offset` directly; `data/s01_batch_json_generation.py:354–372` computes each as `p_future - p_current`, rotated by the current orientation. |
| Cumulative/incremental | Cumulative position offset relative to one current ego origin. Both package `predict` and DynaNav `predict_async` return the action head directly, without cumulative summation (`ticvla/models/ticvla.py:678–681`, `DynaNav/ticvla.py:1074–1079`). Controller arc-length `cumsum` operates on lengths of differences, not on raw action offsets. |
| Frame/axes | Intended body FLU: x forward, y left, z up. Runtime controller labels `(T,2)` as body/FLU (`nova_carter_test_ticvla.py:1367`); input prompt also says forward/left (`DynaNav/ticvla.py:687–693`). This is the source convention, not a completed calibration of camera/robot/USD frames. |
| Units | Upstream treats offsets as meters: controller lookahead is 1, body state comments use m/m/s, and velocity limits use m/s (`nova_carter_test_ticvla.py:120–126,1316–1323,1370–1414`). No action normalization/unscale is applied along this path. Source data inherit pose/scene units; runtime stage units and release dataset calibration require verification. |
| Horizon | Default 30 points at target offsets 0.1…3.0 s. Package constructor explicitly defines 10 Hz, `action_horizon_steps=30`; training config agrees. Runtime allows `TICVLA_NUM_ACTION_CHUNKS`, but checkpoint embedding shape must match. Do not change it arbitrarily. |
| Rate | Data generator uses `DST_HZ=10`, `STEP_SEC=0.1`; behavior schedules every 3 updates under an assumed 30-frame/s benchmark convention (`nova_carter_test_ticvla.py:206–209,754–755`). Benchmark sets timeline time-codes/s to 30 (`benchmark.py:520–521`) and uses frame/30. This does not prove 10 predictions per wall-clock second or physical 0.1 s execution of every waypoint. |
| Yaw | Not in native action-head output. VLM reasoning text requests separate `(x,y,theta)` targets at 3/6/9 s; this text and the action tensor are distinct outputs. Controller `atan2(yL,xL)` is a derived bearing, not native yaw. |

## Nova Carter execution path

1. `benchmark.py` launches a fresh SimulationApp per episode, creates scene/people/robot, attaches the appropriate behavior, sets instruction/goal environment variables, and advances navigation frames. The official example contains four episodes; the research wrapper selects one via upstream child-mode flags.
2. Behavior captures front RGB, writes a JPEG every third update, and samples current plus available historical frames at 30 saved-image intervals (nominal -3/-6/-9 s). Current image is the last list element (`nova_carter_test_ticvla.py:754–806,1181–1242`). RGB render exposure time is not separately recorded.
3. `_inference_task` invokes `predict_async` **synchronously in `on_update`** (`863–880`, `1262–1266`, `1344–1356`). The file header's background action-thread/action-queue description is stale.
4. `predict_async` polls completed VLM reasoning, starts a new background generation when idle, uses the latest available KV state, extracts current-image visual features, and runs the action head. Inputs to the state encoder are `[vx,vy,yaw_speed,dx,dy,time_delay]`; behavior's `vz` input is explicitly dropped (`DynaNav/ticvla.py:1038–1074`). KV input comes from last-layer value tensors reshaped from `(B,heads,sequence,head_dim)`.
5. The controller converts the complete `(T,2)` polyline into a **single target `(v,w)`**: differences -> segment arc length; first index at 1 m arc length clipped to `[2,T-3]`; lookahead `(xL,yL)`; bearing `atan2(yL,xL)` filtered with 0.35 gain; curvature `2*yL/(xL²+yL²)`; speed/turn limits and angular feedback (`nova_carter_test_ticvla.py:1367–1414`). Arc length starts at the first predicted point, not at an added ego-origin point.
6. `_current_action` is replaced when the inference call returns, unless recovery/wait gates reject it (`1457–1474`). The remainder of that same update selects priority `backup > waiting > initial reasoning wait > model command`, slew-limits commands, passes `(v,w)` into `DifferentialController.forward`, and applies wheel actions (`967–1048`). Between predictions, the controller continues applying/smoothing the held target.

There is no timed queue executing each native waypoint and no OLD/FRESH reconciliation in this path. There is no explicit transform of a previous action chunk into the new observation frame in this controller.

Spot is a separate execution branch: it also calls the action head synchronously and derives a three-component command `(vx,vy,wz)` from two-dimensional waypoints (`spot_test_ticvla.py:974–1040,1387–1536`). Its checkpoint load uses `strict=False` with a warning/fallback path, unlike Nova Carter's `strict=True` (`400–426`). A Spot process starting is therefore insufficient evidence of released-checkpoint loading. This setup targets the official Nova Carter example and does not claim Spot validation.

## Reasoning delay and timing

- VLM reasoning uses `ThreadPoolExecutor(max_workers=1)`. It generates text and then runs another forward pass over the full input-plus-response conversation to extract KV state (`DynaNav/ticvla.py:651–783`).
- `_start_kv_cache_generation` stores submission `perf_counter`, caller's benchmark step, and a copy of the caller-sampled pose before executor submission (`787–840`). The comment saying “exact moment” does not establish synchronized sensor exposure or worker execution time.
- `_poll_kv_cache_future` records the **polling step** when a completed result is noticed, updating the latest KV state (`843–888`). It does not record an exact background completion timestamp.
- The first `predict_async` blocks on the future for up to 30 wall seconds (`978–1032`). Start/completion may have the same benchmark frame while wall latency is nonzero. Subsequent calls use older available reasoning while new reasoning runs.
- Behavior retains up to two reasoning generation-start poses/frames. Before the next model call it uses entry `[0]` to compute `(current_frame-reference_frame)/30` and `R_reference.T @ (p_current-p_reference)` (`nova_carter_test_ticvla.py:808–837,927–952`). This is input state compensation, not a geometric transform of the output chunk.
- Reasoning start events, the KV state actually consumed, and delay-reference entries are not linked with durable IDs. Bootstrap, completion polling, failed generation, and recovery can make their relationships nontrivial; verify these relationships during future collection.
- Action-head completion and command application occur inside the synchronous update, but GPU work completion, sensor capture, simulation advancement, and wall time are not interchangeable. Do not claim hard real-time 10 Hz from source comments.

## Coordinate caveats confirmed in source

- Nova Carter gets world pose/quaternion, rotates world velocity with `R_IB.T`, then explicitly negates lateral velocity `lin_vel_b[1]` (`1076–1106`). Displacement conversion does not show the same sign flip. Record this existing operation; do not silently “correct” it or assume all state tokens share one verified frame.
- `_get_pose_R_quat` returns the raw upper-left USD matrix while other transforms use `quat_to_rot_matrix`. Row/column-vector conventions and robot base/camera extrinsics must be checked before any future world projection.
- Data-generation comments describe an optical x-right/y-down/z-forward camera, but the actual configured matrix maps `(-z_cam,-x_cam,+y_cam)` to FLU (`data/s01_batch_json_generation.py:31–53`). This mismatch is another reason to preserve raw data and verify actual pose conventions rather than copying a comment.

## Interpretation for this research

Successive native position-offset predictions and changes in reasoning context are plausible observation points for future handoff analysis. The execution interface is a stateful velocity controller with smoothing and recovery, so “committed OLD” cannot simply mean an index prefix of a waypoint queue. No measured geometric/temporal hard case is established by this source audit.

## Not yet verified by this source audit

- Actual prediction values/shapes during a scene run, beyond the verified checkpoint structure (see runtime result in SETUP).
- Scene meters-per-unit, camera extrinsics, robot axes, and the effect of the lateral-velocity sign convention.
- Actual sensor exposure, physics timestep vs benchmark update count, wall-clock inference cadence, and exact command-to-motion latency.
- Validity of reasoning-state/delay associations in every transition.
- Existence/severity of qualifying OLD/FRESH cases or navigation performance.

No conversion into `[x,y,yaw]` is implemented.
