# Separate C24 stationary human and cart trials

**Human-only: contact-free passage and goal entry. Cart-only: navigation failure
at Hospital structure before reaching the cart.** Both added objects remained
fixed. Two separate processes/scenes, 48 predictions each, no retry or tuning.

The user requested separate human/cart scenarios near the previous C24 and
explicitly required both obstacle types to stay stationary. An initial combined
scene interpretation was corrected before any new simulator or model startup.
No combined scene was executed. Each final scene contains exactly one added
obstacle; original Hospital furnishings are preserved.

## Placement and execution scope

Execution repository: `/home/gpuadmin/Workspace/TIC-VLA-TEST` (no Git directory).
The additive tool is `tools/h05_stationary_obstacles/`, developed in the original
`hardcase-probe` workspace and copied without changing existing source files.
Original local SHA at preparation: `d3ae80ba46ea799ae0491a930cfa12f7575fea02`. The preparation
provenance is in `outputs/c24-stationary-preparation-20261013T141828Z-d330fdf4/provenance.json`.
Upstream SHA: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. That provenance also
records dirty status, tracked-diff SHA256 and every added tool's content SHA256.
The TEST receipts independently freeze all current runtime source bytes.

Reference: `h05-api-smoke-20261013T134532Z-2e68f7e7`, whose previous human was at
another position. It supplies the successful measured path and C24 placement
reference; it is not an obstacle-free or deterministic paired control.

Both authored footprint centres are **(-1.541121244430542,
10.704311943054199, 0) m**: reference C24 X and C24 world Y minus **0.65 m**.
The offset leaves a north-side bypass while obstructing the previous path's
conservative robot footprint. Human yaw remains zero. Cart's long horizontal
axis is parallel to world X, with its positive long axis pointing west.

Coordinates use the original Hospital origin, XYZ metres, +Z up, yaw CCW from
+X. Robot quaternions are w,x,y,z; cart USD matrices transform local row vectors
into world coordinates. Cart pivot compensation preserves its true footprint
centre and floor support. Native model outputs remain forward/left offsets;
no native yaw or SE(2) representation was introduced.

The human uses the existing kinematic capsule and visual asset; measured PhysX
position is checked every tick. Cart is one reference copy of
`/Root/SM_SupplyCart_01e_21` at `/World/E16Cart`, with original material and
triangle-mesh collision, no dynamic rigid body. Its full static USD world
transform is checked every tick, which is not a dynamic PhysX pose measurement.
No movement targets, animations, triggers or runtime pose updates are issued.

The two configs are identical after removing their one actor field. Both retain
the successful copy's `RealTimePathTracing`, front Hawk 1920×1080, Hospital
lighting, start/instruction/goal, controller and strict released weights.
Isaac6.0.1-rc.7 runs physics at 60 Hz and target inference at 2 Hz; separate
Isaac5 Python3.11.13 runs model-only inference without a second SimulationApp.
Torch2.7.0+cu128, torchvision0.22.0+cu128, NumPy1.26.0 and transformers4.57.6
are recorded. No dependency, driver, installation, model or upstream change.
This is a modified-copy custom-wrapper test, not unchanged Isaac5 reproduction.

Both scenes passed pre-model robot/wheel/camera/spawn/lighting, completed-render
without physics advance, floor, nonoverlap and connected local bypass checks.
The geometry probes are sampled conservative feasibility checks, never robot
commands or model outputs. Scene queries confirmed no added cart in human-only
and no added human in cart-only. The continuous controller/OLD retention and
observation anchoring are unchanged.

## Measured outcomes

Run IDs share suffix `20261013T141828Z-d330fdf4`:

| Measurement | `c24-fixed-human-…` | `c24-fixed-cart-…` |
|---|---:|---:|
| Runtime / native predictions | PASS / 48 | PASS / 48 |
| Matched robot and actor ticks | 1,522 | 1,543 |
| Maximum added-obstacle displacement | 0 m | 0 m |
| First visible / first CLEAR observation | C11 / C11 | C11 / C12 |
| Passed west beyond obstacle footprint | 13.333334 sim s | No |
| Strict goal distance <2 m entry | 18.683334 sim s | No |
| Closest goal distance | 0.441967 m | 14.218029 m |
| Added-obstacle contact reports | 0 | 0 |
| Other nonfloor contact spans | 0 | 14 |
| First nonfloor contact | None | 10.683334 sim s |
| End of recorded simulation | 25.350001 s | 25.700001 s |
| Scenario evaluation | PASS_THIS_SCENARIO | FAIL_THIS_SCENARIO |

Human motion is measured as exactly zero across all PhysX position samples.
Its measured Y differs from the authored double by about 0.38 micrometres due
to the stored physics representation; this is not motion. Minimum sampled
human/robot enclosing-disk clearance is +0.124583 m. Cart transform remains
exactly fixed; minimum cart-bounds/robot-disk clearance is +2.469478 m because
the robot never reaches it. These proxies are not exact physical separation.

The human run passes north of the person and enters the goal region without
recorded contacts. The unchanged 48-prediction loop then continues beyond the
goal: final goal distance is 7.990530 m. Goal entry does not imply stopping.

In the cart run, C12 already contains 7,224 visible cart pixels; C12–C19 are
CLEAR. The robot keeps a 1.5 m/s forward target while its left-turn command at
C14/C15 is only approximately +0.067/+0.045 rad/s. Measured motion reaches
the north-side Hospital door/wall before completing the turn. The first
contact is between the right wheel and `/Root/SM_Door_01b8/SM_Door_02b` at
10.683334 sim s; subsequent contacts include doorframe, trim and wall geometry.
Recorded nonzero impulses (maximum norm 42.177623) and negative separation
(minimum -0.006881 m) accompany the stall. There are 14 contact *spans*, not
a claim of 14 independent collisions. C20 onward RGB is heavily obstructed
by nearby geometry. It is a failure to navigate to and around the cart,
not evidence of hitting the cart itself.

This pair does not prove the cart caused the turn error: each scene was run
once, the original model is stochastic/asynchronous, and no matched empty
scene was rerun. No handoff correction, optimization or reconciliation method
was added. Native predictions, inferred commands and measured motion remain
distinct in the saved records.

## Saved artifacts and commands

Each TEST run has `derived/fixed-obstacle/summary.json`, `contacts.json`,
`trajectory.png`, `rgb-contact-sheet.png` and `rgb-observations.gif`. GIFs contain
the 48 actual saved model-camera observations, downscaled and held according
to simulation timestamps (25.35/25.70 s); no intermediate frames are invented.
The trajectory comparison labels the previous H05 path as descriptive.

Executed commands, grouped by scenario below. Simulator processes ran sequentially;
human saved-only analysis ran during cart startup:

```bash
cd /home/gpuadmin/Workspace/TIC-VLA-TEST
bash outputs/c24-fixed-human-20261013T141828Z-d330fdf4-setup/run.sh
bash outputs/c24-fixed-human-20261013T141828Z-d330fdf4-setup/analyze.sh
bash outputs/c24-fixed-cart-20261013T141828Z-d330fdf4-setup/run.sh
bash outputs/c24-fixed-cart-20261013T141828Z-d330fdf4-setup/analyze.sh
```

These IDs are consumed. Reexecution requires new preparation and IDs, using
the README's `prepare --scenario human` / `prepare --scenario cart` commands.
Both bounded run processes exited zero and produced validated model/motion
records; that runtime success is separate from the cart's navigation failure.

Fifteen focused CPU tests passed. Saved-only independent verification checks
all 96 finite native arrays and original RGB images, actor/robot tick matching,
zero motion, strict-goal distances, distinct single-actor assets, GIF decoding,
and identical common configs. Each run's 392 raw hashes and 163 current source
hashes verify. All 456 prior reference files and 163 preexisting TEST source
files remain unchanged. Both plots and RGB sheets were visually reviewed.
Full validation and cart contact diagnosis are in the original workspace's
preparation audit directory. All generated evidence is ignored; only additive
tools/tests and documentation are committed. No further trial was executed.
