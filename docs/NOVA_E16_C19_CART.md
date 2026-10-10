# One fixed Hospital cart at the baseline C19 position

**CART REVEAL TIMING NOT RECONCILIATION-RELEVANT.** Physical placement passed
and exactly one real model run completed. The cart was already partly visible
at C1 and first CLEAR at C5. Primary C4→C5 predicts no cart conflict in either
chunk. The robot subsequently takes the north side and contacts Hospital
structure; this is not evidence of reconciliation improvement.

Starting clean local/fetched origin `hardcase-probe`:
`1a1355888e492913d6ea7f9a1bf1287b5e332d6c`; fetched upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Reference:
`outputs/nova-e16-hospital-lights-baseline-20261010-01`. Previous no-human
and both manual human outputs remain unchanged.

## Asset and immutable placement

Source `/Root/SM_SupplyCart_01e_21`, asset
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Environments/Hospital/Props/SM_SupplyCart_01e.usd`.
One reference copy: `/World/E16Cart`. Original material
`Looks/MI_SupplyCart_02a` and original mesh/collision are retained. Collider
`SM_SupplyCart_01e` uses CollisionAPI and triangle-mesh approximation `none`;
it is static, with no dynamic rigid body or replacement proxy collider.

| Quantity | Value |
|---|---|
| Baseline C19 observation time | 10.300000537186861 s |
| Baseline C19 SE(2) | (1.9812549352645874, 11.180365562438965, 3.030531579778966) |
| Frozen footprint centre XYZ | (1.9812549352645874, 11.180365562438965, -2.384185791015625e-07) m |
| Frozen yaw | 3.032406244735343 rad |
| Local bounds min | (-0.46186652104854886, -0.2624729097826872, -1.496198998029996e-17) m |
| Local bounds max | (0.6399123239536806, 0.2649667299173757, 1.1127104701240995) m |
| Dimensions X × Y × Z | 1.1017788450022294 × 0.5274396397000629 × 1.1127104701240995 m |
| Original world translation | (-13.634139404296874, 7.36949462890625, 0.0010006103515625142) m |
| Original world bounds min | (-14.096005925345423, 7.107021719123563, 0.0010006103515624992) m |
| Original world bounds max | (-12.994227080343194, 7.634461358823626, 1.113711080475662) m |
| Copy world bounds min | (1.4049085723685633, 10.858185977161623, -2.3841857910155914e-07) m |
| Copy world bounds max | (2.5576012981606104, 11.502545147716301, 1.1127102317055204) m |

Original root rotation/scale are identity. Long local X is aligned with the
saved C18→C20 measured XY tangent. Z comes from the actual GroundPlane floor
raycast. The root pivot is offset from the footprint centre; the frozen
matrix compensates for it. World metres, +Z up, yaw CCW from +X; recorded
body quaternions are w,x,y,z and USD matrices act on row vectors. No position
or yaw search, post-result adjustment, motion, hiding or teleporting.

Audit: `outputs/e16-cart-audit-20261010-01/asset.json`. Physical/visibility
preflight: `outputs/e16-cart-preflight-20261010-01`. Its `freeze.json` SHA256
is `63d4b86570a4af743ddbb7b8106237079ba5567261e04ea362250f6f131f4a09`.
Source prim signature, mesh arrays, material binding, both USD byte hashes,
frozen transform and original source immutability are checked at authoring.

## Preflight and single run

Floor support, no static overlap, initial separation and local bypass all
**PASS**. Initial robot/cart clearance is **+9.953998 m**. The baseline path
intersects the inflated oriented footprint: minimum **−0.870702 m**, first
entry **9.525052 s**. The first feasible sampled bypass is on the south side,
with cart clearance **+0.050000 m** and an additional 0.05 m static margin.
The unchanged enclosing Nova disk radius is **0.6069825421953869 m**.
These are local geometric feasibility checks, not a navigation guarantee.

All 48 saved full camera poses were checked at 1920×1080 with the existing
fixture lights and semantic criterion: HIDDEN=0 pixels; CLEAR=fraction≥.002,
bbox width≥20 and height≥80; other positive masks=MARGINAL. Preflight:
**C1–C4 MARGINAL, C5–C19 CLEAR, C20–C48 HIDDEN**. C1 has 1,971 cart pixels.
Visibility was diagnostic and did not block the physically valid experiment.

Exactly one run: **`outputs/nova-e16-cart-20261010-01`**, 48 finite native
(30,2) predictions and 1,565 measured poses, t=0–26.066668026149273 s.
Strict released checkpoint and continuous OLD execution pass the existing
validator. All baseline config fields are identical except `stationary_cart`:
instruction/start/goal, scene/lights, Nova/Hawk, controller, 2 Hz target and
60 Hz physics/control remain unchanged. The cart exists before reset/settle;
1,565 logged world-transform checks confirm it stays fixed. The actor log is
`raw/cart_state.csv`; legacy `human_at_*` request fields explicitly identify
the cart and unchanged USD static-collider transform, not a measured dynamic
PhysX rigid-body position. Model inputs are original RGB without overlays.

Actual visibility has the same C ranges; C1=1,969 pixels and C5=5,160 pixels.
First CLEAR is C5 at **4.066666878759861 s**. Primary **C4→C5** follows that
event, without later favourable substitution. All 47 adjacent pairs are
exported; none changes from OLD cart conflict to FRESH clear.

## Actual geometry and outcome

Own-observation projection is `world = observation_xy + R(yaw) @ native`.
Native axes are forward/left metres. Target timing is assigned from the
source-confirmed +0.1…3.0 s convention; neither yaw nor timestamps are native
tensor channels. Common absolute nominal times use linear interpolation
inside both curves, without extrapolation or spatial matching. C4→C5 has
gap 0.500000026077032 s and 25 common points at approximately +0.1…2.5 s
after C5. OLD remaining is clipped at FRESH observation time.

| C4→C5 metric | Value |
|---|---:|
| OLD remaining / FRESH cart clearance | +6.893210 / +6.206886 m |
| Current robot/cart clearance at C5 | +8.280966 m |
| OLD predicted conflict time | none |
| Aligned RMSE / max displacement | 0.397823 / 0.493824 m |
| Mean / max absolute lateral revision | 0.017280 / 0.050131 m |
| Mean signed lateral revision | −0.008135 m |
| Common-horizon endpoint revision | 0.493824 m |
| Mean / max absolute derived tangent change | 29.367190 / 143.720156° |
| Command Δv / Δw | 0 m/s / +0.010019857 rad/s |
| Observation→application, simulation | 0.116666673 s, 7 ticks |
| Action call / observation→switch, wall clock | 0.078718213 / 0.555015241 s |
| Robot XY / yaw transport | 0.175127223 m / +0.401242260° |
| RAW boundary position gap | 0.036156120 m |
| RAW executed→FRESH tangent gap | 0.623385030° |
| Robot heading→FRESH tangent gap | 0.493931450° |

Revision is predominantly forward progress; small or reversing geometric
segments make derived tangent differences large. These are not native yaw,
executed heading jumps or evidence of a large lateral avoidance revision.
RAW seam uses the existing saved execution tangent and interpolated FRESH
curve at application, with a derived observation-origin anchor; no correction
or latency compensation is applied.

Actual minimum cart clearance proxy is **−0.023660693 m**; first proxy entry
is **10.836022034 s**. There are **0 recorded cart contact spans**. The signed
proxy uses enclosing cart bounds plus an enclosing robot disk, so its negative
value is not physical penetration evidence. There are **268 other non-floor
contact spans**, first at **11.383333927 s** against
`/Root/Geo_M_DoorFrame52/Geo_M_DoorFrame/Geo_M_DoorFrame`. Later wall/door trim
contacts accompany the robot remaining near final XY (0.84612745,12.14828587).
PhysX emits repeated material-face-index warnings during this contact period;
the saved contact reports include nonzero impulses. A completed 48-prediction
run is not successful goal completion or a navigation performance claim.

## Figures, replay and verification

Final derived directory: **`outputs/e16-cart-analysis-20261010-02`**.
`trajectory_result.png/.pdf` and `trajectory_turn_zoom.png/.pdf` use world
metres/equal XY, both actual paths, structural bounds, exact cart/inflation,
all actual C markers, OLD/FRESH and application. Labels C1–C21 and C48 avoid
the late stalled-pose cluster; markers never move. Primary C4/C5 lie outside
the corner zoom. `trajectory_revision.png/.pdf` describes measured nominal
displacement and signed components, explicitly marked non-qualifying; it does
not depict a fabricated qualifying response. `trajectory_coordinates.json`,
`observation_poses.csv`, `pair_metrics.csv/.json` and `pair_details.json`
export all coordinates and metrics. Analysis -01 remains preserved.

```bash
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/analyze_e16_cart.py --run-dir outputs/nova-e16-cart-20261010-01 --preflight-dir outputs/e16-cart-preflight-20261010-01 --output-dir outputs/e16-cart-analysis-REVIEW
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/e16-cart-analysis-20261010-02/viewer_data --output-dir outputs/e16-cart-gui-REVIEW
```

Use a fresh output name. The saved-data viewer opens paused at t=0/C1, with
Restart, Play/Pause, 0.25×/0.5×/1×/2×, slider, Previous/Next C, first cart
visibility, primary OLD/FRESH, switch, Overview, Follow and front Hawk.
Original archived model RGB is the panel default; mask overlay is optional.
Cart remains fixed throughout; model calls=0 and physics reexecution=false.
Validated replay: **`outputs/e16-cart-gui-20261010-01`**, GUI_READY at t=0/C1.
All **262 PNG frames**, 1600×1000 at 10 fps, cover **0–26.066668026149273 s**,
including the exact final saved pose and over two seconds after interaction.
Every frame decodes and its XYZ exactly matches its saved tick; maximum
sample-and-hold pose age is **0.016666662320494657 s**. All 48 original model
observations occur in the export. `frames.json` and `video.json` record the
mapping and absent ffmpeg; no MP4 or package installation. Original RGB is
held at its separately labelled observation timestamp. Overview/follow/Hawk
views and representative exported frames were visually reviewed. Legacy
self-test `human_present`/`replay_keeps_human_fixed` keys refer to the cart in
this adapter; no human is instantiated. No conflict marker is fabricated.

Seven focused tests pass: six cart tests plus the existing exact observation
plot test. Independent checks confirm all 96 preflight/actual masks, all
48 plotted observation poses, 658 protected source files, unchanged baseline
config, native contracts and every static-cart tick. PNG/PDF figures were
visually reviewed. `outputs/e16-cart-setup-20261010-01` holds validation logs.

Runtime config SHA256:
`c119a56a24d7850f25a6fdf8a7c3564e67e4ea945abb72702b7b083b99852ba8`;
receipt SHA256:
`cc8d0fc6be931c0002049ea4dac16fbbca3ca0bfd6b4690d7e457e0256b6a416`.
The historical receipt has a `source_sha256` name collision with code
provenance: it does not independently bind the entire preflight directory.
Its `code_sha256`, config hash, embedded asset/source hashes remain intact.
After the single run, the wrapper was fixed to use `source_evidence_sha256`
and a regression test added. No receipt/raw data was rewritten and no model
rerun occurred. Executed wrapper and pre-label-review analyzer are archived
under setup; their hashes verify against the original receipt. Plot labels
and contact summary were improved only after inference completed.

One placement and one run support the decision above. No threshold retuning,
causal cart-effect attribution, latency-induced hard-case claim, automatic
correspondence, graph optimization, reconciliation or improvement claim.
