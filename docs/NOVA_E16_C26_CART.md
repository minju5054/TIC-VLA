# Fixed C26 Hospital cart: preflight stop

**No inference was run.** The cart is supported and separated from Hospital
colliders, but neither side passes the unchanged conservative local bypass
check. The prescribed stop gate therefore fails. Placement was not moved or
retuned. Saved-camera inspection and no-cart baseline geometry analysis were
completed; neither is a cart-conditioned model response.

Starting clean local/fetched origin `hardcase-probe`:
`c408dbefa0b3a859ca2bb61701278bb9ff2a4a17`; fetched upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Source:
`outputs/nova-e16-hospital-lights-baseline-20261010-01` (48 finite predictions).

## Frozen placement and physical gate

| Quantity | Exact value |
|---|---|
| Baseline C26 observation time | 13.800000719726086 s |
| Baseline C26 SE(2) | (-3.2318756580352783, 10.711772918701172, -2.9310779489526193) |
| Cart footprint centre XYZ | (-3.2318756580352783, 10.711772918701172, 3.8743019104003906e-07) m |
| Cart yaw / C25→C27 tangent | -2.9354307603346976 rad |
| Pivot-compensated root translation | (-3.1449931715228283, 10.731216820191438, 3.8743019105500105e-07) m |

One `/World/E16Cart` reference to original `/Root/SM_SupplyCart_01e_21`,
`Hospital/Props/SM_SupplyCart_01e.usd`. Original mesh, material, scale and static
triangle collision are preserved. The longest local horizontal axis is X;
its 1.1017788450022294 × 0.5274396397000629 m footprint uses the exact C26
centre, with Z from the actual floor query. World metres, +Z up, yaw CCW from
+X; body quaternions w,x,y,z; USD matrices act on row vectors.

Preflight: `outputs/e16-c26-preflight-20261011-01`. Floor support, zero cart/
static overlap and initial clearance **+12.910207 m PASS**. Original executed
path first enters the inflated proxy at **13.028501776230325 s**, minimum
**−0.870702362 m**. Proxy = cart oriented bounds + unchanged enclosing Nova
disk radius **0.6069825421953869 m**.

| Side | Tested strips | Passing strips | Nearest strip cart clearance | Structural-bound clearance |
|---|---:|---:|---:|---:|
| North/right | 10 | 0 | +0.050000 m | −0.188301 m |
| South/left | 10 | 0 | +0.050000 m | +0.039392 m |

Existing checks use 41 points per strip, additional **0.05 m static margin**,
and offsets in the same prescribed .05… .50 m range. They vary robot-centre
strips, not cart placement. North intersects DoorFrame51 and adjacent wall/
handrail geometry. The nearest south strip fails against
`/Root/Geo_M_WallHandBase2/Geo_M_WallHandBase/Geo_M_WallHandBase`; its positive
AABB clearance is smaller than the unchanged .05 m margin. Wider strips also
fail. This is **no feasible bypass demonstrated by the existing conservative
check**, not a proof that every possible continuous path is impossible.
The cart itself is validly supported; the overall physical experiment gate
is **INVALID (local bypass FAIL)**. The gate is not relaxed after inspection.

## Saved visibility and camera investigation

All 48 original full camera poses, fixture lights and 1920×1080 resolution
were reused with stopped physics. Semantic criterion unchanged: HIDDEN=0;
CLEAR=fraction≥.002 and bbox≥20×80 pixels; other positive masks=MARGINAL.

- **C1–C10 HIDDEN; C11 MARGINAL; C12–C26 CLEAR; C27–C48 HIDDEN.**
- C1=**0 pixels**. First visible C11, **6.300000329 s**, 1,446 pixels,
  bbox `[157,449,192,568]`. First CLEAR C12, **6.800000355 s**, 4,389 pixels,
  bbox `[159,442,235,571]` (half-open pixel bounds).
- At C11 baseline SE(2)=(6.556397438049316,7.796733379364014,1.983917864803299),
  cart clearance **+8.996158 m**. C12=(6.219180107116699,8.466606140136719,
  2.0877356472537167), clearance **+8.496348 m**.
- First CLEAR precedes baseline proxy entry by **6.228501422 s**; its saved
  baseline application at **6.900000360 s** also precedes entry. This useful
  visibility-timing check passes, but the physical gate remains closed.

`outputs/e16-c26-camera-20261011-01` records original camera attributes,
projection coefficients, all 48 camera-space sample projections and static
collision-ray hits. Original front Hawk is **fisheyePolynomial**, not pinhole.
Horizontal centre-line FOV derived from its polynomial is **120.314737°**;
maximum radial FOV attribute is 150° (a different quantity). Nominal sensor
1920×1200, optical centre (957.85107421875,589.5376586914062), scaled to the
unchanged 1920×1080 image. Camera origin in body coordinates is
(0.10030000000000006,0.0749979999999999,0.3459) m; optical −Z is body forward,
optical +X is body right, optical +Y is up. Full roll/pitch/yaw transforms
are retained, not replaced with a yaw-only camera approximation.

Each tested side uses nine longitudinal positions × heights .1/.45/.9 m.
Both sets are entirely inside the lens FOV at C1/C10/C11/C12/C23:

| Pose | North/right unoccluded samples | South/left unoccluded samples |
|---|---:|---:|
| C1 | 0/27 | 0/27 |
| C10, before reveal | 9/27 | 0/27 |
| C11, first visible | 23/27 | 0/27 |
| C12, first CLEAR | 26/27 | 21/27 |
| C23, hypothetical conflict inspection | 27/27 | 27/27 |

At C11 the south samples are occluded by `Geo_M2_BaseWallSide3`; north is
mostly exposed, with four rays blocked by the cart itself. At C12 south has
six blocked samples (wall/corner trim) and north one (cart). Thus both routes
are substantially exposed geometrically by C12, with partial south occlusion;
this does not establish enough information for model reasoning or traversal.
C1/C10 cart hiding is geometric occlusion, including existing scene props;
C27's cart samples are outside FOV after passage, not evidence of renewed
wall occlusion. Rays use collision meshes and finite samples; original RGB
and frozen semantic masks remain the cart visibility evidence. No right-bias
claim follows from a path or from this visibility asymmetry.

## No-cart OLD/FRESH predictions against the inserted footprint

Each raw (30,2) chunk is projected using **its own observation pose**:
`world = observation_xy + R(observation_yaw) @ [forward,left]`. Timing is
assigned from the source-confirmed +.1…3.0 s convention, not a native channel.
Same absolute nominal times are linearly interpolated inside overlapping
curves, with no extrapolation or spatial matching. Full raw native and world
curves, remaining curves, observations, baseline switch poses and all 47
adjacent metrics are exported.

First-visible pair **C10→C11** has OLD/FRESH clearance +8.221948/+7.624676 m;
first-CLEAR pair **C11→C12** has +7.624676/+6.950262 m. Neither predicts conflict.
The reveal is later than C19's start visibility but still **early relative to
an OLD nominal-horizon conflict**.

Full baseline chunks **C22–C27** intersect the inserted proxy. Earliest OLD
remaining conflict with positive current separation and timely baseline
application is **C22→C23**; this is the designated hypothetical inspection,
not a replacement first-reveal event and not a cart-conditioned FRESH response.

| C22→C23 quantity | Value |
|---|---:|
| OLD / FRESH observation | 11.800000615 / 12.300000641 s |
| OLD remaining / FRESH clearance | −0.237458 / −0.823395 m |
| Current separation at C23 | +1.091413 m |
| Nominal first predicted conflict | 14.397397220 s |
| Nominal conflict lead from C23 / saved application | 2.097397 / 1.997397 s |
| Actual baseline proxy-entry lead from C23 / application | 0.728501 / 0.628501 s |
| Aligned RMSE / max displacement | 0.443981 / 0.573560 m |
| Mean / max absolute lateral revision | 0.045945 / 0.122315 m |
| Mean signed left revision | +0.044003 m |
| Common-horizon endpoint revision | 0.514220 m |
| Mean / max derived tangent change | 27.519426 / 92.984092° |
| Baseline controller Δv / Δw | 0 / −0.015212854 rad/s |
| Observation→application, sim / wall | 0.100000005 / 0.519708405 s |
| Action call, wall | 0.055375867 s |
| Robot translation / yaw during pending inference | 0.150095875 m / +0.492623288° |
| RAW baseline position / tangent seam | 0.018258922 m / 4.149095628° |

OLD observation SE(2)=(-0.26671308279037476,11.166097640991211,-3.06393831324127);
FRESH=(-1.013498067855835,11.092246055603027,-3.022630096804683);
baseline switch B=(-1.1624375581741333,11.073650360107422,-3.0140321984517504).
The disparity between nominal predicted time and actual baseline entry is
retained; nominal timing is not a guarantee of executed arrival. Tangents
are derived geometry, not native yaw. Predominantly forward revision and
small lateral variation are ordinary no-cart replanning evidence here.

| Baseline pair | OLD clearance | FRESH clearance | Aligned RMSE | Max lateral |
|---|---:|---:|---:|---:|
| C23→C24 | −0.823395 | −0.870702 | 0.489397 | 0.091995 |
| C24→C25 | −0.870702 | −0.870702 | 0.394329 | 0.099532 |
| C25→C26 | −0.870702 | −0.870702 | 0.359269 | 0.077219 |
| C26→C27 | −0.650128 | −0.271617 | 0.387716 | 0.067311 |
| C27→C28 | +0.096462 | +0.478881 | 0.404466 | 0.065946 |
| C28→C29 | +0.837371 | +1.231037 | 0.381417 | 0.055505 |

All distances metres. Later geometric clearance merely follows the original
baseline passing through/then beyond the newly inserted proxy; it is not
avoidance or improvement. No actual C26 avoidance direction, controller
response, first contact prim or pre-contact cart-conditioned pair exists.

## Instruction, outputs and decisions

Exact runtime instruction is identical in the original baseline, exact-C19
and C19+.40Y runs and matches official E16:

> Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.

It contains **no turn-RIGHT-on-obstacle directive**. “Right side” locates the
bed; the specified turn is left into the hallway. There is **no fourth C26
runtime instruction** because no model run occurred; its planned unchanged
instruction is the same baseline text.

Final analysis: **`outputs/e16-c26-analysis-20261011-02`**. Required
`trajectory_result`, `trajectory_turn_zoom`, `trajectory_revision`, and
`old_fresh_pair_comparison` each have PNG/PDF. Individual PNG/PDF cover
C10→C11, C11→C12 and C23→C24…C28→C29. Pair plots show full 30-point OLD
dashed, OLD remaining orange, full FRESH purple, exact observation markers,
baseline executed path/switch B, cart/inflation and structural context.
All labels identify no-cart evidence; overall plots retain both previous
cart runs and their contact robot positions as secondary context, with no
invented C26 execution. Coordinate JSON, dense path/observation CSV, all-pair
CSV/JSON, instruction audit, visibility table, and camera projection figures
C10/C11/C12/C23 are included. Initial analysis -01 is preserved.

```bash
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/analyze_e16_c26.py --preflight-dir outputs/e16-c26-preflight-20261011-01 --camera-dir outputs/e16-c26-camera-20261011-01 --output-dir outputs/e16-c26-analysis-REVIEW
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/e16-c26-analysis-20261011-02/viewer_data --output-dir outputs/e16-c26-gui-REVIEW --self-test
```

Use fresh directories. GUI starts paused at t0 with fixed cart, explicitly
**PREFLIGHT ONLY / no-cart baseline**. It offers Restart, Play/Pause, speeds,
slider, Prev/Next C, separate first-visible/first-CLEAR, selected baseline
OLD/FRESH/switch, baseline closest approach, Overview/Follow/Hawk. First
collision is disabled because no C26 run/contact exists. Original no-cart
model RGB is the default; toggle shows archived cart preflight overlay.
Full OLD dashed and OLD remaining/FRESH guides are displayed. Baseline replay
can pass through the inserted cart: no physics or model is reexecuted.

Separate decisions: **PHYSICAL PLACEMENT INVALID under the unchanged experiment
gate** (supported/nonoverlapping cart, bypass FAIL); **START HIDDEN**;
**EARLY REVEAL relative to OLD conflict** (useful visibility timing PASS);
**NO CLEAR BYPASS demonstrated**; **PRE-CONTACT CART-CONDITIONED REVISION:
NOT MEASURED**; **OLD CONFLICT→FRESH IMPROVEMENT: NOT MEASURED**;
**RECONCILIATION-RELEVANT PAIR: INSUFFICIENT EVIDENCE**.

Final saved GUI **outputs/e16-c26-gui-20261011-02** reached GUI_READY at
t0/C1, with full OLD and separate visibility jumps verified. GUI -01 is
preserved. Independent checks verified **1,705 protected files**, all 48
masks, 47 overlap metrics, original asset identity and instruction-submission
source. Frozen XY/Z are exact; independent NumPy yaw recomputation agrees
within 4.44e-16 rad (the frozen transform is not rewritten).

16 focused tests pass, covering exact C26/pivot/asset-source immutability,
one-pixel C1 gate, temporal gate/contact ordering, camera projection,
own-observation anchors, exact plotting coordinates and instruction equality,
plus existing cart/shift/observation regressions. Source hashes, masks and
original asset identity are verified; PNG/PDF and GUI captures reviewed.
Detailed validation and Git receipts: `outputs/e16-c26-setup-20261011-01`.
No environment, visibility thresholds, model/controller, correspondence,
graph optimization or trajectory correction changes. No causal navigation,
latency-hard-case or reconciliation-performance claim.
