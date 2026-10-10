# One cart translation: C19 +0.40 m world Y

**NO CLEAR CART BYPASS; CONTACT BEFORE BYPASS.** The new run contacts an
earlier northern structure and never passes the cart. No OLD-conflict to
FRESH-improvement pair is observed.

This experiment changes only the fixed cart footprint centre from baseline
C19 XY to `[C19.x, C19.y + 0.40]`. It compares one new closed-loop run with the
[previous exact-C19 run](NOVA_E16_C19_CART.md) and the saved no-cart reference.
No position/yaw sweep, controller or lighting adjustment, or retry is allowed.

Starting clean local/fetched origin `hardcase-probe`:
`f47f31fe8e5ac761178c719f15f380368b85abce`; fetched upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Reference:
`outputs/nova-e16-hospital-lights-baseline-20261010-01`.

## Frozen intervention and physical preflight

| Quantity | Exact value |
|---|---|
| Saved no-cart C19 SE(2) | (1.9812549352645874, 11.180365562438965, 3.030531579778966) |
| Previous cart centre XYZ [m] | (1.9812549352645874, 11.180365562438965, -2.384185791015625e-07) |
| New cart centre XYZ [m] | (1.9812549352645874, 11.580365562438965, -2.384185791015625e-07) |
| Both cart yaws [rad] | 3.032406244735343 |

The addition uses unrounded saved coordinates and **world +Y**, not robot
lateral. Actual floor support supplies Z. Previous rotation, local bounds,
asset bytes, source-prim signature, collision and material are identical.
Source `/Root/SM_SupplyCart_01e_21`, asset `Hospital/Props/SM_SupplyCart_01e.usd`,
copy `/World/E16Cart`. No original asset is changed. World metres/+Z up,
yaw CCW from +X, body quaternion w,x,y,z; USD transforms use row vectors.

Preflight **PASS**: supported floor, zero cart/static overlap, initial
Nova/cart clearance **+10.310301 m**, original path intersects the inflated
footprint (minimum **-0.473084 m**, first entry **9.580405 s**), feasible south
bypass. Existing Nova radius **0.6069825421953869 m**, cart footprint
**1.1017788450022294 × 0.5274396397000629 m**, unchanged static triangle mesh.

| Sampled local bypass | Cart clearance [m] | Minimum structural-bound clearance [m] | PhysX overlap check |
|---|---:|---:|---|
| North/right, nearest tested strip | +0.050000 | -0.712759 | FAIL; DoorFrame52 and nearby structures |
| South/left, first feasible strip | +0.050000 | +0.899101 | PASS |

Ten existing robot-centre strips per side were checked, without moving the
cart. All ten north strips fail; south is feasible. The signed wall proxy is
a conservative disk/AABB quantity, not measured penetration or a complete
navigation guarantee. The exact traces are in
`outputs/e16-cart-plus-y-preflight-20261010-01/physical.json`.

Saved-camera visibility at 1920×1080 uses the unchanged semantic criterion:
HIDDEN=zero pixels; CLEAR=fraction≥.002 and bbox≥20×80; otherwise MARGINAL.
First visible **C1** (2,530 pixels), first CLEAR **C4** (4,163 pixels).
Visibility is diagnostic and does not alter placement or block a physical PASS.
All 48 RGB/mask/overlay frames, camera matrices and per-C fractions/bboxes are
in the new preflight directory. Prior baseline/cart/human evidence is preserved.

## Outcome definitions fixed before inference

`analysis_rules.json` was written before the run and included in the frozen
preflight receipt. Travel direction `f` is the baseline C18→C20 unit tangent;
left normal `n=(-f_y,f_x)` points south here. For `p-cart_center`, longitudinal
`s=dot(p-center,f)` and signed left `l=dot(p-center,n)`.

The first forward crossing of `s=0` defines the geometric passage side:
`l > half_short + robot_radius` is SOUTH/LEFT, below its negative is
NORTH/RIGHT, otherwise CENTER/NO CLEAR BYPASS. Earlier non-floor contact gives
CONTACT BEFORE BYPASS while retaining the geometric side separately. This
cross-section classification does not guarantee contact-free passage.
Complete local clearance requires the later crossing of
`s=half_long+robot_radius`; contact ordering and travelled distance after that
crossing are recorded separately. Distance is measured path arc length,
including any subsequent stalled-pose motion, so forward progress is also
reported. The no-cart row uses the same downstream plane only as a location
reference; it has no actual cart clearance or bypass label.

The interaction window begins at measured
`s >= -(half_long + robot_radius + 3.0*controller.max_v)` and ends at the
first downstream-clear crossing, or run end if absent. The descriptive
inspection pair is the largest **max absolute FRESH-frame lateral revision**
inside that window, earliest OLD ID breaking ties. It never replaces the
independent first-visible/first-CLEAR pair. All adjacent pairs are exported.
Prediction side uses the same centre-plane test on OLD remaining and FRESH
native-world curves; no crossing means UNKNOWN, without extrapolation.

Clearance uses the oriented cart bounds plus Nova disk. OLD remaining is
clipped at FRESH observation time. Both chunks are projected with their own
observation poses; nominal +0.1…3.0 s times supply temporal interpolation.
Native (30,2) is forward/left metres with no yaw or timestamp channel. Tangents
and RAW boundary seams are derived diagnostics; no spatial matching, graph,
transport correction or reconciliation is applied. OLD-conflict→FRESH-improved
means OLD≤0 and FRESH>OLD+1e-9 m; FRESH CLEAR additionally requires FRESH>0.
All such pairs retain actual-contact/current-clearance/timing context.

## Measured result

**NO CLEAR CART BYPASS.** New run
**`outputs/nova-e16-cart-plus-y-20261010-01`** completed exactly once with
48 finite predictions, 1,522 measured/static-cart ticks, simulation
0–25.350001322105527 s. Strict checkpoint and continuous OLD validation PASS.
All original baseline config fields match, except the frozen cart. Real-run
C1=0, C2=1.8500000964850187 s, then approximately 0.5 s cadence; bootstrap
latency is not a change to the 2 Hz target. No retries or outcome-driven tuning.

Measured classification: **CONTACT BEFORE BYPASS**. The robot never crosses
the cart centre plane or clears its downstream region. Minimum actual
cart-clearance proxy is **+1.194815 m**, at **24.033335 s**, and cart contact
spans are **0**. There are **338 other non-floor contact spans**. First
structural contact at **9.583333833143115 s**:
`/Root/Geo_M2_BaseWallSide10_646/Geo_M2_BaseWallSide2/Geo_M2_BaseWallSide2/Section0`.
**DoorFrame52 contact=false**, because this run stops at an earlier northern
structure; that is not successful local obstacle avoidance. Final XY is
(4.30936861038208,12.212631225585938). Distance after clearing is **undefined**,
not zero, because no cart-region clearance event occurs. PhysX material-face
warnings and all raw contact reports remain archived.

Both preflight and actual first visibility are C1; first CLEAR is C4.
Preflight timeline: C1–C3 MARGINAL, C4–C19 CLEAR, C20–C48 HIDDEN.
Actual timeline: C1–C3 MARGINAL, C4–C16 CLEAR, C17–C48 HIDDEN. HIDDEN after
turning away is not evidence of structural occlusion. Separate first-CLEAR
pair **C3→C4** has OLD/FRESH cart clearance +8.042590/+7.338706 m and aligned
RMSE 0.429216 m; no OLD conflict. No later pair is substituted as first reveal.

## Three-run comparison

All side/clearance values describe actual cart runs. The no-cart row has no
cart contact/clearance/bypass; its progress is relative to the matched
geometric downstream plane. Local progress does not require full E16 success.
| Metric | No cart | Exact C19 cart | C19 +0.40Y cart |
|---|---|---|---|
| Actual bypass | N/A - no cart | NORTH/RIGHT BYPASS | CONTACT BEFORE BYPASS |
| Cart contact spans | N/A | 0 | 0 |
| Minimum cart proxy [m] | N/A | -0.023661 | 1.194815 |
| First structural contact | N/A | DoorFrame52 | BaseWallSide10_646/Section0 |
| First structural contact [s] | N/A | 11.383334 | 9.583334 |
| DoorFrame52 contact | no | yes | no |
| Clears region before structural contact | N/A | no | no |
| Travel after clearance / reference plane [m] | 21.310198 | 1.646020 | N/A |
| Forward progress after plane [m] | 20.883604 | 0.075970 | N/A |
| Max lateral revision, interaction [m] | 0.292575 | 0.352351 | 0.815858 |
| Max absolute delta w, interaction [rad/s] | 0.105891 | 0.194173 | 1.252430 |
| Max lateral revision, all pairs [m] | 0.292575 | 1.960638 | 0.815858 |
| Max absolute delta w, all pairs [rad/s] | 0.137873 | 1.451470 | 1.252430 |

The previous NORTH/RIGHT classification is geometrically reproduced: centre
crossing at 11.370144 s, signed left -0.884990 m. Its later downstream-plane
crossing at 12.483188 s occurs after contact. Its 1.646020 m post-plane arc
length includes stalled motion; net forward progress is only 0.075970 m.

## All-pair geometry and contact context

`plus_Y_pair_metrics.csv/.json` contains **all 47 adjacent pairs**;
`interaction_pair_metrics.csv` contains **41 pairs, C7→C8 through C47→C48**.
Each row includes OLD/FRESH observation XY/times, remaining OLD/FRESH cart
clearance, signed/absolute FRESH-frame lateral change, endpoint/tangent,
command delta, simulation/wall timing, transport and RAW seams. Detailed
projected points and nominal time correspondences are in pair-details JSON.
Baseline/previous pair tables use the identical metric implementation.

The frozen interaction window runs **4.766667–25.350001 s** because the cart
is never cleared. Its maximum lateral pair is **C40→C41**, after structural
contact, not a reveal or an avoidance response. The supplementary pre-contact
maximum is **C12→C13**, also without cart conflict. This supplementary slice
does not replace the frozen selected pair.

| Metric | C40→C41, post-contact peak | C12→C13, pre-contact context |
|---|---:|---:|
| OLD observation time [s] | 20.850001 | 6.850000 |
| FRESH observation time [s] | 21.350001 | 7.350000 |
| OLD remaining clearance [m] | 1.226270 | 2.059611 |
| FRESH clearance [m] | 1.177420 | 2.056103 |
| Aligned RMSE [m] | 0.462486 | 0.365056 |
| Mean absolute lateral [m] | 0.311218 | 0.121049 |
| Max absolute lateral [m] | 0.815858 | 0.363014 |
| Common endpoint revision [m] | 0.710921 | 0.458076 |
| Mean absolute derived tangent [deg] | 64.956861 | 42.395518 |
| Max absolute derived tangent [deg] | 164.341301 | 171.733817 |
| Delta v [m/s] | 0.000000 | 0.000000 |
| Delta w [rad/s] | 0.935516 | -0.177997 |
| Observation→application [sim s] | 0.083333 | 0.100000 |
| Action call [wall s] | 0.052962 | 0.054687 |
| Observation→switch [wall s] | 0.228366 | 0.573466 |
| Robot XY transport [m] | 0.004565 | 0.150086 |
| Robot yaw transport [deg] | 0.669185 | 0.700154 |
| RAW position seam [m] | 0.028180 | 0.014779 |
| RAW executed→FRESH tangent seam [deg] | 179.433007 | 1.352189 |

Large post-contact tangent seams occur with only millimetres of robot motion;
they are derived geometric directions, not native yaw or demonstrated useful
correction targets. **No predicted bypass-side change** occurs under the
centre-plane test. **No OLD-conflict→FRESH-clear/improved pair exists** among
all 47 adjacent pairs. The result is **NO RECONCILIATION-RELEVANT ADJACENT PAIR**.
The measured position change did not produce a completed south/left bypass
in this run. Different closed-loop observations, reasoning histories and
inference schedules limit causal attribution beyond this single intervention.
No graph/reconciliation improvement, formal significance or navigation
performance claim is made.

## Outputs, GUI and validation

Final comparison: **`outputs/e16-cart-plus-y-analysis-20261010-03`**.
`trajectory_result.png/.pdf`, `trajectory_turn_zoom.png/.pdf` and
`trajectory_revision.png/.pdf` show all three measured paths, both footprints,
new inflation, structural context/DoorFrame52, contacts and new C markers.
All 48 C markers use exact observations; spaced labels avoid the stalled
cluster. Selected OLD/FRESH remain anchored to their own observations with
the saved application pose. All figures use world metres/equal XY and
explicitly mark the inspection POST-CONTACT. `trajectory_coordinates.json`,
`observation_poses.csv`, `comparison.csv` and the pair exports preserve plotted
data. `first_reveal/` retains independent first-CLEAR results and actual RGB
mask/overlay evidence. -01/-02 outputs remain preserved; later versions only
improve callout layout and remove a plotting radius literal.

```bash
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/compare_e16_cart_shift.py --run-dir outputs/nova-e16-cart-plus-y-20261010-01 --preflight-dir outputs/e16-cart-plus-y-preflight-20261010-01 --rules outputs/e16-cart-plus-y-preflight-20261010-01/analysis_rules.json --output-dir outputs/e16-cart-plus-y-analysis-REVIEW
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/e16-cart-plus-y-analysis-20261010-03/viewer_data --output-dir outputs/e16-cart-plus-y-gui-REVIEW
```

Fresh output names are required. The existing saved-data GUI adds the gold
previous-cart path/footprint and approach, closest-approach and structural
contact jumps; the original no-cart path is grey, new path cyan. It retains
Restart/t=0, Play/Pause, speeds, Previous/Next C, selected OLD/FRESH/application,
Overview, Follow and front Hawk. Actual archived new-run RGB is displayed;
cart stays fixed, model_calls=0 and physics_reexecuted=false. No new frame
export/video is required for this task.

**10 focused tests PASS**, including exact world-Y arithmetic, identical
yaw/asset, contact-aware side classification, exact three-run plotted
coordinates and source immutability. Independent saved-data checks verify
48 finite predictions, 1,522 constant cart transforms, 96 semantic masks,
1,254 protected source files, all 88 frozen research/runtime code files and
150 frozen preflight files. Baseline config matches exactly except cart.
Full source verification passed immediately after the one run; only plotting
code was edited afterward, with its executed version archived under setup.
No raw data, receipt or old report was rewritten. Figures were rendered from
PDF and visually reviewed. Detailed checks and GUI receipts are under
`outputs/e16-cart-plus-y-setup-20261010-01` and the GUI output directory.

Final GUI validation: **`outputs/e16-cart-plus-y-gui-20261010-02`**, GUI_READY
at **t=0/C1**, all event jumps and stopped-physics/fixed-cart checks PASS.
The GUI was opened and its saved overview/follow/Hawk views inspected; the
window subsequently closed. The command above reopens it with a fresh output
name. Earlier GUI -01 remains preserved. No additional model run occurred.
Final three-path/144-observation coordinate checks, all pair counts, raw
cart/DoorFrame52 report absence and source hashes PASS in
`outputs/e16-cart-plus-y-setup-20261010-01/final-verification.json`.
