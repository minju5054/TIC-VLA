# TIC-VLA 2 Hz native cart analysis and stationary human preflight

**Decision: `HUMAN_SITE_QUALIFIED_MODEL_FREE`, selected site H05.** This establishes a stationary placement, conservative connected bypass and early actual Hawk visibility. It does not establish human-conditioned TIC-VLA avoidance, dynamic tracking, stopping, or reconciliation.

Stage A is **`INSUFFICIENT_GEOMETRIC_EVIDENCE`** for a local cart detour: 28 original cart instances were verified, two were near the eight goal-cut paths, and neither supplied an eligible straight west-corridor passage. Their closest approaches occur on the initial approach or around the required corner. Successful passage cannot be attributed to cart recognition from these unchanged-scene observations.

All new work is model-free: **0 model inferences, 0 checkpoint loads, 0 navigation trials, 0 physics steps**. The 254 new images are stopped-stage renders of saved original robot poses with a stationary human. Dashed candidate bypasses are constructed geometric paths and were never predicted or executed.

## Scope and repository provenance

The user requested exactly two stages: original Hospital cart analysis, then a bounded stationary-human placement qualification. The retained failure analysis is [Current obstacle avoidance failure analysis](OBSTACLE_AVOIDANCE_FAILURE_ANALYSIS.md). No 2 Hz/10 Hz baseline rerun, obstacle-conditioned prediction, speed governor, controller/history/KV change, moving human, correspondence or graph method was performed.

| Identity | Value |
|---|---|
| Branch | `hardcase-probe` |
| Starting local HEAD | `2258caa21efed76b2c6be47273313642416ffbfa` |
| Starting origin after fetch | `f99f6d2f29f24e93ff02e19161642353416f1748` |
| Upstream | `9fa6f8b66b9e121d5df5df071297bba8e5353ebb` |
| Starting divergence, remote/local | `0 / 1`; local failure-analysis commit preserved |
| New evidence directory | `outputs/ticvla-2hz-human-preflight-20261013-01` |
| Publication receipt | `outputs/ticvla-2hz-human-preflight-20261013-01/git-finalization.json` records the final local/remote commit after publication |

Repository preparation ran `pwd`, status, branch, HEAD, remotes, `git fetch --all --prune`, remote HEAD, recent log and divergence. The tree began clean. Each infrastructure launch records arguments, wall-clock start/end, source hashes, local SHA and dirty diff identity under `attempts/`. New untracked source bytes are explicitly hashed; the tracked Git diff alone would not identify them. Only additive research code/config/tests/report/result and an append-only work-log entry are published. Official `DynaNav/`, previous configs/reports/raw outputs, weights, environments and LightNav are unchanged.

## Validated historical reference data

The six identities below were resolved from `outputs/e16-zero-repeatability-plan-20261011-01/receipt.json`. Primary and secondary references were not inferred from request IDs. All eight configurations agree on scene, robot, camera, instruction, seed, controller, simulation, lighting and inference identity. They contain 48 finite native `(30,2)` arrays; this study does not convert those arrays into trajectories or assign them yaw.

Each path is cut at its first measured dense pose with strict XY goal distance `<2 m` from `(-12.23, 10.19)`. That row is included; later motion is excluded. Measured west-gate passage and recorded nonfloor-contact absence were rechecked. The fixed 48 predictions continue after entering the goal region, which is not evidence of successful stopping.

| Reference directory under `outputs/` | First goal entry, sim s | Dense rows used | Contacts |
|---|---:|---:|---|
| `nova-e16-hospital-lights-baseline-20261010-01` — primary | 18.566668 | 1115 | None under recorded nonfloor rule |
| `nova-e16-no-cart-15-repro-20261011-01` — secondary | 19.050001 | 1144 | None |
| `nova-e16-zero-repeat-20261011-01-r01-no_hook` | 18.516668 | 1112 | None |
| `nova-e16-zero-repeat-20261011-01-r02-zero_hook` | 18.483334 | 1110 | None |
| `nova-e16-zero-repeat-20261011-01-r03-zero_hook` | 19.083334 | 1146 | None |
| `nova-e16-zero-repeat-20261011-01-r04-no_hook` | 18.766668 | 1127 | None |
| `nova-e16-zero-repeat-20261011-01-r05-no_hook` | 18.400001 | 1105 | None |
| `nova-e16-zero-repeat-20261011-01-r06-zero_hook` | 18.750001 | 1126 | None |

The contact rule is the existing recorded environment contact with `abs(normal.z)<0.5`, retained until CONTACT_LOST; it is not a universal collision oracle. `reference-validation.json` preserves each historical metadata/config/runtime snapshot, upstream/local SHA and dirty-state identity. Previous analysis hashes for **3,536 output files match**. A separate historical source audit found **143 distinct source/hash pairs: 133 current-byte matches, seven retained Git-byte matches, three unrecovered historical dirty versions**. The unrecovered versions are `analyze_e16_repeatability.py`, `plot_e16_repeatability.py`, and `calibrate_hospital_lights.py`; none is required to execute the saved-pose analysis or the already archived lighting configuration. This is a provenance limitation, not reconstructed experimental evidence. Required raw poses, contacts, RGB, request records and configs are present.

The checkpoint is only hashed, never loaded: `TIC-VLA-model.ckpt` SHA256 `376263f89fad0f42c267d85655019232edc91d36e214e23424804dd4cd42e036`; base `model.safetensors` SHA256 `a8b67c54568417f3631723e6b3e120720eaa638e03e62dc25666c70e3ae3e484`. The protected before/after manifest covers **3,440 files**, including selected raw data, all initially tracked files and model-directory files.

## Coordinates, geometry and clocks

World units are metres, +Z up; yaw is CCW about +Z. Westward travel is `forward=(-1,0)`, `left=(0,-1)`. With structural entrance `x0=2.5521036832844546`, progress is `s=x0-x`; left offset is `l=10.599539794921895-y`. Positive left therefore decreases world Y. The origin is the original Hospital world origin; no trajectory-fitted frame is used.

Saved body poses retain XYZ and the full **wxyz** quaternion. Row-vector transforms map camera-local → body-local → world: `T_camera_world = T_camera_body @ T_body_world`; translation is in the final row. The camera's local +X is image right, +Y up, and optical depth is −Z. Full roll/pitch are retained; a yaw-only reconstruction is not used. The independent camera check's maximum absolute matrix error is **2.7683e-7**, within the frozen `1e-5` bound.

Dense pose and observation times are historical simulation times. The historical observation wall timestamp remains historical; new static render wall-clock ranges are recorded separately in attempt receipts. Per-frame render wall start/ready timestamps were not measured. Render completion is explicitly awaited with four subframes, `delta_time=0`, paused timeline and unchanged timeline time 0. No inference start/ready, reasoning generation/poll, command switch or new navigation time is generated in this study.

Hospital source SHA256 is `83e5d606fbb2de873314379e0ff74900930735eb3ae99b312c9bf17db0033383`. Its **2,161 original collider entries** match the previous inventory before and after static queries/rendering. All source layers remain unsaved; the original scene geometry is unchanged.

The south inner wall face is **Y=8.850853576660183** and north face **Y=12.348226013183607**. Their midpoint gives nominal structural centerline **Y=10.599539794921895**, width **3.497372436523424 m**. Sources are the existing audited south `Geo_M2_BaseWallCorner2_578/.../Section0` and north `Geo_M2_DoorWall2Corner2_656/.../Section0`, with full paths/matrices in `stage-a/corridor_geometry.json`.

Independent PhysX rays at Z=.35/.70 confirm both lateral faces. The initial geometry screen excluded closed doors by name and a .03 m tolerance. Before any human-specific feasibility or RGB evaluation, inspection showed known closed north doors recessed .05177 m. The documented geometry-only revision accepts parallel wall/closed-door/trim hits within .075 m, preserving their measured hit coordinates and the nominal wall-derived centerline. **110/133 cross-sections are valid; 23 ambiguous/obstructed sections remain invalid.** Both original and revised geometry records are retained. All four fixed candidate stations have valid sections and lie before the goal region's east extent X=−10.23. The grid was not relocated.

## Stage A: verified carts and measured passages

Original carts are identified from actual root-instance USD reference assets in the `SM_SupplyCart_*`, `SM_GasCart_*`, and `SM_Cart_*` families, with source byte hashes, world matrices, real collider entries and conservative XY AABBs. There are **28 original instances**. Furniture, beds and structures are not relabelled as carts. Historical added `/World/E16Cart` objects are absent from this scene and excluded.

The preregistered near-route screen is center-path distance to cart AABB `<=3 m`; meaningful local passage additionally requires conservative robot clearance `<=2 m`, corridor overlap and complete valid approach/local/post windows. Of the 28 instances, two pass the near-route screen across all eight paths:

| Original root prim | Verified asset | Primary closest time, sim s | Primary XY | Primary disk-to-cart AABB clearance | Range over eight runs |
|---|---|---:|---|---:|---:|
| `/Root/SM_SupplyCart_02a2_91` | `Props/SM_SupplyCart_02a.usd` | 3.906000 | (7.320399, 4.309116) | 1.584295 m | 1.291853–1.584295 m |
| `/Root/SM_SupplyCart_01e9` | `Props/SM_SupplyCart_01e.usd` | 9.556308 | (3.077060, 10.975517) | 1.394084 m | 0.592021–1.689634 m |

The asset hashes are respectively `ea41149b6e26b9039c2109b20757f5337c24d3c1acf732985ce81ea81af3b39c` and `882bbdf100c171307303e920c4c48af19899f1f72a1abf747a16d46226ea5be6`. Their complete footprints are retained in `native_cart_inventory.json`. These are conservative visual AABBs, with separate collider AABBs; their distance is not a measured contact distance.

There are **16 proximity records and zero eligible straight-corridor cart passages**. The first cart is on the initial northbound approach, the second outside the corner/west-corridor boundaries. Crossing their world-X station does not make either a validated westbound local passage. Their raw closest points, times, signed west-frame coordinates, measured yaw changes/curvature, commands and body yaw rates are saved, but outside-corridor offsets and corner-dominated windows are not interpreted as local detours. A separate validated north-corridor approach trend is not established by this study.

The frozen local screen would fit OLS left-offset versus progress over cart-relative `[-3,-1] m`, evaluate local `[-1,+1] m`, require residual magnitude at least .20 m sustained over .50 m progress, then absolute median post residual over `[+1,+3] m` no more than .10 m and half the peak. Invalid geometry or incomplete windows yield `INSUFFICIENT_GEOMETRIC_EVIDENCE`. The maximum signed/absolute **eligible local excursion is unavailable, not zero**. No cart-local excursion candidate is established; `PASSIVE_CORRIDOR_FOLLOWING_CONSISTENT` is not forced onto invalid windows.

General goal-cut corridor following remains descriptively measurable:

| Path | Minimum left offset | Maximum left offset | Minimum nominal parallel-wall clearance with .606983 m disk |
|---|---:|---:|---:|
| Primary | −0.626528 m | +0.886459 m | +0.255244 m |
| Secondary | −1.455550 m | +0.867353 m | −0.313846 m |
| Repeat 1 | −0.628031 m | +1.094704 m | +0.047000 m |
| Repeat 2 | −0.396405 m | +0.916987 m | +0.224716 m |
| Repeat 3 | −0.999754 m | +1.134014 m | +0.007690 m |
| Repeat 4 | −0.731828 m | +1.095035 m | +0.046669 m |
| Repeat 5 | −0.306186 m | +1.064291 m | +0.077413 m |
| Repeat 6 | −0.746222 m | +1.028720 m | +0.112984 m |

These are corridor-wide centerline offsets, **not cart-relative residual excursions**. The secondary's negative circumscribed-disk/nominal-wall proxy does not invent a historical collision; recorded nonfloor contacts remain absent. Proxy conservatism, local recesses and the actual oriented footprint differ from a full disk against infinite parallel walls. Candidate bypass qualification separately checks real PhysX overlaps and conservative continuous bounds.

The unchanged scene supplies no obstacle-present/absent intervention. Therefore cart recognition, cart-caused steering, route memorization and training-trajectory leakage remain unestablished. No causal intervention was run.

## Stage B: frozen candidates and collision convention

The plan was written and hashed before human-specific queries or RGB. Versioned plan: [ticvla_2hz_cart_human_preflight.json](../configs/research/ticvla_2hz_cart_human_preflight.json). SHA256: **`1142a519f28e0d29ce402e5d8bca538efb16bd233ccc73018d397bfb542f8e47`**. The 12 candidates use downstream distances 3/5/7/9 m and left offsets 0/+.25/−.25 m in that order. None reuses C19/C26 placement.

| IDs, offset order 0 / +.25 / −.25 | World X | World Y values in the same order |
|---|---:|---|
| H01 / H02 / H03 | −0.4478963167155454 | 10.599539794921895 / 10.349539794921895 / 10.849539794921895 |
| H04 / H05 / H06 | −2.4478963167155454 | same three Y values |
| H07 / H08 / H09 | −4.447896316715545 | same three Y values |
| H10 / H11 / H12 | −6.447896316715545 | same three Y values |

All root Z=0 m, root yaw=0 rad. This is the audited Z-up rest-pose transform with RotateZ=0; anatomical facing is not inferred from axis names. The real rendered asset is `Assets/Isaac/6.0/Isaac/People/Characters/male_adult_construction_03/male_adult_construction_03.usd`, SHA256 **`2e5d7fc72d6e2ed67f0ab86e2f3aaa91066019a4c71266361735956f232ccff0`**, audited height **1.9383541345596313 m**. Existing `human_actor.author_human` provides identical centering, units and ground alignment. The loaded world transform is checked at each capture. The human is visual-only during stopped rendering; static query proxies provide qualification without advancing a kinematic actor.

Robot radius **0.6069825421953869 m**, human radius **0.25 m**, additional static margin **0.05 m**, required center separation **0.9069825421953869 m**. Historical path relevance uses strict `<0.8569825421953869 m` continuous center-path distance. This denotes a hypothetical conflict if a human were added; no human collision occurred in those historical runs. The capsule does **not** enclose the rest pose's outstretched arms. Placement additionally checks the audited full visual bounds against original geometry, but the bypass does not claim full mesh-human clearance.

Each side independently connects the primary measured path at human X±2.5 m to a side strip at longitudinal ±1.1 m and lateral ±1.0 m from the person. RIGHT increases world Y. Every connector and intermediate segment is checked, with samples at most .05 m apart, center plus four footprint floor probes, real original-geometry overlap queries using robot radius+.05, and continuous segment-to-collider-AABB distances. The sphere query is centered at Z=.70 m; continuous bounds include original enabled colliders intersecting its vertical range, excluding the floor. “Structural clearance” conservatively includes fixed furniture and trim as well as walls. The goalward connector is 2.5 m downstream, **1.593017 m beyond the required separation boundary**, satisfying the additional metre requirement. This is a sufficient test of these fixed constructions, not an exhaustive planner; a failed side does not prove no other bypass exists.

## Candidate results and deterministic selection

| ID | Physical | Primary conflict | Conflicted primary + six | LEFT | RIGHT | CLEAR ≥5 m | Decision |
|---|---|---|---:|---|---|---|---|
| H01 | PASS | PASS | 7/7 | FAIL | PASS | PASS | Qualified |
| H02 | PASS | PASS | 6/7 | FAIL | PASS | PASS | Qualified |
| H03 | PASS | PASS | 7/7 | FAIL | FAIL | Not evaluated | Reject: no passing construction |
| H04 | PASS | PASS | 7/7 | FAIL | PASS | PASS | Qualified |
| **H05** | **PASS** | **PASS** | **7/7** | FAIL | **PASS** | **PASS** | **Selected** |
| H06 | PASS | PASS | 7/7 | FAIL | FAIL | Not evaluated | Reject: no passing construction |
| H07 | PASS | PASS | 7/7 | PASS | PASS | PASS | Qualified |
| H08 | PASS | PASS | 7/7 | FAIL | PASS | PASS | Qualified |
| H09 | PASS | PASS | 6/7 | PASS | FAIL | PASS | Qualified |
| H10 | PASS | PASS | 6/7 | PASS | PASS | PASS | Qualified |
| H11 | PASS | PASS | 7/7 | FAIL | PASS | PASS | Qualified |
| H12 | PASS | PASS | 2/7 | PASS | FAIL | PASS | Qualified |

Counts: **12 placements, 12 primary conflicts, 4 LEFT passes, 8 RIGHT passes, 10 candidates with at least one bypass, 10 early RGB passes, 10 qualified, one selected**. All floor/placement checks and initial robot separation pass. Original colliders remain unchanged.

H03 LEFT has only .019583 m conservative structural clearance near `/Root/trashcan_5/SM_TrashCan`, below .05 m, and overlap samples fail; RIGHT has −.148551 m at north handrail/trim. H06 LEFT has .026512 m conservative AABB clearance near the same bin even though sampled PhysX overlaps are empty; the continuous conservative margin still fails. H06 RIGHT has −.148535 m at north trim with overlap/door-frame hits. Thresholds were not relaxed and neither candidate was rendered. Other individual failing sides and all failing prims/segments/sample coordinates are saved in `stage-b/physical/Hxx.json`.

Ranking is exactly the frozen order: largest minimum structural clearance of a valid side, then largest valid early CLEAR remaining distance, then smallest absolute left offset, then ID. H05 and H08 tie at **.35146472432884646 m**; H05 wins on early CLEAR distance **9.801864989002507 m** versus H08 **9.518032753123935 m**. H02 is slightly lower at .35144855001244113 m; no post-hoc rounding tolerance was applied. The winning site is not chosen from model output.

## Selected H05 visibility and connected geometry

H05 world pose is **(X,Y,Z,yaw)=(-2.4478963167155454, 10.349539794921895, 0, 0 rad)**. The complete future configuration is [ticvla_2hz_stationary_human_selected.json](../configs/research/ticvla_2hz_stationary_human_selected.json), with the unchanged full baseline config plus stationary human definition. It is not an executable authorization to run a navigation trial. `stage-b/selected-site.json` retains the complete checked construction and selection records.

| Observation | Historical sim time | State | Human pixels | Fraction | Bounding box, half-open XY | Remaining measured path | Camera-to-human base distance |
|---|---:|---|---:|---:|---|---:|---:|
| C11 — first pixels | 6.300000329 s | MARGINAL | 264 | .000127315 | [158,380,179,455] | 10.552168 m | 9.244371 m |
| C12 — first CLEAR | 6.800000355 s | CLEAR | 5358 | .002583912 | [154,348,213,553] | 9.801865 m | 8.753695 m |

C12 bearing is **−0.842745819 rad** in camera horizontal coordinates: left side of the image. The 59×205 px bounding box and exact semantic mask satisfy the unchanged CLEAR rule (fraction≥.002, width≥20, height≥80). Along-track distance is the remaining measured path arc length to its continuous closest station to the person, not Euclidean distance or longitudinal world-X distance. Rendering stops before the first hypothetical robot-plus-human conflict, not after passing through the person.

**H05's valid ≥5 m observations are on the original approach/turn, not already inside the straight corridor.** This was allowed explicitly by the frozen rule; the user's “preferably already within the straight corridor” was not made a mandatory gate. Farther H07–H12 also have valid ≥5 m straight-corridor observations, but the predeclared priority selects H05. This preference limitation is not concealed by calling C12 a straight-corridor frame.

H05 C1–C10 are HIDDEN, C11 MARGINAL, C12–C24 CLEAR; the full measured progression is saved. C12 RGB luminance mean **151.374/255**, standard deviation **42.645**, p95 **212.209**, dark fraction **.001616**, saturation fraction **.000144676** pass the original brightness profile. **All 254 captured frames across ten candidates pass brightness.** No exposure or camera movement was used to force a pass. Full raw RGB, uint semantic arrays, label maps and binary masks are saved separately from derived sheets. Actual raster masks determine visibility; original-geometry rays to three torso heights explain occlusion without substituting for visibility. The partial early emergence at the image's left edge was visually checked.

The selected RIGHT bypass, world XY in upstream→downstream order:

```text
( 0.05210368328445458, 11.188700185246221)
(-1.3478963167155453,  11.349539794921895)
(-3.5478963167155455,  11.349539794921895)
(-4.947896316715545,   10.351896536563132)
```

Its minimum center separation is **1.0 m**; conservative robot-human surface clearance is **.14301745780461306 m**, exceeding the .05 m margin. Minimum robot-to-original-geometry clearance is **.35146472432884646 m**, at `/Root/Geo_M_WallHandBaseSmall3/Geo_M_WallHandBaseSmall`. Both connectors and all intermediate samples have support and no overlap. The H05 LEFT construction fails at the bin; this is why selection retains RIGHT. Primary no-human minimum center-path distance to H05 is **.510681 m**, below the .856983 m exclusion threshold; all six repeats also conflict under that same proxy.

## Runtime, attempts and verification

The isolated `bash scripts/isaac6_python.sh` launcher uses Isaac **6.0.1-rc.7+release.42383.32955d8d.gl**, Python **3.12.13**, NumPy **2.3.1**, Pillow **12.2.0**. GPU: RTX 5060 Ti, driver **595.99.02**. The exact loaded Kit extension versions are in the attempt logs. This is the separately authorized Isaac 6 compatibility path, not unchanged Isaac 5 reproduction. CPU analysis uses the pre-existing system Python/NumPy/Pillow/matplotlib; no package was installed. Historical inference dependency snapshots remain in `reference-validation.json`; the Isaac 5 inference process is not started.

| Attempt | Result and disposition |
|---|---|
| `startup-01` | Launcher was not executable; child never started. Kept failure, used `bash`; no permission-bit change. |
| `startup-02` | Simulator-only READY and clean shutdown PASS; no model import or inference probe. |
| `inspect-01`, `inspect-02` | Inventory comparison failed only on empty infinite-floor AABB serialization; second attempt saved the exact one-entry difference. No scene mutation. |
| `inspect-03` | Existing audit's empty→null convention restored; all 2,161 entries equal, cart references and real static rays PASS. |
| `geometry-01`, `stage-a-analysis-01`, `freeze-01` | Saved geometry clarification, eight-path analysis, then plan/source freeze PASS. |
| `physical-01` | All 12 positions and 24 connected side constructions checked; actual static-query gate PASS. |
| `render-01` | Isaac 6 has no `add_update_semantics`; failed before creating candidate images. |
| `render-02` | Separate process-local adapter maps that class-label operation to existing audited `add_labels(prim,[label],instance_name='class',overwrite=True)`; 254 RGB/semantic captures PASS. |
| `runtime-versions-01` | `pxr` is unavailable before Kit starts; version-only attempt failed without a SimulationApp. |
| `runtime-versions-02` | Available Python/NumPy/Pillow and installation version recorded without starting another app. |
| `historical-code-01` | 143 historical source/hash pairs classified, including the three explicit provenance gaps. |
| `cpu-tests-01` | **31 focused CPU tests PASS**. This is not the Isaac gate. |
| `finalize-01`, `review-figures-01` | Frozen ranking reproduced, one selected config, saved-only figures/sheets PASS. |
| `evidence-validation-01` | Independent validation PASS: 3,440 protected files unchanged, frozen sources match, 24 side checks, all 254 raw RGB/semantic masks/boxes/full camera transforms and selection rechecked. |

The original frozen research files remain byte-identical. The added render compatibility adapter was separately hashed before rendering in `stage-b/render-api-adapter.json`; the original failed code and initial freeze receipt are retained. No placement, threshold, ranking or physical result changed. Isaac reports static-body joint warnings when the saved renderer disables robot rigid bodies for stopped pose replay; no robot joints or policy are executed, and the full camera/world transform check passes at every capture. These warnings are not hidden as an exit-code-only success.

Tests cover exact reference identities and goal cutoff, signs, deterministic grid, continuous between-sample conflict, cart AABB distance, excursion/persistence/return, missing connectors, both bypass sides, spacing/margins, early CLEAR gates, deterministic ranking, exclusive writes and forbidden model/navigation calls. Actual artifact validation adds floor/overlap evidence, full original-camera reconstruction, mask recounts, immutable source/config/weight checks and zero-execution counters. The independently recounted masks contain **4,382,712 human pixels** across all frames. Additional final documentation/config validation, diff review and publication are recorded in their append-only receipts.

## Evidence, limitations and missing evidence

Positive evidence is the unchanged-scene 2 Hz saved success, exact goal cutoff, verified original cart identities, independently derived corridor, 12 valid placements, ten conservative connected bypass candidates, measured early rendered human visibility and deterministic H05 selection. The new static query/render gate passed; raw/source/weight immutability and camera/mask validation passed.

Negative evidence is the absence of an eligible local original-cart detour window, H03/H06 failing the fixed bypass constructions, multiple other failing sides, and H05 lacking a ≥5 m observation already inside the straight corridor. The broad corridor offsets are neither a causal detour signature nor evidence that the model recognized a cart. Failed conservative AABBs do not prove actual collision, and fixed construction failures do not prove global infeasibility.

Missing evidence includes any human-conditioned FRESH action, executable dynamic bypass, acceleration/curvature/tracking feasibility, full visual-human mesh clearance, safe interaction, stopping at E16, real-world behavior, and obstacle-conditioned causal cart intervention. Three unused historical dirty source versions and per-frame new-render wall timings are unavailable as stated above. No synthetic path or static source inspection is presented as navigation experimental evidence. The checkpoint/final-paper policy identity remains outside this task's scope.

## Artifacts, visualization and reproduction

Tracked summary: [results/ticvla-2hz-cart-human-preflight.json](../results/ticvla-2hz-cart-human-preflight.json). Raw and derived artifacts are in the fresh output directory and are not committed. Source RGB and native arrays remain untouched; upstream temporary image buffers are not claimed as an archival dataset.

Key saved artifacts relative to that directory:

- `stage-a/native_cart_inventory.json`, `root_asset_identities.json`, `collision_geometry.json`, original and validated corridor geometry, `cart_passages.json`, `path_centerline_offsets.csv`, `stage-a-summary.json`.
- `stage-a/figures/corridor-path-carts.png`, `offset-progress.png`, and two `near-cart-*.png` equal-aspect XY proximity views. Offset/progress uses metric axes rather than pretending it is an XY map.
- `stage-b/frozen-plan.json`, `freeze-receipt.json`, `render-api-adapter.json`, all 12 `physical/Hxx.json` and asset hash.
- `stage-b/renders/Hxx/Cxx-rgb.png`, `-semantic.npy`, `-labels.json`, `-mask.png`, `-observation.json`, complete `visibility.json` sequences and camera/lighting readback.
- `stage-b/qualification-matrix.json`, `selection-receipt.json`, `selected-site.json`.
- `stage-b/figures/candidate-map.png`, all 12 bypass plots and ten actual Hawk sheets. `stage-b/review-figures/selected-site-conflict-and-bypass.png` separately labels the .856983 m conflict disk and .906983 m margin disk.
- `protected-sha256.json`, historical raw/code verification, `reference-validation.json`, `evidence-validation.json`, and all `attempts/*/{start,end}.json` plus logs.

The selected sheet, selected bypass and whole-route cart map were visually reviewed. Equal XY aspect and labels distinguish measured paths, original AABB proxies, hypothetical conflicts, geometric constructions and actual rendered human pixels. No GUI was produced: static evidence sheets are sufficient and avoid presenting a constructed bypass as executed motion. A direct local review command is `xdg-open outputs/ticvla-2hz-human-preflight-20261013-01/stage-b/figures/H05-hawk-sheet.png`.

The exact executed commands are in attempt receipts. A new full reproduction must choose an unused output ID and an unused plan-config filename. Never rerun into this directory. The successful phases are:

```bash
python3 research/two_hz_preflight.py --phase prepare --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt startup-02 -- bash scripts/isaac6_python.sh scripts/probe_isaac6.py
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt inspect-03 -- bash scripts/isaac6_python.sh research/two_hz_preflight_isaac.py --phase inspect --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt geometry-01 -- python3 research/analyze_two_hz_preflight.py --phase geometry --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt stage-a-analysis-01 -- python3 research/analyze_two_hz_preflight.py --phase analyze --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt freeze-01 -- python3 research/analyze_two_hz_preflight.py --phase freeze --output-dir "$PREFLIGHT_OUT" --config "$PREFLIGHT_PLAN"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt physical-01 -- bash scripts/isaac6_python.sh research/two_hz_preflight_isaac.py --phase physical --output-dir "$PREFLIGHT_OUT"
```

Before the render adapter command, create an exclusive `stage-b/render-api-adapter.json` binding the new frozen plan hash, adapter SHA256 and its two unchanged API evidence sources. The adapter rejects an unbound source:

```bash
PREFLIGHT_OUT="$PREFLIGHT_OUT" python3 - <<'PY'
import os
from pathlib import Path
from research.two_hz_preflight import digest
from research.records import write_json
p = Path(os.environ['PREFLIGHT_OUT']) / 'stage-b'
adapter = 'research/two_hz_preflight_render.py'
sources = ['research/render_hospital_occlusion.py',
           '/home/gpuadmin/isaacsim/extsDeprecated/isaacsim.core.utils/isaacsim/core/utils/semantics.py']
write_json(p / 'render-api-adapter.json', {
    'plan_sha256': digest(p / 'frozen-plan.json'),
    'adapter_source': adapter, 'adapter_sha256': digest(adapter),
    'evidence_sources': {s: digest(s) for s in sources},
    'reason': 'Isaac 6 class labels through existing add_labels API; rules unchanged',
    'candidate_rgb_captures_before_adapter': 0,
    'original_frozen_sources_unchanged': True})
PY
```

```bash
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt render-02 -- bash scripts/isaac6_python.sh research/two_hz_preflight_render.py --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt finalize-01 -- python3 research/analyze_two_hz_preflight.py --phase finalize --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt review-figures-01 -- python3 research/plot_two_hz_preflight_review.py --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt historical-code-01 -- python3 research/verify_two_hz_preflight.py --phase historical --output-dir "$PREFLIGHT_OUT"
python3 research/two_hz_preflight.py --phase launch --output-dir "$PREFLIGHT_OUT" --attempt evidence-validation-01 -- python3 research/verify_two_hz_preflight.py --phase evidence --output-dir "$PREFLIGHT_OUT"
python3 -m pytest -q tests/test_two_hz_preflight.py
```

These commands are static research infrastructure, not instructions to execute a human-navigation trial. The final decision remains **`HUMAN_SITE_QUALIFIED_MODEL_FREE`**. No such trial was executed.

**Can the existing successful adapted 2 Hz TIC-VLA configuration, without a speed governor or controller modification, generate a safe human-conditioned FRESH trajectory and actually pass this one frozen stationary human while reaching the original E16 goal region?**
