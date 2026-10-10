# Model-free stationary-human reveal-window search

## Question and source identity

Find a stationary human placement whose **first CLEAR** saved observation follows a HIDDEN OLD observation, with three immediately preceding HIDDEN observations, while OLD still predicts a proxy conflict after the saved baseline FRESH application. This task uses saved baseline predictions, static geometry queries and archived sensor evidence. It makes no TIC-VLA calls and does not rerun navigation physics.

Starting local and freshly fetched origin: `f1b216a809047fb29248eb136722775f6df4c993`, branch `hardcase-probe`. Fetched upstream/main: `9fa6f8b66b9e121d5df5df071297bba8e5353ebb`. Baseline `outputs/nova-e16-hospital-lights-baseline-20261010-01`, saved analysis `outputs/nova-e16-hospital-lights-analysis-20261010-01`, unchanged lighting `configs/research/nova_e16_hospital_lights_on.yaml`.

The saved-only loader validates all 48 finite native `(30,2)` predictions and their observation/application records. Native coordinates are observation-body forward/left, in metres. Observation times are C1=0, C2=1.800000094 s (bootstrap), then approximately 0.5 s gaps through C48=24.800001293 s. The source baseline continued OLD execution during inference; this search does not execute that physics again. The baseline brightness and physical corner prerequisites passed in the prior report.

The prior 4 m left-corner study showed gradual HIDDEN→MARGINAL→CLEAR emergence, first CLEAR C4–C11 and no remaining OLD conflict at reveal (minimum clearances +5.52…+9.06 m). Those results, images and reports remain intact.

## Preserved partial search and protocol amendment

Original rules are unchanged in `configs/research/reveal_window_search.json`. Its original exhaustive render schedule is historical and is superseded **only for scheduling** by `configs/research/reveal_window_prefilter.json`, following the user's pause instruction. Candidate generation/IDs, human orientation, physical conditions, visibility thresholds, first-CLEAR definition and ranking remain unchanged.

Paused output: `outputs/nova-reveal-window-search-20261010-04`. The last progress report showed 86/391; by the time the two owned renderers received SIGINT, **98 complete candidates** existed. All 98 complete records and every incomplete file were preserved. No renderer resumed inside that output. The new analysis is **`outputs/nova-reveal-window-search-20261010-05`**. Its reuse manifest binds candidate ID/XYZ/yaw, original configuration SHA, source manifest, observation JSON and exact image/mask evidence. It validates all 48 RGB dimensions/modes/PNG integrity and mask pixel counts/fractions per reused candidate. Source and paused-output SHA manifests are checked before and after work.

The amended renderer refuses to start without a preregistered cheap-prefilter receipt. Every new candidate must pass all cheap gates. Already completed evidence is reused read-only, including cheap-rejected candidates as diagnostic evidence. A zero-new-render path merges the manifests before importing or starting SimulationApp.

## Fixed search region and candidate generation

The measured path spans X −20.3458…7.3802 m, Y 1.4922…11.2261 m. Coarse centres lie within 4 m of its polyline on a 0.5 m world grid, including the centreline. Four diagonal 0.25 m neighbours are added only around physically valid coarse centres within 0.75 m of an existing vertical structural collider AABB endpoint and with some OLD remaining conflict. This refinement preceded visibility outcomes. All 192 old corner-band positions are included independently; duplicate XYZs are merged and sorted lexicographically.

There are **1,322 coarse positions, 60 added refinement positions and 1,518 unique candidates** after including the old band. The search covers route-local wall ends, door/room entrances, intersections and the previous corner, not arbitrary Hospital-wide space. Every independent placement is stationary at Z=0, fixed world yaw zero, using the same official upright construction character. Orientation is not optimized per candidate.

Physical validity uses actual frozen PhysX floor/support and overlap queries: human radius 0.25 m at five heights plus the full upright visual enclosing box. The source's wide rest-pose arm bounds make this conservative. Floor contact is allowed; wall/furniture overlap is rejected. Nova radius is 0.6069825421953869 m, combined conflict radius **0.8569825421953869 m**.

Bypass uses the nearest measured path tangent only to orient a local cross-section, not for waypoint correspondence. Both sides are searched in a fixed order at offsets 1.0…4.0 m in 0.25 m steps. A 2 m strip, sampled every 0.1 m, requires floor support, static clearance for a robot sphere enlarged by 0.05 m and human clearance ≥0.05 m. The verified supporting floor is excluded from lateral obstacle hits. The first passing strip and tested offsets are saved. This is a sampled local feasibility diagnostic, not global planning or full-robot swept-volume proof.

The original geometry cohort contained 391 render candidates, including 180 prior-region candidates. **That exhaustive render cohort was not completed.** The new prefilter evaluates all 1,518 fixed positions.

## Render-free cheap filters

`research/prefilter_reveal_window.py` reuses hash-verified floor/overlap/bypass evidence for identical candidate poses and the unchanged scene/config. It evaluates all **1,518 × 47 = 71,346 adjacent pairs**, C1→C2 through C47→C48, from saved observation poses/chunks and actual saved application times. One and the same pair must satisfy remaining OLD conflict, positive lead, current clearance >0, application before conflict, and possible ray transition. Conditions passing at different pairs cannot be combined.

Only the 308 physically/bypass/temporally valid candidates require rays. Each uses 17 fixed samples: five XY offsets (centre, ±0.12 m X, ±0.12 m Y) at each leg/torso/head height (0.35/1.05/1.75 m), plus two auxiliary arm points. Fixed human yaw maps these approximate upright-body samples to world. They are not mesh occupancy or semantic pixels.

Saved USD camera matrices are row-vector camera-to-world transforms. They are checked against camera/body extrinsic × the full saved body transform (quaternion wxyz) within 1e−6. A world sample is multiplied by the inverse matrix; optical depth is −camera Z. A permissive approximate camera envelope uses positive depth and twice the aperture/focal half-angle slopes. It is not an exact fisheye projection. Each camera-to-sample segment queries actual Hospital PhysX colliders; the earliest Hospital hit before the sample blocks that ray. Nova and candidate geometry are absent from this query stage. Hit prim paths/distances and envelope/exposure flags are saved.

The fixed possibility screen requires, in each of three previous observations, at least one complete head/torso/leg group to be non-exposed (blocked or outside the broad envelope). Partial prior exposure is permitted. At FRESH, at least one sample must be exposed and newly exposed relative to immediate OLD. This is a permissive scheduling heuristic, **not a HIDDEN→CLEAR classification or a guaranteed necessary condition for pixel visibility**. Finite samples and the approximate lens envelope can miss transitions.

The prefilter performs **251,328 static ray queries, zero RGB/semantic captures**, no render products or annotators, no timeline play, no physics step/reset and no human authoring. Isaac initializes the static query environment with viewport updates disabled. The actual collider inventory matches all 2,161 frozen Hospital collision prims.

## Actual semantic visibility remains authoritative

For a new cheap-pass candidate, the renderer would capture all 48 saved poses using the unchanged front Hawk at **1920×1080 RGB plus semantic segmentation**, with stopped physics. The prefilter's favourable pair never substitutes for the first actual CLEAR. In this amended run there are **zero cheap survivors, so zero new preflight renders**. Existing 98×48 = **4,704** candidate observations are reused.

- HIDDEN: human semantic pixel count = 0.
- MARGINAL: count >0, failing at least one CLEAR criterion.
- CLEAR: fraction ≥0.002, bbox height ≥80 px and width ≥20 px.

Only the first CLEAR counts; OLD is its immediately previous request. The three immediate predecessors must all be HIDDEN. Thresholds are unchanged; MARGINAL is not redefined as HIDDEN. Unrendered candidates have **visibility unmeasured**, never inferred HIDDEN or never-CLEAR labels.

Paired absent/present RGB difference remains diagnostic. Before this search, the previous 125 semantic-hidden pairs all exceeded the former RGB gate (mean variation 3.19…9.90/255); renderer variation alone cannot prove human/shadow leakage. Any strict proposal requires direct hidden/reveal visual review before freezing selection. No strict proposal exists here.

## Remaining OLD horizon and temporal criterion

For OLD request j, project native p=(forward,left) using its own observation pose:

`p_world = [x_obs, y_obs] + R(yaw_obs) p_body`.

No ready/application re-anchor is used. Native arrays are unchanged, with neither yaw nor timestamp channels. Waypoint timing is assigned from the source-confirmed model target convention, **t_obs +0.1…+3.0 s**. At FRESH observation t_r, clip OLD to t≥t_r, adding a derived linear interpolation point at t_r if needed. There is no extrapolation beyond the native target domain.

For every retained segment a+u(b−a), solve `||a+u(b−a)−H||² ≤ r_combined²` for the earliest u∈[0,1]. Closed-circle tangency, degenerate segments and starting inside are handled explicitly. The earliest hit yields nominal conflict point/time. No spatial matching, model calls or new predictions are involved.

`lead = t_conflict − t_r`; `switch_margin = t_conflict − saved_baseline_application(C_k)`; `current_clearance = ||saved_robot_XY(t_r)−H|| − r_combined`. Qualification requires an in-horizon conflict and strictly positive lead, current clearance and switch margin. Saved application timing is descriptive; it does not predict future human-run inference latency.

## Unchanged selection and accounting

Hard filters combine physical validity, bypass, first-CLEAR/three-HIDDEN history and temporal conditions. Strict candidates rank by smallest positive saved switch margin, then larger first-CLEAR fraction, larger bypass human clearance, and lexicographic world XYZ. No weighted score or manual favourable-event substitution.

A=no OLD conflict at first CLEAR; B=MARGINAL before first CLEAR; C=current robot already intersects; D=no bypass; E=unsupported/static overlap; F=never CLEAR across all 48 measured images; G=nonpositive lead/switch margin; H=insufficient three-HIDDEN history. Failure counts are multi-label. For unrendered survivors of physical checks, P_TEMPORAL/P_RAY record cheap-filter rejection, not semantic outcomes. Exclusive plot categories are presentation only, not severity ranking.

## Results

The sequential funnel and its conditional rejections are:

| Gate | Retained | Rejected at this gate |
|---|---:|---:|
| Fixed candidates | 1,518 | — |
| Floor/static validity | 953 | 565 |
| Local bypass | 882 | 71 |
| Some remaining OLD conflict | 318 | 564 |
| Positive lead on that pair | 310 | 8 |
| Current clearance >0 on that pair | 308 | 2 |
| Saved application before conflict | 308 | 0 |
| Multi-ray transition on a qualifying pair | **0** | **308** |

Candidate-level reason records also report 1,131 positions with no remaining conflict at any pair across **all** 1,518, including physically invalid positions; this differs from the conditional funnel's 564. Full reasons and counts are in `geometry_candidates.json`, `prefilter_rejection_counts.json/.csv`; all pair metrics and ray traces are in `temporal_pair_prefilter.json` and `multi_ray_prefilter.json`.

There are 667 temporal-pass pairs before physical screening; 561 belong to the 308 physically valid candidates. Of those 561, 18 lack three prior observations. The other 543 have no newly exposed sample and at least one preceding observation with exposed samples in every body group; seven also have no exposed FRESH sample. These are overlapping diagnostic counts, not semantic pixel states.

**98 completed candidates reused, 0 newly rendered, 0 strict-qualified, selected_candidate=null.** Among those 98 measured candidates, all 98 fail A, B and H. Their first CLEAR spans C16–C38; OLD remaining minimum clearance is +6.229828…+7.862214 m. C/F/G counts are zero in this measured cohort, not across unrendered candidates. Across the complete candidate table: E=565, D=71, P_TEMPORAL=574, P_RAY=210, and B=98 are exclusive primary categories summing to 1,518. The reused 98 are included in the 308 ray-rejected candidates but retain their actual semantic evidence as the primary recorded diagnosis.

The old region's 192 positions are preserved in the new candidate set; 107 have a temporal-pass pair but zero pass all cheap gates. None of that region's planned renders had completed in the paused cohort, so this amendment **does not reproduce all of its semantic images**. Its previous independent evidence remains available unchanged.

Diagnostic candidate 80 (not selected) is at **(−21,10,0), yaw 0**. C35–C37 are MARGINAL; first CLEAR C38 has 4,891 pixels, fraction 0.002358700 and bbox 44×188 px. OLD C37 minimum clearance is +6.850042 m; current reveal clearance +7.998073 m. Reveal is 19.800001033 s, saved application 19.900001038 s, OLD horizon end 22.300001007 s. No remaining OLD conflict, lead or switch margin exists at that first CLEAR. Its later temporal-pass pair C47→C48 cannot replace the first reveal. The saved image sheet visually confirms gradual distant visibility, not a strict reveal.

Decision: **NO STRICT REVEAL-WINDOW CANDIDATE FOUND**. This is a filtered finite-search result; it is not proof that every cheap-rejected candidate would fail the final semantic criterion or that no Hospital placement could work.

## Artifacts, implementation and validation

Final output `outputs/nova-reveal-window-search-20261010-05` contains preregistration/source hashes, unchanged candidate IDs, reuse receipts, full cheap-filter/pair/ray records, candidate CSV/JSON, empty `qualified_strict.csv`, `near_misses.json`, null `selected_candidate.json` and `summary.json`. Scientific figures include search overview, prefilter funnel, ray/semantic temporal diagnostic, reveal-lead plot, world close-up, visibility timeline, conflict timeline and actual RGB/overlay sheet. When conflict/selection is absent, figures say so instead of fabricating a point or distribution. Derived overlays are written into the new output, never into reused source directories.

Pure rules are in `research/reveal_window.py` and `research/reveal_prefilter.py`; the geometry/render, prefilter, analyzer and GUI CLIs are separate. Final validation: **146 tests PASS (110 retained +36 new)**. Tests cover projection/timing-related existing contracts, clipping and exact conflict, first-CLEAR/no substitution, three-HIDDEN history, filter conjunction on the same pair, deterministic generation/ranking, semantic thresholds, ray possibility, source immutability/exclusive writes, navigation controls and forbidden model/render/physics operations. A subprocess regression proves an old exhaustive-render directory is refused before Isaac starts. Synthetic tests are contract tests, not experimental evidence.

Initial attempts remain preserved: -01 wrong PhysX interface; -02 unbounded prim handling; -03 bypass sphere/support-floor grazing; -04 corrected independent support-floor handling with unchanged radius/margin. Earlier performance interruption and disjoint lossless-PNG renderer shards are also retained. The user's subsequent cheap-prefilter amendment is recorded separately and did not change scientific selection thresholds.

To repeat the amended saved-only analysis, use a **fresh** directory (the following -06 name was unused at delivery):

```bash
bash scripts/isaac6_python.sh research/prefilter_reveal_window.py --output-dir outputs/nova-reveal-window-search-20261010-06
python3 research/search_reveal_window.py --phase render --render-part reuse --output-dir outputs/nova-reveal-window-search-20261010-06
MPLCONFIGDIR=/tmp/tic-vla-mpl python3 research/analyze_reveal_window.py --search-dir outputs/nova-reveal-window-search-20261010-06
```

The system-Python render command above uses the proven zero-survivor reuse-only path. A future configuration with genuine unrendered survivors must run that phase through `scripts/isaac6_python.sh`; unchanged semantic captures are permitted only after all cheap gates pass. Never resume the superseded exhaustive loop in -04.

## GUI

Validated non-headless **`outputs/reveal-window-gui-20261010-02/GUI_READY.json`**, left open. It checks candidate next/previous, strict/near filters, OLD/first-CLEAR jumps, observation navigation, saved-pose robot movement with a fixed human, archived overlay display, stopped physics and mask/log agreement. Hospital/Nova/human, remaining OLD and conflict proxy are present. A conflict point is correctly absent for diagnostic candidate 80. Overview/Hawk/follow screenshots and the actual desktop panel were visually inspected. Earlier -01 is preserved; -02 widens only the display follow camera so Nova is visible.

The GUI uses normal display viewports plus **archived 1920×1080 evidence**, with zero new semantic captures and no sensor preflight recapture of cheap-rejected candidates. It does not attach sensor annotators. Display guides are cyan baseline, orange OLD, magenta human, red conflict proxy/point, green bypass and yellow attributed occluder bounds. Roof clipping is display-only. Candidate 80 has no single attributed OLD centre-ray occluder, so no invented occluder is shown. The archived RGB/overlay panel is the visibility evidence; the guided viewport is a reconstruction for inspection.

Controls include previous/next/top candidate, strict/near/all, previous/next observation, Jump OLD/first CLEAR, replay/play/pause/slider, overview/front Hawk/follow and archived overlay toggle. Top opens diagnostic 80 because selection is null; Jump OLD goes to C37, Jump first CLEAR to C38. This is a MARGINAL→CLEAR near miss, not a staged HIDDEN reveal. Navigation physics and inference stay off.

```bash
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/nova-reveal-window-search-20261010-05 --output-dir outputs/reveal-window-gui-REVIEW
```

## Limitations and next allowed experiment

One saved closed-loop baseline, finite route-local positions, fixed camera/character orientation, conservative rest-pose bounds, approximate sample rays/camera envelope, nominal model timing and circular/sampled bypass proxies. The interrupted 98-candidate semantic cohort is neither exhaustive nor an unbiased sample. Ray false negatives are possible; a zero-survivor screen is not semantic proof of impossibility. Proxy overlap is not measured contact, and baseline chunks are a counterfactual for stationary-human placement, not predictions conditioned on that human.

**NOT EXECUTED — MODEL-FREE SEARCH TASK ONLY.** No human inference, baseline resampling, latency-causal hard-case claim, navigation-performance claim, FRESH-response evidence, correspondence validation or reconciliation validation. This task ends at an explicit no-candidate result. Any further scenario/filter revision or stationary-human model experiment belongs to a separately requested task; none is implemented or launched here.
