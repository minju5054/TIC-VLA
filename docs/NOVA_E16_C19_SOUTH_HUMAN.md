# C19 minus 0.40 m world Y: single stationary-human preflight

**Physical PASS; strict reveal FAIL; human navigation/model run NOT EXECUTED.**
The actual first CLEAR is C4. C1–C3 are already MARGINAL, and C3's remaining
nominal future does not conflict with the fixed human. No other location,
offset, later pair, threshold or model run was tried. The previous
[+0.40 m experiment](NOVA_E16_C19_MANUAL_HUMAN.md) remains negative evidence.

Starting clean local/fetched origin `hardcase-probe`:
`53b5382212193d1984283bd95acf35385654cd9f`; fetched upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
Source: `outputs/nova-e16-hospital-lights-baseline-20261010-01` (48 finite
predictions, 1,519 measured poses). No baseline/inference/navigation rerun.

## Immutable placement and physical gate

| Quantity | Exact saved / derived value |
|---|---|
| C19 observation time | 10.300000537186861 s |
| C19 world X, Y | 1.9812549352645874, 11.180365562438965 m |
| C19 yaw | 3.030531579778966 rad |
| Human XYZ | (1.9812549352645874, 10.780365562438964, 0.0) m |
| Human yaw | -0.11106107381082708 rad |

Formula: `X=C19.X; Y=C19.Y-0.40; Z=0; yaw=wrap(C19.yaw+pi)`.
This offset is world −Y, not robot lateral. Hospital world metres, +Z up,
yaw CCW from +X, saved quaternions ordered w,x,y,z. The last decimal reflects
direct floating-point subtraction from the source, not rounded input.
Receipt `outputs/e16-c19-south-20261010-01/freeze.json`, SHA256
`63b05e643bdafea532381ed96d7e8010660c20a94c30d575eea69bad5f4fd88a`, was
written before rendering. It binds one location, rules and 353 source hashes.
The executed freeze/physical/semantic script remained byte-identical.

Floor support `/Root/GroundPlane/CollisionPlane` PASS; static overlap paths
empty; initial robot-human proxy clearance **+9.886213433602205 m**;
sampled local bypass PASS, clearance **+0.14301745780461306 m**. Unchanged
Nova radius **0.6069825421953869 m**, human radius **0.25 m**, combined
**0.8569825421953869 m**. The existing capsule plus oriented visual bounds
and two-metre sampled bypass are local proxy checks, not complete navigation
feasibility. All 2,161 Hospital collider records match the frozen inventory.

## Authoritative visibility and first-event gate

The fixed human was authored before C1 and stayed at the same world transform
through all 48 captures. Saved full body poses reconstruct the original Hawk
extrinsic; camera matrices match within 1e-6. Original lights-on fixture
profile, 1920×1080 camera and semantic thresholds are unchanged. HIDDEN means
zero human pixels; CLEAR requires fraction ≥0.002, bbox width ≥20 and height
≥80; other nonzero masks are MARGINAL. No rays replace this oracle.

| Request | Simulation time [s] | State | Human pixels | Bbox width × height |
|---|---:|---|---:|---:|
| C1 | 0 | MARGINAL | 886 | 65×137 |
| C2 | 1.8000000938773155 | MARGINAL | 879 | 65×137 |
| C3 (immediate OLD) | 2.3000001199543476 | MARGINAL | 2,120 | 66×164 |
| C4 (first CLEAR) | 2.8000001460313797 | CLEAR | 5,223 | 86×176 |

Complete timeline: **C1–C3 MARGINAL; C4–C18 CLEAR; C19–C48 HIDDEN**.
The CSV records every observation; original RGB, masks, overlays and per-frame
camera matrices remain in the preflight directory. Later HIDDEN can mean
outside camera view, not necessarily structural occlusion.

At first CLEAR: current robot-human clearance **+8.8978543893842 m**;
OLD C3 remaining nominal future clearance **+7.647539756449073 m**;
predicted conflict **none**. Baseline C4 application is
**2.9333334863185883 s**; conflict lead and switch-to-conflict margin are
undefined, not passing. Remaining OLD is projected using C3's own observation
pose and clipped at C4 observation time; target times come from the confirmed
+0.1…3.0 s convention, not a native timestamp/yaw channel. Failure codes:
**B** (OLD MARGINAL), **H** (no three prior HIDDEN), **A** (no OLD conflict).
No later favorable pair substitutes for C4.

## Plots and saved replay

Final figures: `outputs/e16-c19-south-analysis-20261010-03/`:
`trajectory_result.png/.pdf`, `trajectory_turn_zoom.png/.pdf` and
`first_reveal_sheet.png`. Exact C observation markers, measured baseline
path, fixed human/proxy, first CLEAR and OLD remaining curve are shown with
structural XY bounds and equal metres. C3/C4 lie outside the turn zoom and
are identified on the full map. Label offsets never move data markers.
`trajectory_coordinates.json` and `observation_poses_visibility.csv` export
the underlying coordinates/history. No human-conditioned FRESH, human-run
path or revision figure exists. Analysis -01 and incomplete -02 are retained;
-03 corrects label placement and a plotting metadata-key error only.

The existing viewer now opens paused at **t=0/C1**. Controls: Restart from
start, Play/Pause, 0.25×/0.5×/1×/2× speed, timeline slider, OLD, first CLEAR,
saved application, Overview, Follow robot and Nova front Hawk. A saved RGB
panel with semantic overlay uses archived observation evidence, held until
the next observation. The live viewport is a display reconstruction, not a
new observation. Full replay uses saved measured poses with the stationary
human inserted throughout and the label **PREFLIGHT ONLY — NO HUMAN
NAVIGATION RUN**. No model calls or navigation physics reexecution.

```bash
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/e16-c19-south-analysis-20261010-03/viewer_data --output-dir outputs/e16-c19-south-gui-REVIEW
```

Use a fresh GUI output name for each launch. `--self-test --export-full`
validates controls and exports the entire saved drive at 10 fps with an
archived Hawk semantic inset, timestamps and frame/source manifest. Existing
ffmpeg is used if available; no video-only installation is performed.

Validated replay: **`outputs/e16-c19-south-gui-20261010-03`**, `GUI_READY`
paused at **t=0/C1**, visually verified on the desktop. `frames/` contains
**255 PNGs, 1600×1000**, covering **0–25.300001319497824 s**, including the
exact final saved pose. `frames.json` maps every frame to measured tick/pose
and archived RGB/mask; all 48 observations occur in the export. Sampling
holds the most recent measured 60 Hz pose, with maximum age
**0.016666662320494657 s**; it does not simulate or invent a path. Original
Hawk images are held at their separately displayed observation timestamps.
`video.json` records ffmpeg unavailable: no MP4 and no package installation.
Both earlier GUI attempts remain preserved; they exposed asynchronous PNG
write completion, fixed by waiting for the capture and full PNG decode.

Validation: **5 focused tests PASS** (legacy/exact minus-world-Y placement,
receipt and source immutability, first-CLEAR/no substitution, t=0/full-run
export extent, observation-coordinate identity). All 48 semantic masks were
independently recomputed; PNG/PDF and representative replay frames visually
checked. GUI self-checks pass for restart, speed, observation/OLD/CLEAR/
application navigation, fixed human, original-mask display and stopped
physics. All 255 PNGs decode and robot XYZ exactly matches the logged saved
tick. Frozen 353 sources and 177 previous +0.40 m artifact files remain
unchanged. Detailed checks: `outputs/e16-c19-south-setup-20261010-01/`.

## Interpretation

This manually specified point fails the frozen strict-reveal prerequisite.
It does not establish navigation difficulty, a human-conditioned response,
latency-induced hard case, correspondence or reconciliation. This preflight
uses stopped saved poses; the source baseline's continuous OLD execution
settings remain unchanged. There is no new model or physics experiment.
