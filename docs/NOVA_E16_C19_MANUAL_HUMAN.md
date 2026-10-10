# Single frozen C19 human placement

**Physical PASS; semantic strict-reveal FAIL. No human model run executed.**
The person is already CLEAR at the first saved observation C1. There is no
preceding OLD chunk or three-observation HIDDEN history. The position was not
adjusted and no alternative was searched.

## Freeze and physical check

Starting clean local/fetched origin `hardcase-probe`:
`6cd4517ff9a2401fedb39753bf72d0204b869cde`; upstream/main:
`9fa6f8b66b9e121d5df5df071297bba8e5353ebb`.
Source: `outputs/nova-e16-hospital-lights-baseline-20261010-01`.

| Quantity | Exact saved / derived value |
|---|---|
| C19 observation time | 10.300000537186861 s |
| C19 world X, Y | 1.9812549352645874, 11.180365562438965 m |
| C19 yaw | 3.030531579778966 rad (173.6366692024487 degrees) |
| Frozen human XYZ | (1.9812549352645874, 11.580365562438965, 0.0) m |
| Frozen yaw | -0.11106107381082708 rad (-6.36333079755131 degrees) |

The user-specified formula is `X=C19.X`, `Y=C19.Y+0.40`, `Z=0`,
`yaw=wrap(C19.yaw+pi)`. This is a **world +Y** offset, not a robot lateral offset.
Hospital is Z-up, coordinates are metres, yaw is CCW from world +X about +Z.
The receipt was created before any human visibility render:
`outputs/e16-c19-manual-20261010-01/freeze.json`, SHA256
`50311853964c2f847a03f41ac98fd719a04bdffb9478a008ab38a5e56ff70341`.
It binds the exact source observation, placement, visibility rules, code and
353 source-evidence hashes. The freeze/physical/semantic script stayed byte
identical through these stages.

Exactly one static physical check reused the existing Hospital queries and
human capsule plus yaw-oriented enclosing visual bounds. Ground support is
`/Root/GroundPlane/CollisionPlane`; static-overlap paths are empty. Initial
Nova-human proxy clearance is **10.584927515324896 m**. Existing radii are
Nova **0.6069825421953869 m**, human **0.25 m**, combined **0.8569825421953869 m**.
The sampled two-metre bypass strip on the south side passes, clearance
**0.14301745780461306 m**, with the existing +0.05 m robot safety margin.
This local proxy check does not establish complete navigation feasibility.
All 2,161 Hospital collider records match the original audit. Physical stage:
zero RGB renders, model calls and navigation physics steps.

## Authoritative semantic preflight

One fixed human visual was authored before the first capture, with no movement,
teleport, delayed spawn or visibility manipulation. All 48 **saved baseline**
robot poses were rendered with the unchanged 1920x1080 front Hawk and E16
fixture-light profile. The exact full saved body pose, quaternion **w,x,y,z**,
is replayed; each camera matrix matches camera-to-body times saved body-to-world
in USD row-vector convention within 1e-6. Camera local forward is -Z, +X right,
+Y up. Captures keep timeline time fixed; they do not rerun robot navigation.

Frozen thresholds: HIDDEN = 0 semantic human pixels; CLEAR requires fraction
>=0.002 and bbox height>=80, width>=20 pixels; other nonzero visibility is
MARGINAL. Semantic pixels, not rays or RGB appearance, determine these states.

| Saved observations | State |
|---|---|
| C1-C18 | CLEAR |
| C19-C48 | HIDDEN |

First CLEAR is **C1 at t=0**: **4,381 pixels**, fraction
**0.0021127507716049384**, bbox `[477,398,561,547]` = **84x149 pixels**.
The raw RGB/overlay shows the person visible beside the corridor cart. There
are no earlier saved observations. Thus OLD ID, OLD remaining clearance,
predicted OLD conflict, reveal lead and saved switch margin are **undefined**.
Current robot-human clearance at C1 is positive, **10.584927515324896 m**.
Failure codes H/A reflect missing hidden history and no OLD conflict at the
actual first event; they do not claim a measured positive OLD clearance.
No later pair replaces C1. Later HIDDEN means no semantic pixels, not proof
that a wall occludes the human (the person can leave the camera view).

Existing nominal OLD logic remains own-observation world projection,
`p_world = translation_obs + R(yaw_obs) @ [forward,left]`, with source-assigned
`t_obs+0.1...3.0 s` target times. No native yaw/timestamp channel or spatial
correspondence is introduced. There is no applicable OLD window for C1.

**Stage C NOT EXECUTED.** The no-human baseline itself remains validated; this
failure concerns the manual strict-reveal placement. No lighting, route,
speed, controller, cadence, checkpoint or geometry was changed. No inference,
human navigation run, response revision, latency-causal hard-case, navigation
performance, correspondence or reconciliation result is claimed.

## Artifacts and saved replay

Preflight: `outputs/e16-c19-manual-20261010-01`, including immutable receipt,
physical result, 48 raw RGB/mask/derived overlays, camera matrices and complete
candidate/visibility result. The official human stays in its authored rest pose;
the capsule is a proxy, not a complete mesh collision representation.

Analysis: `outputs/e16-c19-manual-analysis-20261010-01`:

- `trajectory_result.png/.pdf`: no-human actual path and C1-C48 observation
  points colored by semantic state, frozen human/proxy, structural bounds and
  exact failed gate.
- `trajectory_turn_zoom.png/.pdf`: the same world geometry at readable turn
  scale. Equal XY units in metres; markers are observation poses, not native
  trajectory points.
- `trajectory_coordinates.json` and `observation_poses_visibility.csv`: exact
  plotted coordinates/states; dense path source and values are also recorded.
- `first_reveal_sheet.png`: C1 first CLEAR and C2; unavailable prior/OLD frames
  are explicitly identified. No fabricated negative request IDs or response.
- `viewer_data/`: small adapter pointing to original archived RGB/masks.

Revision plots are intentionally absent: no human-conditioned FRESH response
exists. The existing viewer was generalized only for first CLEAR at C1:
boundary-safe observation-navigation self-test, an overview including the
recorded robot when OLD is absent, and a single-placement/preflight-only label.
There is no invented OLD curve or conflict marker.

```bash
bash scripts/isaac6_python.sh research/view_reveal_window_search.py --search-dir outputs/e16-c19-manual-analysis-20261010-01/viewer_data --output-dir outputs/e16-c19-manual-gui-REVIEW
```

Validation: three focused tests pass (exact immutable world-Y offset/facing,
first-CLEAR/no substitution with conflict timing, and plot coordinate identity).
All 48 masks are independently checked against logged count/bbox/state; source
hashes and freeze are preserved. PNG/PDF and first-reveal sheet were visually
reviewed. Saved GUI checks/scene identity are recorded under
`outputs/e16-c19-manual-gui-20261010-01`; model calls and navigation physics
reexecution remain zero. Generated evidence is not committed.
