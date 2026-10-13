"""CPU geometry/contract tests; these are not Isaac or navigation evidence."""
import ast
import json
from pathlib import Path
import numpy as np
import pytest
from research.two_hz_preflight import (POLICY, PRIMARY, SECONDARY, RADIUS, SEPARATION,
    references, candidates, point_path, rect_path, sample_path, excursion, bypass_vertices,
    early_clear, select, load_reference, digest)
from research.records import write_json
from research.analyze_two_hz_preflight import section_valid


def geometry():
    return {'entrance_x_m':2.5521036832844546,'center_y_m':10.599539794921895,
            'cross_sections':[{'x_m':0.,'valid':True},{'x_m':-1.,'valid':False}]}


def test_actual_receipt_identities():
    paths=references()
    assert paths[:2]==[PRIMARY,SECONDARY]
    assert [Path(p).name for p in paths[2:]]==[
        'nova-e16-zero-repeat-20261011-01-r01-no_hook','nova-e16-zero-repeat-20261011-01-r02-zero_hook',
        'nova-e16-zero-repeat-20261011-01-r03-zero_hook','nova-e16-zero-repeat-20261011-01-r04-no_hook',
        'nova-e16-zero-repeat-20261011-01-r05-no_hook','nova-e16-zero-repeat-20261011-01-r06-zero_hook']


@pytest.mark.parametrize('path',references())
def test_historical_goal_cutoff_and_success(path):
    r=load_reference(path);dist=np.linalg.norm(r['xy']-r['cfg']['scene']['goal'][:2],axis=1)
    assert dist[-1]<2 and np.all(dist[:-1]>=2)
    assert r['validation']['full_rows']>len(r['rows'])
    assert not r['validation']['nonfloor_contacts'] and r['validation']['west_gate_pass']
    assert all(e['observation']['sim_time']<=r['times'][-1] for e in r['events'])


def test_independent_corridor_and_left_sign():
    g=geometry();grid=candidates(g)
    assert len(grid)==12 and grid==candidates(g)
    assert grid[1]['position_world_xyz'][1]==g['center_y_m']-.25
    assert grid[2]['position_world_xyz'][1]==g['center_y_m']+.25
    assert grid[0]['position_world_xyz'][0]==g['entrance_x_m']-3
    # No robot trajectory argument exists in independent grid generation.
    assert [r['id'] for r in grid]==[f'H{i:02d}' for i in range(1,13)]


def test_invalid_sections_not_filled_from_path():
    assert section_valid(0.,geometry())
    assert not section_valid(-1.,geometry())
    assert not section_valid(1.,geometry())


def test_continuous_conflict_between_endpoints():
    r=point_path([[-2.,0.],[2.,0.]],[0.,0.],.5,np.array([0.,4.]))
    assert r['conflict'] and r['distance_m']==0
    assert r['first_conflict']['fraction']==pytest.approx(.375)
    assert r['closest_arc_m']==2.


def test_tangent_is_not_strict_baseline_conflict():
    assert not point_path([[-2.,.5],[2.,.5]],[0.,0.],.5)['conflict']


def test_continuous_cart_footprint_distance():
    assert rect_path([[-2,2],[2,2]],[-1,-1,0],[1,1,1],.6)['clearance_m']==pytest.approx(.4)
    assert rect_path([[-2,0],[2,0]],[-1,-1,0],[1,1,1])['clearance_m']==-1


def test_excursion_and_return_screen():
    s=np.linspace(-3,3,601);v=np.zeros_like(s);v[(s>=-.75)&(s<=.75)]=.25
    assert excursion(s,v,0,np.ones(len(s),bool))['classification']=='LOCAL_LATERAL_EXCURSION_OBSERVED'
    assert excursion(s,-v,0,np.ones(len(s),bool))['maximum_negative_residual_m']==pytest.approx(-.25)


def test_no_excursion_without_return_or_persistence():
    s=np.linspace(-3,3,601);v=np.where(s>=-.5,.3,0)
    assert excursion(s,v,0,np.ones(len(s),bool))['classification']=='PASSIVE_CORRIDOR_FOLLOWING_CONSISTENT'
    v=np.where(abs(s)<.1,.3,0)
    assert excursion(s,v,0,np.ones(len(s),bool))['persistence_m']<.5
    assert excursion(s,v,0,np.zeros(len(s),bool))['classification']=='INSUFFICIENT_GEOMETRIC_EVIDENCE'


def test_bypass_is_connected_both_sides_and_downstream():
    ref={'xy':np.column_stack([np.linspace(5,-5,1001),np.zeros(1001)])}
    c={'position_world_xyz':[0,0,0]}
    for side,y in [('LEFT',-1.),('RIGHT',1.)]:
        vertices=bypass_vertices(ref,c,side)
        assert vertices[0].tolist()==[2.5,0.] and vertices[-1].tolist()==[-2.5,0.]
        assert vertices[1,1]==y and vertices[2,1]==y
        samples=sample_path(vertices)
        assert np.linalg.norm(np.diff(samples,axis=0),axis=1).max()<=.05+1e-12
        assert point_path(vertices,[0,0])['distance_m']>=SEPARATION
        assert 2.5-SEPARATION>=1.
    with pytest.raises(ValueError):bypass_vertices(ref,c,'FORWARD')
    with pytest.raises(ValueError):sample_path([[0,0],[1,0]],.051)


def test_missing_upstream_connector_rejected():
    with pytest.raises(ValueError):bypass_vertices({'xy':np.array([[1,0],[-1,0]])},{'position_world_xyz':[0,0,0]},'LEFT')


def test_robot_human_margin_identity():
    assert RADIUS==.6069825421953869
    assert SEPARATION==pytest.approx(.9069825421953869)
    assert POLICY['human_radius_m']==.25 and POLICY['static_margin_m']==.05


def clear_row():
    return {'human_visible_fraction':.002,'human_bbox_if_visible':[0,0,20,80],
            'remaining_along_track_m':5.,'before_conflict':True,'render_valid':True,'brightness_pass':True}


@pytest.mark.parametrize('key,value',[('human_visible_fraction',.00199),('human_bbox_if_visible',[0,0,19,80]),
    ('human_bbox_if_visible',[0,0,20,79]),('remaining_along_track_m',4.999),('before_conflict',False),
    ('render_valid',False),('brightness_pass',False)])
def test_early_clear_requires_all_gates(key,value):
    r=clear_row();assert early_clear(r);r[key]=value;assert not early_clear(r)


def test_selection_priority_is_frozen():
    rows=[{'id':'H02','qualified':True,'best_structural_clearance_m':.2,'early_clear_distance_m':9.,'left_offset_m':0},
          {'id':'H01','qualified':True,'best_structural_clearance_m':.3,'early_clear_distance_m':5.,'left_offset_m':.25}]
    assert select(rows)['id']=='H01'
    rows[1]['best_structural_clearance_m']=.2
    assert select(rows)['id']=='H02'
    rows[1]['early_clear_distance_m']=9.;rows[1]['left_offset_m']=0
    assert select(rows)['id']=='H01'
    assert select([]) is None


def test_output_exclusivity_and_raw_unchanged(tmp_path):
    p=tmp_path/'receipt.json';write_json(p,{'value':1});before=digest(p)
    with pytest.raises(FileExistsError):write_json(p,{'value':2})
    assert digest(p)==before


def test_no_model_or_navigation_calls_in_additive_code():
    forbidden={'InferenceClient','predict_async','predict_action','torch.load','SimulationContext','SimulationManager','TICVLA','sim.step','timeline.play'}
    for p in [Path('research')/n for n in ['two_hz_preflight.py','two_hz_preflight_isaac.py','analyze_two_hz_preflight.py',
            'two_hz_preflight_render.py','verify_two_hz_preflight.py','plot_two_hz_preflight_review.py']]:
        tree=ast.parse(p.read_text())
        for node in ast.walk(tree):
            if isinstance(node,ast.Call):assert ast.unparse(node.func) not in forbidden
            if isinstance(node,ast.Import):assert not any(a.name.startswith(('torch','transformers','DynaNav')) for a in node.names)
            if isinstance(node,ast.ImportFrom):assert not (node.module or '').startswith(('torch','transformers','DynaNav'))


def test_saved_camera_and_rgb_contract():
    source=Path('research/two_hz_preflight_isaac.py').read_text()
    assert 'rotation_wxyz(obs[\'quaternion_wxyz\']).T' in source
    assert 'matrix-expected' in source and 'semantic.get_data()' in source
    assert 'source_rgb_sha256' in source and 'brightness_pass' in source
    assert 'render.capture()' in source
    helper=Path('research/nova_render_saved.py').read_text()
    assert 'delta_time=0.' in helper and 'wait_for_render=True' in helper


def test_physical_queries_and_source_identity_contract():
    source=Path('research/two_hz_preflight_isaac.py').read_text()
    for token in ['floor_probes','failure_samples','continuous_aabb_checks','colliders(stage)!=before',
                  'SM_SupplyCart_','source_bytes(url)','GetAppliedItems','human_asset_audit']:
        assert token in source
