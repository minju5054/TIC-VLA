"""Saved-only cart analysis, candidate freeze, and model-free result publication."""
import argparse
import csv
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.two_hz_preflight import (read, digest, canonical, POLICY, PRIMARY, RADIUS,
    references, load_reference, rect_path, excursion, candidates, select, early_clear)
from research.records import write_json


def section_valid(x, geometry):
    sections=geometry['cross_sections'];xs=np.array([r['x_m'] for r in sections])
    if x<xs.min() or x>xs.max(): return False
    i=int(np.argmin(abs(xs-x)))
    return sections[i]['valid']


def validate_geometry(out):
    """Pre-candidate geometry clarification: closed doors are known boundaries."""
    source=out/'stage-a/corridor_geometry.json';g=read(source)
    for s in g['cross_sections']:
        s['valid']=all(h['hit'] and any(w in h['hit']['prim'] for w in ['Wall','Door','Trim']) and
            abs(h['hit']['normal'][1])>.99 and
            abs(h['hit']['position'][1]-(g['south_y_m'] if h['direction_world_y']==-1 else g['north_y_m']))<=.075
            for h in s['queries'])
        s['invalid_reason']=None if s['valid'] else 'No unambiguous parallel structural wall/closed-door boundary within .075m of nominal face'
    g['cross_section_validity']='Wall/closed-door/trim nearest hits at both heights, normal within .99 of Y, within .075m of nominal parallel wall plane. Recessed closed doors retain actual hit coordinates; furniture/open recesses invalid.'
    g['pre_candidate_geometry_revision']={'source_sha256':digest(source),
        'reason':'Initial screen excluded verified closed doors by name and .03m tolerance; actual north doors are recessed .05177m. Treat known closed parallel doors as unambiguous boundaries before any human feasibility/RGB; candidate grid unchanged.',
        'not_a_candidate_threshold_relaxation':True,'candidate_queries_performed':0,'candidate_renders_performed':0}
    write_json(out/'stage-a/corridor_geometry_validated.json',g)


def analyze(out):
    folder=out/'stage-a';g=read(folder/'corridor_geometry_validated.json')
    refs=[load_reference(p) for p in references()];inventory=read(folder/'native_cart_inventory.json')
    all_rows=[];passages=[];near=set();summaries=[]
    for r in refs:
        xy=r['xy'];s=g['entrance_x_m']-xy[:,0];left=g['center_y_m']-xy[:,1]
        west=(xy[:,0]<=g['entrance_x_m'])&(xy[:,0]>=g['goal_region_east_extent_x_m'])
        valid=np.array([section_valid(x,g) for x in xy[:,0]]) & west
        yaw=np.unwrap([float(row['yaw']) for row in r['rows']]);ds=np.gradient(r['arc'])
        curvature=np.divide(np.gradient(yaw),ds,out=np.zeros_like(ds),where=ds>1e-5)
        wall=np.minimum(xy[:,1]-g['south_y_m'],g['north_y_m']-xy[:,1])-RADIUS
        for i in np.flatnonzero(west):
            row=r['rows'][i]
            all_rows.append({'run':Path(r['path']).name,'tick':row['tick'],'sim_time_s':r['times'][i],
                'x_world_m':xy[i,0],'y_world_m':xy[i,1],'progress_west_m':s[i],
                'offset_left_m':left[i],'cross_section_valid':bool(valid[i]),
                'parallel_wall_proxy_clearance_m':wall[i],'yaw_rad':yaw[i],
                'measured_curvature_rad_per_m':curvature[i],'command_applied_w_rad_s':float(row['applied_w']),
                'measured_body_yaw_rate_rad_s':float(row['body_yaw_rate_measured'])})
        summaries.append({'run':r['path'],'valid_section_samples':int(valid.sum()),
            'left_offset_min_m':float(left[valid].min()),'left_offset_max_m':float(left[valid].max()),
            'max_absolute_centerline_offset_m':float(abs(left[valid]).max()),
            'minimum_parallel_wall_proxy_clearance_m':float(wall[valid].min())})
        for cart in inventory:
            lo=np.array(cart['world_aabb_min']);hi=np.array(cart['world_aabb_max'])
            q=np.maximum(np.maximum(lo[:2]-xy,xy-hi[:2]),0)
            if np.linalg.norm(q,axis=1).min()>POLICY['near_cart_surface_distance_m']+.05: continue
            approach=rect_path(xy,lo,hi,RADIUS)
            if approach['clearance_m']+RADIUS>POLICY['near_cart_surface_distance_m']:continue
            near.add(cart['prim']);idx=approach['segment'];u=approach['fraction'];closest=xy[idx]+u*(xy[idx+1]-xy[idx])
            station=g['entrance_x_m']-(lo[0]+hi[0])/2
            in_corridor=hi[1]>=g['south_y_m'] and lo[1]<=g['north_y_m'] and station>=0
            crossed=bool(s.min()<=station<=s.max())
            result=excursion(s,left,station,valid)
            if not in_corridor or approach['clearance_m']>POLICY['meaningful_cart_robot_clearance_max_m']:
                result={'classification':'INSUFFICIENT_GEOMETRIC_EVIDENCE',
                    'reason':'Cart outside validated west corridor or beyond preregistered meaningful proximity; no eligible straight-corridor passage'}
            mask=abs(s-station)<=3.
            commands=np.array([float(row['applied_w']) for row in r['rows']])
            rates=np.array([float(row['body_yaw_rate_measured']) for row in r['rows']])
            passages.append({'run':r['path'],'cart_prim':cart['prim'],'asset':cart['asset'],
                'closest_cart_aabb_robot_clearance_m':approach['clearance_m'],
                'closest_sim_time_s':float(r['times'][idx]+u*(r['times'][idx+1]-r['times'][idx])),
                'closest_world_xy':closest.tolist(),'closest_left_offset_m':float(g['center_y_m']-closest[1]),
                'closest_parallel_wall_proxy_clearance_m':float(min(closest[1]-g['south_y_m'],g['north_y_m']-closest[1])-RADIUS) if closest[0]<=g['entrance_x_m'] else None,
                'cart_west_station_m':station,'cart_footprint_intersects_corridor':bool(in_corridor),
                'west_station_crossed_before_goal':crossed,'valid_cart_passage':result['classification']!='INSUFFICIENT_GEOMETRIC_EVIDENCE',
                'local_measured_yaw_change_rad':float(yaw[mask][-1]-yaw[mask][0]) if mask.any() else None,
                'local_curvature_min_max_rad_per_m':[float(curvature[mask].min()),float(curvature[mask].max())] if mask.any() else None,
                'local_applied_w_min_max_rad_s':[float(commands[mask].min()),float(commands[mask].max())] if mask.any() else None,
                'local_measured_yaw_rate_min_max_rad_s':[float(rates[mask].min()),float(rates[mask].max())] if mask.any() else None,
                **result})
    with (folder/'path_centerline_offsets.csv').open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(all_rows[0]));writer.writeheader();writer.writerows(all_rows)
    write_json(folder/'cart_passages.json',passages)
    valid=[p for p in passages if p['valid_cart_passage']]
    summary={'classification': 'INSUFFICIENT_GEOMETRIC_EVIDENCE' if not valid else
        ('LOCAL_LATERAL_EXCURSION_OBSERVED' if any(p['classification']=='LOCAL_LATERAL_EXCURSION_OBSERVED' for p in valid) else 'PASSIVE_CORRIDOR_FOLLOWING_CONSISTENT'),
        'original_cart_instances':len(inventory),'near_route_cart_instances':len(near),'near_route_cart_prims':sorted(near),
        'run_cart_proximity_records':len(passages),'valid_cart_passages':len(valid),
        'max_local_excursion_m':max((p['maximum_absolute_residual_m'] for p in valid),default=None),
        'min_near_cart_clearance_m':min((p['closest_cart_aabb_robot_clearance_m'] for p in passages),default=None),
        'corridor_following_descriptive':summaries,'causality':'Unchanged-scene observation cannot identify cart-caused steering; no memorization or leakage tested',
        'policy_sha256':digest(out/'analysis-policy.json'),'model_calls':0,'physics_steps':0}
    write_json(folder/'stage-a-summary.json',summary)
    plots(out,refs,g,inventory,passages)
    print('STAGE_A',summary['classification'],len(near),len(valid),flush=True)


def base_axes(ax,geometry,collisions):
    from matplotlib.patches import Rectangle
    for c in collisions:
        if c['world_aabb_min'] is None:continue
        lo=np.array(c['world_aabb_min']);hi=np.array(c['world_aabb_max'])
        if lo[2]>1.3 or hi[2]<.1 or hi[0]<-15 or lo[0]>10 or hi[1]<0 or lo[1]>16:continue
        ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],facecolor='.85',edgecolor='.6',lw=.3))
    ax.axhline(geometry['center_y_m'],ls='--',color='black',lw=.8,label='structural centerline')
    ax.set_aspect('equal');ax.set_xlabel('World X (m)');ax.set_ylabel('World Y (m)')


def plots(out,refs,g,inventory,passages):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle,Circle
    folder=out/'stage-a/figures';folder.mkdir(exist_ok=False)
    collision=read(out/'stage-a/collision_geometry.json')
    fig,ax=plt.subplots(figsize=(13,7));base_axes(ax,g,collision)
    for i,r in enumerate(refs):ax.plot(*r['xy'].T,lw=1.2,label='Primary' if i==0 else ('Secondary' if i==1 else f'Repeat {i-1}'))
    for c in inventory:
        if c['prim'] not in {p['cart_prim'] for p in passages}:continue
        lo=np.array(c['world_aabb_min']);hi=np.array(c['world_aabb_max'])
        ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],fill=False,edgecolor='red',lw=1.3))
        ax.text(lo[0],hi[1],c['prim'].split('/')[-1],fontsize=6)
    ax.set_xlim(-14,9);ax.set_ylim(0,15.5);ax.legend(fontsize=7,ncol=3)
    ax.set_title('Measured goal-cut paths; original cart/structure AABB proxies')
    fig.tight_layout();fig.savefig(folder/'corridor-path-carts.png',dpi=170);plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,4))
    for i,r in enumerate(refs):
        s=g['entrance_x_m']-r['xy'][:,0];left=g['center_y_m']-r['xy'][:,1];valid=[section_valid(x,g) for x in r['xy'][:,0]]
        ax.plot(s,np.where(valid,left,np.nan),label='Primary' if i==0 else ('Secondary' if i==1 else f'Repeat {i-1}'))
    ax.axhline(0,color='black',ls='--');ax.set_xlim(0,13);ax.set_xlabel('West progress from structural gate (m)');ax.set_ylabel('Left offset, -world Y (m)')
    ax.set_title('Measured corridor offsets; gaps denote invalid structural cross-sections');ax.legend(fontsize=8,ncol=4)
    fig.tight_layout();fig.savefig(folder/'offset-progress.png',dpi=170);plt.close(fig)
    for j,p in enumerate([p for p in passages if p['run']==PRIMARY]):
        fig,ax=plt.subplots(figsize=(7,7));base_axes(ax,g,collision);ax.plot(*refs[0]['xy'].T,color='blue',label='measured primary')
        c=next(c for c in inventory if c['prim']==p['cart_prim']);lo=np.array(c['world_aabb_min']);hi=np.array(c['world_aabb_max'])
        ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],fill=False,edgecolor='red',lw=2));xy=p['closest_world_xy']
        ax.add_patch(Circle(xy,RADIUS,fill=False,edgecolor='blue',label='conservative robot disk'))
        mid=(np.array(xy)+(lo[:2]+hi[:2])/2)/2;ax.set_xlim(mid[0]-3.5,mid[0]+3.5);ax.set_ylim(mid[1]-3.5,mid[1]+3.5)
        ax.set_title(c['prim'].split('/')[-1]+'\n'+p['classification'],fontsize=9);ax.legend(fontsize=7)
        fig.tight_layout();fig.savefig(folder/f'near-cart-{j+1:02d}.png',dpi=160);plt.close(fig)


def freeze(out,config_path):
    g=read(out/'stage-a/corridor_geometry_validated.json');rows=candidates(g)
    # Station validity uses geometry only, before human-specific queries or RGB.
    for c in rows:
        c['station_valid']=bool(section_valid(c['position_world_xyz'][0],g) and
            g['goal_region_east_extent_x_m']<c['position_world_xyz'][0]<g['entrance_x_m'])
        c['invalid_reason']=None if c['station_valid'] else 'Grid station lacks a valid structural cross-section; retain invalid, do not relocate'
    asset=read('outputs/nova-reveal-window-search-20261010-04/human_asset.json')
    plan={'schema':1,'policy':POLICY,'geometry':{k:g[k] for k in ['entrance_x_m','south_y_m','north_y_m','center_y_m','width_m','goal_region_east_extent_x_m']},
        'references':references(),'primary':PRIMARY,'candidates':rows,
        'human':{'prim':'/World/PreflightHuman','asset':asset['asset'],'radius_m':.25,'capsule_cylinder_height_m':1.2},
        'human_asset_audit':asset,'geometry_sha256':digest(out/'stage-a/corridor_geometry_validated.json'),
        'reference_validation_sha256':digest(out/'reference-validation.json'),'historical_receipt_sha256':digest('outputs/e16-zero-repeatability-plan-20261011-01/receipt.json')}
    folder=out/'stage-b';folder.mkdir(exist_ok=False)
    write_json(config_path,plan);write_json(folder/'frozen-plan.json',plan)
    sources={str(p):digest(p) for p in sorted(Path('research').glob('*.py'))}
    sources[config_path]=digest(config_path)
    write_json(folder/'freeze-receipt.json',{'plan_sha256':digest(folder/'frozen-plan.json'),'versioned_config':config_path,
        'versioned_config_sha256':digest(config_path),'executed_source_sha256':sources,
        'before_candidate_queries_and_rgb':True,'candidate_count':len(rows),'source_policy_sha256':digest(out/'analysis-policy.json')})
    print('FROZEN',digest(folder/'frozen-plan.json'),flush=True)


def finalize(out):
    plan=read(out/'stage-b/frozen-plan.json');rows=[]
    for c in plan['candidates']:
        p=read(out/'stage-b/physical'/f"{c['id']}.json")
        path=out/'stage-b/renders'/c['id']/'visibility.json';visibility=read(path) if path.exists() else []
        early=[r for r in visibility if early_clear(r)];passing=[(s,d) for s,d in p['sides'].items() if d['pass']]
        best=max(passing,key=lambda pair:pair[1]['minimum_structural_clearance']['clearance_m']) if passing else None
        rows.append({**c,'physical_pass':p['physical_pass'],'baseline_conflict':p['baseline_conflict'],
            'obstructed_primary_plus_six':p['obstructed_primary_plus_six'],'left_bypass_pass':p['sides']['LEFT']['pass'],
            'right_bypass_pass':p['sides']['RIGHT']['pass'],'early_clear_pass':bool(early),
            'first_visible':next((r for r in visibility if r['human_visible_pixel_count']>0),None),
            'first_clear':next((r for r in visibility if r['state']=='CLEAR'),None),
            'first_valid_early_clear':early[0] if early else None,
            'early_clear_distance_m':max((r['remaining_along_track_m'] for r in early),default=None),
            'best_side':best[0] if best else None,'best_structural_clearance_m':best[1]['minimum_structural_clearance']['clearance_m'] if best else None,
            'qualified':bool(p['physical_pass'] and p['baseline_conflict'] and passing and early),
            'reasons':[reason for good,reason in [(p['physical_pass'],'PHYSICAL_FAIL'),(p['baseline_conflict'],'NO_PRIMARY_CONFLICT'),(bool(passing),'NO_CONNECTED_BYPASS'),(bool(early),'NO_VALID_EARLY_CLEAR' if p['eligible_for_rgb'] else 'RGB_NOT_ELIGIBLE')] if not good]})
    selected=select(rows);decision='HUMAN_SITE_QUALIFIED_MODEL_FREE' if selected else 'NO_QUALIFIED_STATIONARY_HUMAN_SITE'
    write_json(out/'stage-b/qualification-matrix.json',rows)
    counts={key:sum(r[key] for r in rows) for key in ['physical_pass','baseline_conflict','left_bypass_pass','right_bypass_pass','early_clear_pass','qualified']}
    receipt={'decision':decision,'candidate_count':len(rows),'counts':counts,'selected_id':selected['id'] if selected else None,
        'plan_sha256':digest(out/'stage-b/frozen-plan.json'),'model_calls':0,'checkpoint_loads':0,'navigation_trials':0,'physics_steps':0}
    if selected:
        physical=read(out/'stage-b/physical'/f"{selected['id']}.json")
        write_json(out/'stage-b/selected-site.json',{'decision':decision,'candidate':selected,
            'human':{**plan['human'],'start_position':selected['position_world_xyz'],'yaw':selected['yaw_rad']},
            'connected_bypass':physical['sides'][selected['best_side']],
            'reference_config':str(Path(PRIMARY)/'runtime-config.json'),'reference_config_sha256':digest(Path(PRIMARY)/'runtime-config.json'),
            'note':'Future experiment input only; no navigation or controller change authorized by this result'})
    write_json(out/'stage-b/selection-receipt.json',receipt)
    final_plots(out,rows,plan)
    print('DECISION',receipt,flush=True)


def final_plots(out,rows,plan):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle
    from PIL import Image,ImageDraw
    folder=out/'stage-b/figures';folder.mkdir(exist_ok=False)
    ref=load_reference(PRIMARY);g=read(out/'stage-a/corridor_geometry_validated.json');collision=read(out/'stage-a/collision_geometry.json')
    fig,ax=plt.subplots(figsize=(14,6));base_axes(ax,g,collision);ax.plot(*ref['xy'].T,color='blue',label='measured original no-human path')
    selected=select(rows)
    for row in rows:
        x,y=row['position_world_xyz'][:2];color='green' if row['qualified'] else 'red'
        ax.add_patch(Circle((x,y),.25,fill=False,color=color));ax.text(x,y,row['id'],fontsize=7)
        if selected and row['id']==selected['id']:ax.scatter([x],[y],marker='*',s=150,color='gold',edgecolors='black')
    ax.set_xlim(-10.5,4);ax.set_ylim(7.8,13.5);ax.legend(fontsize=8)
    ax.set_title('MODEL-FREE HUMAN PLACEMENT PREFLIGHT — NO HUMAN-CONDITIONED NAVIGATION',fontsize=10)
    fig.tight_layout();fig.savefig(folder/'candidate-map.png',dpi=180);plt.close(fig)
    for row in rows:
        p=read(out/'stage-b/physical'/f"{row['id']}.json");fig,ax=plt.subplots(figsize=(11,5));base_axes(ax,g,collision)
        ax.plot(*ref['xy'].T,color='blue',label='measured no-human path');x,y=row['position_world_xyz'][:2]
        ax.add_patch(Circle((x,y),.25,facecolor='orange',alpha=.5,label='human proxy .25 m'))
        ax.add_patch(Circle((x,y),POLICY['required_center_separation_m'],fill=False,ls=':',label='required center separation'))
        for side,d in p['sides'].items():
            if d['vertices_xy']:ax.plot(*np.array(d['vertices_xy']).T,ls='--',color='green' if d['pass'] else 'red',label=f"geometric {side}: {'PASS' if d['pass'] else 'FAIL'}")
        ax.set_xlim(x-3.5,x+3.5);ax.set_ylim(8,13.1);ax.legend(fontsize=7)
        ax.set_title(row['id']+' static geometry only; dashed paths NEVER executed',fontsize=10)
        fig.tight_layout();fig.savefig(folder/(row['id']+'-bypass.png'),dpi=160);plt.close(fig)
        vis=out/'stage-b/renders'/row['id']/'visibility.json'
        if not vis.exists():continue
        records=read(vis);selected_frames=[]
        for r in [records[0],row['first_visible'],row['first_clear'],row['first_valid_early_clear'],records[-1]]:
            if r and r['request_id'] not in [v['request_id'] for v in selected_frames]:selected_frames.append(r)
        sheet=Image.new('RGB',(960,40+len(selected_frames)*300),'white');draw=ImageDraw.Draw(sheet)
        draw.text((8,8),row['id']+' MODEL-FREE: saved poses + stationary human; NO NAVIGATION',fill='black')
        for i,r in enumerate(selected_frames):
            rgb=Image.open(vis.parent/f"C{r['request_id']:02d}-rgb.png").resize((480,270))
            mask=Image.open(vis.parent/f"C{r['request_id']:02d}-mask.png").convert('RGB').resize((480,270))
            top=40+i*300;sheet.paste(rgb,(0,top));sheet.paste(mask,(480,top))
            draw.text((8,top+272),f"C{r['request_id']:02d} {r['state']} remaining={r['remaining_along_track_m']:.3f}m bright={r['brightness_pass']}",fill='black')
        sheet.save(folder/(row['id']+'-hawk-sheet.png'))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['geometry','analyze','freeze','finalize'],required=True)
    p.add_argument('--output-dir',required=True);p.add_argument('--config',default='configs/research/ticvla_2hz_cart_human_preflight.json')
    a=p.parse_args();out=Path(a.output_dir)
    if a.phase=='freeze':freeze(out,a.config)
    else:{'geometry':validate_geometry,'analyze':analyze,'finalize':finalize}[a.phase](out)


if __name__=='__main__': main()
