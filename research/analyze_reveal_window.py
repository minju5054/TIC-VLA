"""Saved-only tables/figures for the preregistered model-free search."""
import argparse
import csv
import json
import sys
from collections import Counter
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.reveal_window import classify, ranked, verify_hashes, freeze_selected
from research.analyze_nova_dynamic_handoff import load_saved
from research.records import write_json, provenance
from research.hospital_episode import digest

LABELS={'A':'Too early / no OLD conflict','B':'MARGINAL before CLEAR',
        'C':'Current robot already conflicts','D':'No local bypass','E':'Overlap / unsupported',
        'F':'Never CLEAR','G':'Conflict before saved switch','H':'Insufficient hidden history',
        'A_prefilter':'No conflict at any event; visibility unmeasured',
        'P_TEMPORAL':'No common temporal window; visibility unmeasured',
        'P_RAY':'No sampled ray release; visibility unmeasured',None:'Strict qualified'}
COLORS={'A':'#e5a220','B':'#a451bd','C':'#e64242','D':'#964d27','E':'#b6b6b6',
        'F':'#2266aa','G':'#f37740','H':'#8075a0','A_prefilter':'#e8dba7',
        'P_TEMPORAL':'#dfc486','P_RAY':'#8dacc0',None:'#119748'}


def table(path, candidates):
    keys=['id','x','y','z','yaw','rendered','strict_qualified','selection_rank','primary_failure','failures',
          'prefilter_pass','new_render_required','old_request_id','fresh_request_id','first_clear_pixel_count','first_clear_fraction','first_clear_bbox',
          'old_min_clearance_m','robot_current_clearance_at_reveal_m','reveal_sim_time','baseline_application_sim_time',
          'conflict_time_s','reveal_lead_s','switch_to_conflict_margin_s','bypass_clearance_m','occluding_prim_path']
    with path.open('x') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
        for c in candidates:
            row={k:c.get(k) for k in keys};row.update(zip(['x','y','z'],c['position']))
            first=c.get('first_clear') or {}
            row.update(first_clear_pixel_count=first.get('human_visible_pixel_count'),first_clear_fraction=first.get('human_visible_fraction'),
                       first_clear_bbox=first.get('human_bbox_if_visible'),conflict_time_s=(c.get('conflict') or {}).get('time'),
                       bypass_clearance_m=c['bypass'].get('clearance_m'),failures=';'.join(c['failures']))
            writer.writerow(row)


def figures(out, candidates, chosen, run, config, inventory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    from PIL import Image
    folder=out/'figures';folder.mkdir(exist_ok=False)
    plt.rcParams.update({'font.size':10,'savefig.dpi':170})

    def walls(ax, box=None):
        for p in inventory:
            if p['world_aabb_min'] is None or p['world_aabb_max'] is None:continue
            lo,hi=np.array(p['world_aabb_min']),np.array(p['world_aabb_max'])
            if not p['enabled'] or lo[2]>1.8 or hi[2]<1.5:continue
            if box and (np.any(hi[:2]<box[0]) or np.any(lo[:2]>box[1])):continue
            ax.add_patch(Rectangle(lo[:2],*(hi-lo)[:2],color='#929ba5',alpha=.35,linewidth=0))

    fig,ax=plt.subplots(figsize=(15,6));walls(ax)
    for key in [*COLORS]:
        subset=[c for c in candidates if c['primary_failure']==key]
        if subset:
            xy=np.array([c['position'][:2] for c in subset]);ax.scatter(*xy.T,s=12,color=COLORS[key],label=f'{LABELS[key]} ({len(subset)})')
    ax.plot(*run['xy'].T,color='#00b5c9',lw=2,label='Saved baseline executed path')
    if chosen:ax.scatter(*chosen['position'][:2],s=150,marker='*',color='magenta',edgecolors='black',label='Selected' if chosen['strict_qualified'] else 'Diagnostic near miss')
    lo,hi=run['xy'].min(0)-4.5,run['xy'].max(0)+4.5
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title='Hospital route-local search; exclusive primary rejection colors (all failures in JSON)')
    ax.set_aspect('equal');ax.legend(fontsize=7,loc='upper left',bbox_to_anchor=(1,1));fig.tight_layout();fig.savefig(folder/'search_overview.png');plt.close(fig)

    fig,ax=plt.subplots(figsize=(11,5))
    for key in COLORS:
        cs=[c for c in candidates if c['primary_failure']==key and c.get('reveal_lead_s') is not None]
        if cs:ax.scatter([c['fresh_request_id'] for c in cs],[c['reveal_lead_s'] for c in cs],s=24,label=LABELS[key],color=COLORS[key],alpha=.7)
    ax.axhline(0,color='black',lw=.8);ax.set(xlabel='First CLEAR request ID',ylabel='Reveal → nominal OLD conflict [s]',
        title='Only candidates with an OLD remaining conflict have a defined lead time')
    handles,_=ax.get_legend_handles_labels()
    if handles:ax.legend(fontsize=8)
    else:ax.text(.5,.5,'No defined reveal lead times in the rendered cohort.\nNo OLD remaining conflict at first CLEAR, or never CLEAR.',
                 ha='center',va='center',transform=ax.transAxes)
    fig.tight_layout();fig.savefig(folder/'reveal_lead_time.png');plt.close(fig)
    if (out/'prefilter_summary.json').exists():
        ps=json.loads((out/'prefilter_summary.json').read_text())
        labels=['All candidates',*ps['funnel'].keys()];values=[ps['candidate_count_before'],*ps['funnel'].values()]
        fig,ax=plt.subplots(figsize=(11,5));ax.barh(labels[::-1],values[::-1],color='#527db0')
        for i,value in enumerate(values[::-1]):ax.text(value+12,i,str(value),va='center')
        ax.set(xlabel='Candidates retained by cumulative cheap gates',xlim=(0,max(values)*1.14),
               title='Render-free prefilter — sample-ray possibility is not semantic visibility')
        fig.tight_layout();fig.savefig(folder/'prefilter_funnel.png');plt.close(fig)
    if not chosen:return
    c=chosen;label='SELECTED STRICT' if c['strict_qualified'] else 'DIAGNOSTIC NEAR MISS — NOT SELECTED'
    rid=c['fresh_request_id'];xy=np.array(c['position'][:2]);points=np.array(c.get('remaining_world_xy',[]))
    bypass=np.array(c['bypass']['centerline_xy'])
    allp=np.vstack([xy,points]) if len(points) else xy[None]
    if len(bypass):allp=np.vstack([allp,bypass])
    lo,hi=allp.min(0)-1.5,allp.max(0)+1.5
    fig,ax=plt.subplots(figsize=(10,7));walls(ax,(lo,hi));ax.plot(*run['xy'].T,color='#00b5c9',lw=1,label='Saved path')
    if len(points):ax.plot(*points.T,'o-',ms=3,color='#f18c10',label='OLD remaining nominal future')
    for request,name,color in [(c['old_request_id'],'OLD','orange'),(rid,'First CLEAR','blue')]:
        if request:
            e=run['events'][request-1]['observation'];p=e['position'];theta=e['pose'][2]
            ax.scatter(*p[:2],marker='s',color=color,label=f'C{request} {name} observation')
            ax.arrow(*p[:2],.5*np.cos(theta),.5*np.sin(theta),width=.025,color=color)
    ax.add_patch(Circle(xy,config['human_radius_m'],color='magenta',alpha=.7));ax.scatter(*xy,color='magenta',label='Fixed human')
    ax.add_patch(Circle(xy,config['human_radius_m']+config['robot_radius_m'],fill=False,color='red',linestyle='--',label='Combined conflict proxy'))
    if c.get('conflict'):ax.scatter(*c['conflict']['point'],marker='x',s=110,color='red',label='Earliest nominal conflict')
    if len(bypass):ax.plot(*bypass.T,color='green',lw=3,label='Local bypass strip center')
    ax.set(xlim=(lo[0],hi[0]),ylim=(lo[1],hi[1]),xlabel='World X [m]',ylabel='World Y [m]',title=f'{label}\nCandidate {c["id"]} | failures {c["failures"]}')
    ax.set_aspect('equal');ax.legend(fontsize=8);fig.tight_layout();fig.savefig(folder/'candidate_closeup.png');plt.close(fig)

    rows=c['visibility'];ids=[r['request_id'] for r in rows]
    ray_path=out/'multi_ray_prefilter.json'
    if ray_path.exists():
        raydata=json.loads(ray_path.read_text()).get(str(c['id']))
        if raydata:
            pairdata=json.loads((out/'temporal_pair_prefilter.json').read_text())[str(c['id'])]
            fig,ax=plt.subplots(figsize=(12,4))
            ax.step(ids,[sum(r['exposed']) for r in raydata['observations']],where='mid',label='Exposed ray samples (not pixels)')
            tids=[r['fresh_request_id'] for r in pairdata if r['temporal_pass']]
            ax.scatter(tids,[-.5]*len(tids),marker='|',s=180,color='red',label='Possible temporal conflict windows')
            ax.axvline(rid,color='magenta',ls='--',label=f'Actual first semantic CLEAR C{rid}')
            ax.set(xlabel='Saved request ID',ylabel='Sample count',ylim=(-1,18),title=f'{label} — candidate {c["id"]}: ray and semantic evidence are different')
            ax.legend(fontsize=8);fig.tight_layout();fig.savefig(folder/'ray_temporal_diagnostic.png');plt.close(fig)
    fig,axes=plt.subplots(3,1,figsize=(12,8),sharex=True)
    axes[0].step(ids,[{'HIDDEN':0,'MARGINAL':1,'CLEAR':2}[r['state']] for r in rows],where='mid');axes[0].set_yticks([0,1,2],['HIDDEN','MARGINAL','CLEAR'])
    axes[1].plot(ids,[r['human_visible_fraction'] for r in rows]);axes[1].axhline(.002,color='red',ls='--',label='CLEAR fraction minimum');axes[1].set_ylabel('Semantic fraction');axes[1].legend()
    axes[2].plot(ids,[(r['human_bbox_if_visible'][3]-r['human_bbox_if_visible'][1]) if r['human_bbox_if_visible'] else 0 for r in rows],label='bbox height')
    axes[2].plot(ids,[(r['human_bbox_if_visible'][2]-r['human_bbox_if_visible'][0]) if r['human_bbox_if_visible'] else 0 for r in rows],label='bbox width');axes[2].set(xlabel='Saved request ID',ylabel='Pixels');axes[2].legend()
    for ax in axes:ax.axvline(rid,color='magenta',ls=':');ax.grid(alpha=.25)
    fig.suptitle(f'{label} — candidate {c["id"]}, first CLEAR C{rid}');fig.tight_layout();fig.savefig(folder/'visibility_timeline.png');plt.close(fig)

    fig,ax=plt.subplots(figsize=(12,3));timeline=[('Reveal',c['reveal_sim_time']),('Saved baseline application',c['baseline_application_sim_time']),('OLD horizon end',c['old_horizon_end_s'])]
    if c.get('conflict'):timeline.append(('Nominal OLD conflict',c['conflict']['time']))
    for i,(name,t) in enumerate(timeline):
        if t is not None:ax.scatter(t,i,s=70);ax.text(t,i+.15,f'{name}: {t:.6f}s',ha='center',fontsize=9)
    ax.set(ylim=(-.5,4),xlabel='Baseline simulation / derived nominal absolute target time [s]',yticks=[],title=label+' — no new inference timing prediction')
    ax.grid(axis='x',alpha=.3);fig.tight_layout();fig.savefig(folder/'conflict_timeline.png');plt.close(fig)

    selected_rows=[r for r in rows if max(1,rid-3)<=r['request_id']<=min(48,rid+1)]
    fig,axes=plt.subplots(2,len(selected_rows),figsize=(4*len(selected_rows),5),squeeze=False)
    for j,row in enumerate(selected_rows):
        image=np.asarray(Image.open(row['rgb']));mask=np.asarray(Image.open(row['mask']))>0;overlay=image.copy()
        overlay[mask]=(image[mask]*.45+np.array([255,0,200])*.55).astype(np.uint8)
        axes[0,j].imshow(image);axes[1,j].imshow(overlay)
        name='OLD observation' if row['request_id']==rid-1 else ('FIRST CLEAR FRESH observation' if row['request_id']==rid else '')
        axes[0,j].set_title(f'C{row["request_id"]} {row["state"]}\n{name}',fontsize=9)
        axes[1,j].set_title(f'{row["human_visible_pixel_count"]} px | {row["human_visible_fraction"]:.6f}',fontsize=9)
        for ax in axes[:,j]:ax.axis('off')
        overlay_dir=out/'derived_overlays';overlay_dir.mkdir(exist_ok=True)
        Image.fromarray(overlay).save(overlay_dir/f"candidate_{c['id']:04d}_C{row['request_id']:02d}.png")
    fig.suptitle(label+' | actual saved Hawk RGB (top), semantic overlay (bottom)')
    fig.tight_layout();fig.savefig(folder/'candidate_image_sheet.png');plt.close(fig)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--search-dir',required=True)
    p.add_argument('--freeze-reviewed',help='JSON visual review receipt for the deterministic strict proposal only')
    a=p.parse_args();out=Path(a.search_dir)
    if a.freeze_reviewed:
        review=json.loads(Path(a.freeze_reviewed).read_text())
        candidates=json.loads((out/'candidates.json').read_text());order=ranked(candidates)
        if not order:raise ValueError('No strict candidate to freeze')
        candidate=order[0];rid=candidate['fresh_request_id']
        required=set(range(max(1,rid-3),min(48,rid+1)+1))
        if (review.get('candidate_id')!=candidate['id'] or not review.get('hidden_visual_review_pass')
                or not review.get('clear_visual_review_pass') or not required.issubset(review.get('reviewed_request_ids',[]))):
            raise ValueError('Visual review must cover the deterministic candidate and its reveal window')
        verify_hashes(json.loads((out/'source_evidence_sha256.json').read_text()))
        freeze_selected(out/'selected_candidate.json',candidates)
        summary=json.loads((out/'review_pending_summary.json').read_text())
        summary.update(decision='STRICT REVEAL-WINDOW CANDIDATE IDENTIFIED AND FROZEN',selection_visual_review_pending=False,
                       visual_review_sha256=digest(a.freeze_reviewed),visual_review_path=str(Path(a.freeze_reviewed).resolve()))
        write_json(out/'summary.json',summary)
        print(json.dumps(summary,indent=2));return
    cfg=json.loads((out/'preregistered.json').read_text())['config']
    run=load_saved(cfg['baseline_run'])
    hashes=json.loads((out/'source_evidence_sha256.json').read_text());verify_hashes(hashes)
    raw=json.loads((out/'geometry_candidates.json').read_text());candidates=[]
    manifest={}
    for path in out.glob('render_batch_*/complete.json'):
        part=json.loads(path.read_text())['candidate_observations']
        if set(manifest)&set(part):raise ValueError('Overlapping completed render shards')
        manifest.update(part)
    eligible={str(c['id']) for c in raw if c['render_candidate']}
    if set(manifest)!=eligible:raise ValueError('Missing completed rendered candidates')
    complete={'rendered_candidates':len(eligible),'observation_count':48,'images':48*len(eligible),
              'candidate_observations':manifest,'model_calls':0,'navigation_physics_reexecuted':False}
    write_json(out/'render_complete.json',complete)
    for c in raw:
        rows=json.loads(Path(manifest[str(c['id'])]).read_text()) if c['render_candidate'] else None
        value=classify(c,rows,run,cfg)
        old=value.get('old_request_id')
        ray=c['center_ray_occluders'][old-1] if old else None
        value['occluding_prim_path']=ray['prim'] if ray else None
        value['occluder_note']='OLD center ray attribution only; silhouette visibility is measured separately. Null does not mean no occluder.'
        candidates.append(value)
    order=ranked(candidates);rank_by_id={c['id']:c['selection_rank'] for c in order}
    for c in candidates:c['selection_rank']=rank_by_id.get(c['id'])
    selected=order[0] if order else None
    if selected:
        selected['selection_rationale']=cfg['ranking']
        write_json(out/'proposed_selected_candidate.json',selected)
    else:
        write_json(out/'selected_candidate.json',None)
    reviewable=[c for c in candidates if c.get('first_clear') and c.get('old_request_id')]
    # Diagnostic ordering has no bearing on strict qualification or selection.
    diagnostic=selected or (min(reviewable,key=lambda c:(c.get('conflict') is None,c['id'])) if reviewable else None)
    counts=Counter(f for c in candidates for f in c['failures'])
    exclusive=Counter(c['primary_failure'] or 'STRICT' for c in candidates)
    summary={'candidate_count':len(candidates),'rendered_count':complete['rendered_candidates'],
             'rendered_observations':complete['images'],'strict_qualified_count':len(order),
             'failure_counts_multilabel':dict(counts),'exclusive_primary_counts':dict(exclusive),
             'selected_candidate_id':selected['id'] if selected else None,
             'diagnostic_candidate_id':diagnostic['id'] if diagnostic else None,
             'decision':'STRICT REVEAL-WINDOW CANDIDATE IDENTIFIED AND FROZEN' if selected else 'NO STRICT REVEAL-WINDOW CANDIDATE FOUND',
             'selection_visual_review_pending':bool(selected),'model_calls':0,'navigation_physics_reexecuted':False,
             'human_model_run':'NOT EXECUTED — MODEL-FREE SEARCH TASK ONLY',
             'source_unchanged':True,'nominal_timing':'Source-confirmed +0.1..+3.0s target convention, not native timestamp channel',
             'strict_lead_times_s':[c['reveal_lead_s'] for c in order],
             'strict_switch_margins_s':[c['switch_to_conflict_margin_s'] for c in order],
             'prior_region_count':sum(c['previous_region'] for c in candidates),
             'prior_region_rendered_count':sum(c['previous_region'] and c['rendered'] for c in candidates)}
    if (out/'prefilter_summary.json').exists():
        summary['cheap_prefilter']=json.loads((out/'prefilter_summary.json').read_text())
        rules=json.loads((out/'preregistered.json').read_text())['prefilter']
        for reason in rules['rejection_labels']:summary['cheap_prefilter']['rejection_counts'].setdefault(reason,0)
        pairs=json.loads((out/'temporal_pair_prefilter.json').read_text())
        all_pairs=[p for rows in pairs.values() for p in rows]
        summary['cheap_prefilter']['pair_counts']={key:sum(bool(p[key]) for p in all_pairs)
            for key in ['conflict_exists','positive_lead','current_clear','switch_before_conflict','temporal_pass']}
        summary['cheap_prefilter']['pair_count_total']=len(all_pairs)
        summary['cheap_prefilter']['prior_region']={'candidate_count':sum(c['previous_region'] for c in candidates),
            'temporal_valid_candidates':sum(c['previous_region'] and bool(c['prefilter']['temporal_pass_pair_count']) and c['prefilter']['physical_pass'] and c['prefilter']['bypass_pass'] for c in candidates),
            'ray_survivors':sum(c['previous_region'] and c['prefilter_pass'] for c in candidates)}
        write_json(out/'prefilter_rejection_counts.json',summary['cheap_prefilter'])
        with (out/'prefilter_rejection_counts.csv').open('x') as f:
            writer=csv.writer(f);writer.writerow(['reason','candidate_count','counting'])
            for reason,count in summary['cheap_prefilter']['rejection_counts'].items():writer.writerow([reason,count,'multi-label / conditional temporal reasons'])
        summary['new_rendered_count']=sum(c.get('new_render_required',False) for c in candidates)
        summary['reused_rendered_count']=summary['rendered_count']-summary['new_rendered_count']
    # A strict proposal cannot be described as frozen until visual review receipt.
    if selected:summary['decision']='STRICT CANDIDATE PROPOSED — VISUAL REVIEW PENDING'
    write_json(out/'candidates.json',candidates);table(out/'candidates.csv',candidates);table(out/'qualified_strict.csv',order)
    write_json(out/'near_misses.json',{'counting':cfg['failure_counting'],'labels':{k:v for k,v in LABELS.items() if k},
               'multilabel_counts':dict(counts),'exclusive_counts':dict(exclusive),
               'candidates':[{'id':c['id'],'failures':c['failures'],'rendered':c['rendered']} for c in candidates if not c['strict_qualified']]})
    write_json(out/('review_pending_summary.json' if selected else 'summary.json'),summary)
    write_json(out/'analysis_provenance.json',{'script_sha256':digest(__file__),**provenance(ROOT)})
    figures(out,candidates,diagnostic,run,cfg,json.loads(Path(cfg['collision_inventory']).read_text()))
    verify_hashes(hashes)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
