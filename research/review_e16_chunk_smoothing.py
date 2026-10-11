"""Independent saved-result verification, descriptive comparisons and figure layout.

No fitting, optimizer, model or simulator is called. Original outputs are read-only.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import numpy as np
from scipy.interpolate import BSpline

ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.records import write_json
from research.reveal_window import evidence_hashes,verify_hashes


def independent_validate(source):
    meta=json.loads((source/'metadata.json').read_text());cfg=meta['config']
    for key in ['source_and_prior_output_sha256','analysis_source_sha256','previous_code_sha256']:verify_hashes(meta[key])
    chunks={p.stem:json.loads(p.read_text()) for p in (source/'chunks').glob('*.json')}
    pairs={p.stem:json.loads(p.read_text()) for p in (source/'pairs').glob('*.json')}
    rows=json.loads((source/'chunk_metrics.json').read_text());pr=json.loads((source/'pair_metrics.json').read_text())
    assert len(chunks)==96 and len(pairs)==94 and len(rows)==384 and len(pr)==470
    def evaluate(c,method,q,der=0,bridge=False):
        q=np.atleast_1d(np.array(q,float));t=.1*np.arange(1,31);native=np.array(c['native_local_xy'])
        assert q.min()>=(-1e-8 if bridge else .1-1e-8) and q.max()<=3+1e-8
        qs=np.clip(q,.1,3)
        if method=='RAW':
            if der==0:out=np.column_stack([np.interp(qs,t,native[:,j]) for j in [0,1]])
            else:
                idx=np.minimum(np.searchsorted(t,qs,side='right')-1,28)
                out=np.diff(native,axis=0)[idx]/np.diff(t)[idx,None]
        else:
            info=c['fits'][method]['fit']
            if method.startswith('SPLINE'):
                out=np.column_stack([BSpline(np.r_[[.1]*3,k,[3.]*3],co,3,extrapolate=False)(qs,nu=der) for k,co in zip(info['knots'],info['coefficients'])])
            else:
                out=np.column_stack([np.polynomial.polynomial.polyval(qs,np.polynomial.polynomial.polyder(co,m=der)) for co in info['power_coefficients']])
        before=q<.1
        if before.any():
            first=evaluate(c,method,[.1])[0]
            out[before]=q[before,None]/.1*first if der==0 else first/.1
        return out
    fit_count=0
    for key,c in chunks.items():
        native=np.load(c['native_file'],allow_pickle=False);np.testing.assert_array_equal(native,c['native_local_xy']);assert native.shape==(30,2) and np.isfinite(native).all()
        for method,d in c['fits'].items():
            q=np.array(d['nominal_times']);np.testing.assert_array_equal(q,.1*np.arange(1,31))
            point=evaluate(c,method,q);der=evaluate(c,method,q,1);dd=evaluate(c,method,q,2)
            np.testing.assert_allclose(point,d['fitted_local_xy'],atol=1e-12)
            np.testing.assert_allclose(der,d['derivative_local'],atol=1e-11)
            np.testing.assert_allclose(dd,d['second_derivative_local'],atol=1e-10)
            r=native-point;speed=np.linalg.norm(der,axis=1);valid=speed>=.1
            tangent=der[valid]/speed[valid,None];normal=np.column_stack([-tangent[:,1],tangent[:,0]])
            f=(r[valid]*tangent).sum(1);l=(r[valid]*normal).sum(1)
            m=next(r for r in rows if r['run']==c['run'] and r['request_id']==c['request_id'] and r['method']==method)
            for label,v in [('forward',f),('lateral',l)]:
                rmse=np.sqrt(np.mean(v*v));mad=1.4826*np.median(np.abs(v-np.median(v)))
                np.testing.assert_allclose([rmse,mad],[m[label+'_rmse_m'],m[label+'_mad_m']],atol=1e-11)
            np.testing.assert_allclose(np.sqrt(np.mean(np.sum(r*r,axis=1))),m['xy_rmse_m'],atol=1e-12)
            assert int(valid.sum())==m['reliable_tangent_samples']
            if method.startswith('SPLINE'):assert np.all(np.sum(r*r,axis=0)<=d['fit']['s_per_coordinate_m2']*1.00101)
            fit_count+=1
    compared=0
    def rot(yaw):return np.array([[np.cos(yaw),-np.sin(yaw)],[np.sin(yaw),np.cos(yaw)]])
    def world(c,method,q,der=0,bridge=False):
        pose=c['observation_pose'];p=evaluate(c,method,q,der,bridge)@rot(pose[2]).T
        return p+pose[:2] if der==0 else p
    for key,p in pairs.items():
        old=chunks[f"{p['run']}_C{p['old_request_id']:02d}"];fresh=chunks[f"{p['run']}_C{p['fresh_request_id']:02d}"]
        oe=json.loads(Path(old['native_file']).with_suffix('.json').read_text());fe=json.loads(Path(fresh['native_file']).with_suffix('.json').read_text())
        assert p['boundary']==fe['agent_pose_at_application'];assert fresh['observation_pose']==fe['agent_pose_at_observation'];assert old['observation_pose']==oe['agent_pose_at_observation']
        elapsed=p['switch_sim_time']-p['fresh_observation_sim_time'];b=np.array(p['boundary'])
        for method,d in p['methods'].items():
            point=world(fresh,method,[elapsed],bridge=True)[0];der=world(fresh,method,[elapsed],der=1,bridge=True)[0]
            np.testing.assert_allclose(point,d['boundary']['boundary_world_xy'],atol=1e-12)
            np.testing.assert_allclose(der,d['boundary']['boundary_derivative_world'],atol=1e-11)
            r=next(r for r in pr if r['pair_id']==key and r['method']==method)
            np.testing.assert_allclose(np.linalg.norm(point-b[:2]),r['boundary_position_m'],atol=1e-12)
            if np.linalg.norm(der)>=.1:
                phi=np.arctan2(der[1],der[0])-b[2];error=abs(np.degrees(np.arctan2(np.sin(phi),np.cos(phi))))
                np.testing.assert_allclose(error,r['boundary_robot_heading_deg'],atol=1e-10)
            else:assert r['boundary_robot_heading_deg'] is None
            for scope,v in d['overlaps'].items():
                q=np.array(v['absolute_times']);assert np.array_equal(q,p['methods']['RAW']['overlaps'][scope]['absolute_times'])
                if scope=='editable':assert (q>p['switch_sim_time']+1e-8).all()
                ow=world(old,method,q-p['old_observation_sim_time']);fw=world(fresh,method,q-p['fresh_observation_sim_time'])
                np.testing.assert_allclose(ow,v['old_world_xy'],atol=1e-12);np.testing.assert_allclose(fw,v['fresh_world_xy'],atol=1e-12)
                delta=fw-ow;basis=np.array(v['reference_tangent_world'],float);normal=np.column_stack([-basis[:,1],basis[:,0]])
                good=np.isfinite(basis).all(1);l=(delta[good]*normal[good]).sum(1)
                np.testing.assert_allclose(np.sqrt(np.mean(np.sum(delta*delta,axis=1))),r[scope+'_rmse_m'],atol=1e-12)
                np.testing.assert_allclose(np.mean(np.abs(l)),r[scope+'_mean_abs_lateral_m'],atol=1e-12)
            compared+=1
    for root in cfg['source_runs'].values():
        root=(ROOT/root).resolve();expected={p:h for p,h in meta['source_and_prior_output_sha256'].items() if root in Path(p).parents}
        assert evidence_hashes([root])==expected
    return dict(status='PASS',native_chunks_checked=96,fitted_chunk_outputs_independently_checked=fit_count,
        pair_accounts=94,pair_method_outputs_independently_checked=compared,original_run_files_unchanged=700,
        protected_source_and_prior_output_hashes=len(meta['source_and_prior_output_sha256']),
        existing_code_hashes=len(meta['previous_code_sha256']),no_graph_calls=True,no_model_calls=True,no_simulator=True)


def describe(source):
    rows=json.loads((source/'pair_metrics.json').read_text());chunks=json.loads((source/'chunk_metrics.json').read_text());sens=json.loads((source/'smoothing_sensitivity.json').read_text())
    ds={p.stem:json.loads(p.read_text()) for p in (source/'pairs').glob('*.json')};by={(r['pair_id'],r['method']):r for r in rows}
    stats=lambda a:dict(count=len(a),median=float(np.median(a)),p95=float(np.percentile(a,95)),min=float(min(a)),max=float(max(a))) if a else dict(count=0)
    output={'retention':{},'same_60_previous_graph_cases':{},'same_91_nonbootstrap_heading_cases':{},'derivative_coverage':{},'graph_short_cases':[]}
    common=[k for k,d in ds.items() if not d['bootstrap'] and d['original_graph_metrics'].get('GRAPH',{}).get('executed_to_future_tangent_deg') is not None]
    assert len(common)==60
    heading_common=[k for k,d in ds.items() if not d['bootstrap'] and all(by[(k,m)]['boundary_robot_heading_deg'] is not None for m in ['RAW','SPLINE_01cm','SPLINE_03cm','SPLINE_05cm'])]
    for method in ['RAW','SPLINE_01cm','SPLINE_03cm','SPLINE_05cm','POLY3_DIAGNOSTIC']:
        selected=[r for r in rows if r['method']==method]
        output['same_60_previous_graph_cases'][method]=stats([by[(k,method)]['sampled_first_executed_heading_deg'] for k in common if by[(k,method)]['sampled_first_executed_heading_deg'] is not None])
        output['same_91_nonbootstrap_heading_cases'][method]=stats([by[(k,method)]['boundary_robot_heading_deg'] for k in heading_common])
        output['retention'][method]=stats([r['full_rmse_m']/by[(r['pair_id'],'RAW')]['full_rmse_m'] for r in selected])
        excluded=[r for r in selected if r['existing_graph_eligibility']=='INELIGIBLE']
        output['derivative_coverage'][method]=dict(all_reference_nodes_reliable=sum(r['reliable_reference_derivatives']==r['reference_node_count'] for r in selected),
            previous_excluded_with_all_reliable=sum(r['reliable_reference_derivatives']==r['reference_node_count'] for r in excluded),
            boundary_invalid=[r['pair_id'] for r in selected if not r['boundary_derivative_valid']],
            dense_invalid_derivative_samples=sum(r['dense_invalid_derivatives'] for r in chunks if r['method']==method))
    for method in ['RIGID','GRAPH','GRAPH_NO_CORR']:
        output['same_60_previous_graph_cases']['SAVED_'+method]=stats([ds[k]['original_graph_metrics'][method]['executed_to_future_tangent_deg'] for k in common])
    for key,d in ds.items():
        g=d['original_graph_metrics'].get('GRAPH')
        if g and not g['first_segment_valid']:
            output['graph_short_cases'].append(dict(pair_id=key,bootstrap=d['bootstrap'],raw_segment_m=by[(key,'RAW')]['first_sampled_segment_m'],
                spline03_segment_m=by[(key,'SPLINE_03cm')]['first_sampled_segment_m'],saved_graph_segment_m=g['first_segment_length_m']))
    output['same_91_nonbootstrap_heading_cases']['RAW_CHORD']=stats([by[(k,'RAW')]['raw_graph_chord_robot_heading_deg'] for k in heading_common])
    output['boundary_heading_strength_spread_deg']=stats([s['boundary_heading_spread_deg'] for s in sens if s['boundary_heading_spread_deg'] is not None])
    output['prior_excluded_all_three_spline_reference_derivatives_reliable']=sum(s['all_spline_reference_nodes_reliable'] for s in sens if s['existing_graph_eligibility']=='INELIGIBLE')
    output['same_60_pair_ids']=common;output['same_91_pair_ids']=heading_common
    return output


def compare_future_chords(source):
    """All three estimators at the same overlap points; existing frozen chord rule.

    This supplements boundary chord comparisons without refitting either chunk.
    """
    graph=json.loads((ROOT/'configs/research/e16_se2_graph_offline.json').read_text())['tangent']
    results=[]
    def chord(c,queries):
        t=c['observation_sim_time']+.1*np.arange(1,31);p=np.array(c['native_world_xy']);heads=[]
        for q in queries:
            lo,hi=max(t[0],q-graph['half_window_s']),min(t[-1],q+graph['half_window_s'])
            knots=np.unique(np.r_[lo,t[(t>lo)&(t<hi)],hi]);xy=np.column_stack([np.interp(knots,t,p[:,j]) for j in [0,1]])
            delta=xy[-1]-xy[0];length=np.linalg.norm(delta);arc=np.linalg.norm(np.diff(xy,axis=0),axis=1).sum()
            valid=length>=graph['minimum_chord_m'] and arc>0 and length/arc>=graph['minimum_coherence']
            heads.append(float(np.arctan2(delta[1],delta[0])) if valid else np.nan)
        return np.array(heads)
    for path in sorted((source/'pairs').glob('*.json')):
        d=json.loads(path.read_text());old=json.loads((source/'chunks'/f"{d['run']}_C{d['old_request_id']:02d}.json").read_text());fresh=json.loads((source/'chunks'/f"{d['run']}_C{d['fresh_request_id']:02d}.json").read_text())
        for scope,overlap in d['methods']['RAW']['overlaps'].items():
            q=np.array(overlap['absolute_times']);a=chord(old,q);b=chord(fresh,q);delta=b-a
            deg=np.abs(np.degrees(np.arctan2(np.sin(delta),np.cos(delta))));valid=np.isfinite(deg)
            results.append(dict(pair_id=path.stem,run=d['run'],bootstrap=d['bootstrap'],scope=scope,
                point_count=len(q),valid_count=int(valid.sum()),mean_abs_tangent_deg=float(deg[valid].mean()) if valid.any() else None,
                max_abs_tangent_deg=float(deg[valid].max()) if valid.any() else None,
                absolute_times=q.tolist(),old_chord_heading_rad=[float(x) if np.isfinite(x) else None for x in a],
                fresh_chord_heading_rad=[float(x) if np.isfinite(x) else None for x in b]))
    return results


def improve_layout(source,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from research.plot_e16_chunk_smoothing import COLORS
    cfg=json.loads((source/'metadata.json').read_text())['config'];rows=json.loads((source/'pair_metrics.json').read_text())
    names=list(COLORS);replacement=['native_and_fitted_chunks','boundary_heading_estimators','boundary_position']+[f'case_{r}_C{i:02d}_C{i+1:02d}' for r,i in cfg['selected_pairs']]
    dest=out/'figures';dest.mkdir()
    for p in (source/'figures').iterdir():
        if p.stem not in replacement:shutil.copy2(p,dest/p.name)
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'pdf.fonttype':42})
    def save(fig,name):
        fig.savefig(dest/(name+'.png'),dpi=170,bbox_inches='tight');fig.savefig(dest/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
    getchunk=lambda r,i:json.loads((source/'chunks'/f'{r}_C{i:02d}.json').read_text())
    fig,axes=plt.subplots(2,2,figsize=(13,9));fig.subplots_adjust(top=.91,bottom=.13,hspace=.32,wspace=.24)
    for ax,(run,rid) in zip(axes.flat,cfg['representative_chunks']):
        c=getchunk(run,rid);p=np.array(c['native_local_xy']);ax.plot(*p.T,'o:',c=COLORS['RAW'],ms=3,label='Native prediction')
        allp=[p,np.zeros((1,2))]
        for m in names[1:]:
            q=np.array(c['fits'][m]['grid_local_xy']);allp.append(q);ax.plot(*q.T,c=COLORS[m],ls='--' if m.startswith('POLY') else '-',label=m)
        ax.scatter(0,0,facecolors='none',edgecolors='black',s=45,label='Observation origin')
        span=np.ptp(np.vstack(allp),axis=0);center=(np.vstack(allp).max(0)+np.vstack(allp).min(0))/2;span=np.maximum(span,[.5,max(span[0]*.5,.5)])*1.1
        ax.set(xlim=[center[0]-span[0]/2,center[0]+span[0]/2],ylim=[center[1]-span[1]/2,center[1]+span[1]/2],xlabel='Observation-local forward [m]',ylabel='Observation-local left [m]',title=f'{run} C{rid}: {c["phase"]}')
        ax.set_aspect('equal')
    fig.legend(*axes[0,0].get_legend_handles_labels(),loc='lower center',ncol=3,fontsize=9);fig.suptitle('Native points and independent fits | equal XY aspect; observation origin is not a fitting datum')
    save(fig,'native_and_fitted_chunks')
    normal=[r for r in rows if not r['bootstrap']]
    fig,axes=plt.subplots(2,2,figsize=(14,9));fig.subplots_adjust(top=.91,bottom=.08,hspace=.3,wspace=.22)
    for col,(key,chord) in enumerate([('boundary_robot_heading_deg','raw_graph_chord_robot_heading_deg'),('boundary_executed_heading_deg','raw_graph_chord_executed_heading_deg')]):
        arrays=[[r[key] for r in normal if r['method']==m and r.get(key) is not None] for m in names]
        arrays.insert(1,[r[chord] for r in normal if r['method']=='RAW' and r.get(chord) is not None]);labels=['Adjacent','Chord','1cm','3cm','5cm','Poly3']
        for row in [0,1]:
            ax=axes[row,col];ax.boxplot(arrays,labels=[f'{m}\nn={len(v)}' for m,v in zip(labels,arrays)]);ax.set_ylabel('Heading seam [deg]')
            ax.set_title(('Measured yaw' if col==0 else 'Valid executed direction')+(' - full range' if row==0 else ' - 0 to 10 deg detail'))
            if row==1:ax.set_ylim(-.3,10)
    fig.suptitle('92 nonbootstrap pairs: raw adjacent / existing chord / independently fitted derivative\nBottom panels zoom the same data; all large outliers remain visible above')
    save(fig,'boundary_heading_estimators')
    raw={r['pair_id']:r for r in rows if r['method']=='RAW'}
    fig,axes=plt.subplots(1,3,figsize=(15,5));fig.subplots_adjust(top=.83,bottom=.13,wspace=.3)
    pool=[r for r in rows if r['method'] in names[1:4]];lim=max(r['boundary_position_m'] for r in pool)*1.05
    for ax,m in zip(axes,names[1:4]):
        rs=[r for r in pool if r['method']==m];ax.scatter([raw[r['pair_id']]['boundary_position_m'] for r in rs],[r['boundary_position_m'] for r in rs],s=22,alpha=.7,c=['#bd533f' if r['phase']=='left_turn' else '#176b93' for r in rs])
        ax.plot([0,lim],[0,lim],'--',c='gray');ax.set(xlim=(0,lim),ylim=(0,lim),xlabel='RAW seam [m]',ylabel='Fitted seam [m]',title=m+' | n=94');ax.set_aspect('equal')
    fig.suptitle('Same measured switch B; predictions are never reanchored\nEqual scales across policies; red points mark the saved left-turn phase')
    save(fig,'boundary_position')
    for run,i in cfg['selected_pairs']:
        key=f'{run}_C{i:02d}_C{i+1:02d}';d=json.loads((source/'pairs'/(key+'.json')).read_text());b=np.array(d['boundary'])
        fig,axes=plt.subplots(1,2,figsize=(14,7));fig.subplots_adjust(top=.83,bottom=.24,wspace=.25);points=[]
        for ax in axes:
            ax.plot(*np.array(d['measured_committed_xy']).T,c='#278343',lw=3,label='Measured committed execution')
            for rid,old in [(i,True),(i+1,False)]:
                c=getchunk(run,rid);p=np.array(c['native_world_xy']);q=np.array(c['fits']['SPLINE_03cm']['grid_world_xy']);points.extend([p,q,np.array(c['observation_pose'][:2])[None]])
                ax.plot(*p.T,'.:',c='#999999' if old else '#333333',ms=3,label=('OLD' if old else 'FRESH')+' native prediction')
                ax.plot(*q.T,c='#9864a8' if old else COLORS['SPLINE_03cm'],lw=2,label=('OLD' if old else 'FRESH')+' spline03')
                ax.scatter(*c['observation_pose'][:2],facecolors='none',edgecolors='#9864a8' if old else 'black',s=45,label=('OLD' if old else 'FRESH')+' observation')
            ax.scatter(*b[:2],marker='*',c='#278343',s=100,label='Measured switch B');ax.set_aspect('equal');ax.set(xlabel='World X [m]',ylabel='World Y [m]')
        p=np.vstack(points);span=np.ptp(p,axis=0);span=np.maximum(span,np.max(span)*.55)+.1;center=(p.max(0)+p.min(0))/2
        axes[0].set(xlim=[center[0]-span[0]/2,center[0]+span[0]/2],ylim=[center[1]-span[1]/2,center[1]+span[1]/2],title='Full native/fitted prediction horizons')
        centers=[b[:2],np.array(d['fresh_observation_pose'][:2])]
        for m in names[:4]:
            v=d['methods'][m]['boundary'];p=np.array(v['boundary_world_xy']);der=np.array(v['boundary_derivative_world']);centers.append(p)
            axes[1].scatter(*p,c=COLORS[m],s=24)
            if v['boundary_derivative_valid']:axes[1].arrow(*p,*(der/np.linalg.norm(der)*.13),color=COLORS[m],width=.0015,length_includes_head=True)
        q=np.array(centers);lo=q.min(0)-.25;hi=q.max(0)+.25;axes[1].set(xlim=[lo[0],hi[0]],ylim=[lo[1],hi[1]],title='Boundary derivative arrows: RAW / 1cm / 3cm / 5cm')
        fig.legend(*axes[0].get_legend_handles_labels(),loc='lower center',ncol=4,fontsize=9)
        r=raw[key];s=next(r for r in rows if r['pair_id']==key and r['method']=='SPLINE_03cm')
        fig.suptitle(f"{key}: full future RMSE {r['full_rmse_m']:.4f} -> {s['full_rmse_m']:.4f} m\nSaved measured history vs original/fitted predictions; no Graph rerun",fontsize=13)
        save(fig,'case_'+key)
    chord_rows=compare_future_chords(source);write_json(out/'future_chord_comparison.json',chord_rows)
    fig,axes=plt.subplots(1,2,figsize=(14,5));fig.subplots_adjust(top=.83,bottom=.15,wspace=.25)
    for ax,scope in zip(axes,['full','editable']):
        arrays=[[r[scope+'_mean_abs_tangent_deg'] for r in rows if r['method']==m and r.get(scope+'_mean_abs_tangent_deg') is not None] for m in names]
        values=[r['mean_abs_tangent_deg'] for r in chord_rows if r['scope']==scope and r['mean_abs_tangent_deg'] is not None];arrays.insert(1,values)
        ax.boxplot(arrays,labels=[label+f'\nn={len(v)}' for label,v in zip(['Adjacent','Chord','1cm','3cm','5cm','Poly3'],arrays)])
        ax.set(title=scope+' overlap',ylabel='Per-pair mean tangent difference [deg]')
    fig.suptitle('Future OLD/FRESH heading: raw adjacent, raw +/-0.2s chord, fitted derivatives\nSame nominal query times; each estimator retains its explicit reliability mask')
    save(fig,'future_heading_three_estimators')
    pdfs=sorted(dest.glob('*.pdf'));subprocess.run(['pdfunite',*[str(p) for p in pdfs],str(out/'comparison_figures.pdf')],check=True)
    return len(pdfs)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--analysis-dir',type=Path,required=True);p.add_argument('--output-dir',type=Path,required=True);a=p.parse_args()
    hashes=evidence_hashes([a.analysis_dir,Path(__file__)]);out=a.output_dir;out.mkdir(parents=True)
    result=independent_validate(a.analysis_dir);write_json(out/'independent-validation.json',result)
    write_json(out/'descriptive-comparisons.json',describe(a.analysis_dir))
    pages=improve_layout(a.analysis_dir,out);verify_hashes(hashes)
    write_json(out/'metadata.json',dict(source_sha256=hashes,source_analysis=str(a.analysis_dir.resolve()),numeric_results_unchanged=True,book_pages=pages,graph_solves=0,curve_fits=0,model_calls=0,simulation_launches=0))
    print(json.dumps(result));print(str(out))


if __name__=='__main__':main()
