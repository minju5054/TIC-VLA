"""Independent saved evidence and provenance checks; never runs Isaac or a model."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from research.two_hz_preflight import read,digest,early_clear,select,SEPARATION,RADIUS
from research.records import write_json


def historical(out):
    pairs=set()
    def collect(value):
        if not isinstance(value,dict):return
        for k,v in value.items():
            if isinstance(v,str) and re.fullmatch('[0-9a-f]{64}',v):
                p=str(k);prefix=str(ROOT)+'/'
                if p.startswith(prefix):p=p[len(prefix):]
                if p.startswith(('research/','DynaNav/','configs/','scripts/')):pairs.add((p,v))
            elif isinstance(v,dict):collect(v)
    for ref in read(out/'reference-validation.json'):collect(ref['historical_metadata'])
    rows=[];cache={}
    for path,expected in sorted(pairs):
        actual=digest(path) if Path(path).is_file() else None
        if actual==expected:rows.append({'path':path,'expected':expected,'status':'CURRENT_BYTES_MATCH'});continue
        if path not in cache:
            commits=subprocess.check_output(['git','log','--all','--format=%H','--',path],text=True).splitlines()
            cache[path]={}
            for commit in commits:
                p=subprocess.run(['git','show',commit+':'+path],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
                if p.returncode==0:cache[path][hashlib.sha256(p.stdout).hexdigest()]=commit
        commit=cache[path].get(expected)
        rows.append({'path':path,'expected':expected,'current_sha256':actual,
                     'status':'HISTORICAL_GIT_BYTES_MATCH' if commit else 'UNRECOVERED_HISTORICAL_BYTES',
                     'matching_git_commit':commit})
    result={'source_pairs':len(rows),'current_matches':sum(r['status']=='CURRENT_BYTES_MATCH' for r in rows),
            'historical_git_matches':sum(r['status']=='HISTORICAL_GIT_BYTES_MATCH' for r in rows),
            'unrecovered':[r for r in rows if r['status']=='UNRECOVERED_HISTORICAL_BYTES'],'records':rows,
            'note':'Historical dirty-state source content checked by SHA256 against current files or retained Git blobs; neither restored nor executed',
            'verifier_sha256':digest(__file__)}
    write_json(out/'historical-code-verification.json',result)
    print('HISTORICAL_CODE',result['source_pairs'],result['current_matches'],result['historical_git_matches'],len(result['unrecovered']),flush=True)


def evidence(out):
    from PIL import Image
    protected=read(out/'protected-sha256.json');changed=[]
    for p,h in protected.items():
        if not Path(p).is_file():changed.append(p);continue
        if digest(p)==h:continue
        if p=='docs/WORK_LOG.md':
            old=subprocess.check_output(['git','show','2258caa21efed76b2c6be47273313642416ffbfa:'+p])
            if hashlib.sha256(old).hexdigest()==h and Path(p).read_bytes().startswith(old):continue
        changed.append(p)
    if changed:raise ValueError('Protected files changed: '+str(changed))
    freeze=read(out/'stage-b/freeze-receipt.json')
    assert digest(out/'stage-b/frozen-plan.json')==freeze['plan_sha256']
    assert all(digest(p)==h for p,h in freeze['executed_source_sha256'].items())
    adapter=read(out/'stage-b/render-api-adapter.json')
    assert digest(adapter['adapter_source'])==adapter['adapter_sha256']
    assert all(digest(p)==h for p,h in adapter['evidence_sources'].items())
    rows=read(out/'stage-b/qualification-matrix.json');plan=read(out/'stage-b/frozen-plan.json')
    assert len(rows)==len(plan['candidates'])==12
    render_count=0;mask_count=0;physical_checks=0
    for row in rows:
        p=read(out/'stage-b/physical'/f"{row['id']}.json")
        assert p['position_world_xyz']==row['position_world_xyz']
        assert p['human_asset']['asset']==plan['human']['asset']
        for side,d in p['sides'].items():
            if not d.get('vertices_xy'):continue
            v=np.array(d['vertices_xy']);sample=np.array([r['xy'] for r in d['sample_queries']])
            assert np.allclose(sample[0],v[0]) and np.allclose(sample[-1],v[-1])
            assert np.linalg.norm(np.diff(sample,axis=0),axis=1).max()<=.05+1e-12
            recalculated=(d['minimum_center_separation_m']>=SEPARATION and
                d['minimum_structural_clearance']['clearance_m']>=.05 and
                all(r['floor'] and not r['overlap'] and all(f['valid'] for f in r['floor_probes']) for r in d['sample_queries']))
            assert d['pass']==recalculated
            assert d['downstream_after_exclusion_m']>=1
            physical_checks+=1
        path=out/'stage-b/renders'/row['id']/'visibility.json'
        vis=read(path) if path.exists() else []
        for r in vis:
            stem=path.parent/f"C{r['request_id']:02d}"
            rgb=np.asarray(Image.open(str(stem)+'-rgb.png'));mask=np.asarray(Image.open(str(stem)+'-mask.png'))>0
            data=np.load(str(stem)+'-semantic.npy',allow_pickle=False);labels=read(str(stem)+'-labels.json')
            ids=[int(k) for k,v in labels.items() if 'preflight_human' in str(v)]
            assert rgb.shape==(1080,1920,3) and mask.shape==data.shape==(1080,1920)
            assert np.array_equal(mask,np.isin(data,ids));ys,xs=np.where(mask)
            box=[int(xs.min()),int(ys.min()),int(xs.max()+1),int(ys.max()+1)] if len(xs) else None
            assert box==r['human_bbox_if_visible'] and int(mask.sum())==r['human_visible_pixel_count']
            assert float(mask.mean())==r['human_visible_fraction']
            assert not r['timeline_playing'] and r['timeline_time']==0
            assert digest(r['source_rgb'])==r['source_rgb_sha256']
            assert r['early_clear_pass']==early_clear(r)
            source=read(Path(plan['primary'])/'raw/requests'/f"request_{r['request_id']:06d}.json")
            assert r['saved_observation']==source['observation']
            # Independently reconstruct row-vector camera->world from full saved
            # wxyz quaternion, camera->body extrinsic and body->world transform.
            w,x,y,z=np.array(source['observation']['quaternion_wxyz'],float)
            n=np.sqrt(w*w+x*x+y*y+z*z);w,x,y,z=np.array([w,x,y,z])/n
            rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            body=np.eye(4);body[:3,:3]=rotation.T;body[3,:3]=source['observation']['position']
            extrinsic=np.array(read(Path(plan['primary'])/'robot_asset.json')['camera_body_transform'])
            assert np.allclose(extrinsic@body,r['camera_world_matrix_row_vector'],rtol=0,atol=1e-5)
            render_count+=1;mask_count+=int(mask.sum())
        assert row['early_clear_pass']==any(early_clear(r) for r in vis)
        assert row['qualified']==bool(p['physical_pass'] and p['baseline_conflict'] and any(d['pass'] for d in p['sides'].values()) and row['early_clear_pass'])
    decision=read(out/'stage-b/selection-receipt.json');winner=select(rows)
    assert decision['selected_id']==(winner['id'] if winner else None)
    assert decision['model_calls']==decision['checkpoint_loads']==decision['navigation_trials']==decision['physics_steps']==0
    source={str(p):digest(p) for p in out.rglob('*') if p.is_file() and 'attempts' not in p.parts}
    result={'status':'PASS','protected_file_count':len(protected),'protected_changes':changed,
            'frozen_source_matches':True,'render_frames_revalidated':render_count,'human_mask_pixels_recounted':mask_count,
            'connected_side_checks':physical_checks,'selected_id_reproduced':decision['selected_id'],
            'model_calls':0,'checkpoint_loads':0,'navigation_trials':0,'physics_steps':0,
            'source_sha256':source,'verifier_sha256':digest(__file__)}
    write_json(out/'evidence-validation.json',result)
    print('EVIDENCE_PASS',len(protected),render_count,physical_checks,flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--phase',choices=['historical','evidence'],required=True)
    p.add_argument('--output-dir',required=True);a=p.parse_args()
    {'historical':historical,'evidence':evidence}[a.phase](Path(a.output_dir))


if __name__=='__main__':main()
