"""Synthetic unit fixtures are not experimental evidence."""
import ast,copy,csv,json,tempfile,unittest
from pathlib import Path
import numpy as np
from research.nova_bright import image_stats,bright_pass,rgb_difference,calibrated_config,validate_profile_equal
from research.nova_occlusion import segmentation_stats,visibility_state,first_reveal,leakage_pass,assert_stationary,stationary_velocity,candidate_positions
from research.nova_dynamic_geometry import polyline_clearance


def visible_row(rid,kind):
    return {'request_id':rid,'human_visible_pixel_count':0 if kind=='hidden' else 5000 if kind=='visible' else 1,
            'human_visible_fraction':0 if kind=='hidden' else .003 if kind=='visible' else .000001,
            'human_bbox_if_visible':None if kind=='hidden' else [100,100,150,250]}


class OcclusionTests(unittest.TestCase):
    def test_bright_deterministic_complete_stats(self):
        rgb=np.tile(np.arange(256,dtype='uint8')[None,:,None],(32,1,3))
        self.assertEqual(image_stats(rgb),image_stats(rgb))
        self.assertTrue({'mean','std','p05','p50','p95','p99','dark_pixel_fraction','saturated_pixel_fraction'}<=set(image_stats(rgb)))
    def test_black_and_white_rejected(self):
        for color in [0,255]:self.assertFalse(bright_pass(image_stats(np.full((20,30,3),color,dtype='uint8'))))
    def test_bright_profile_copied_without_base_mutation(self):
        base={k:{} for k in ['robot','scene','official_episode','instruction','camera','controller','inference','simulation','seed']}
        cfg=calibrated_config(base,{'renderer_settings':{'exposure':1.}})
        self.assertNotIn('bright_profile',base);other=copy.deepcopy(cfg);validate_profile_equal(cfg,other)
        other['bright_profile']['renderer_settings']['exposure']=2
        with self.assertRaises(ValueError):validate_profile_equal(cfg,other)
    def test_segmentation_count_bbox(self):
        a=np.zeros((100,200),dtype='uint32');a[5:15,20:40]=17
        s,mask=segmentation_stats(a,{'17':{'class':'occluded_human'},'0':{'class':'background'}})
        self.assertEqual(s['human_visible_pixel_count'],200);self.assertEqual(s['human_bbox_if_visible'],[20,5,40,15]);self.assertEqual(mask.sum(),200)
    def test_hidden_visible_and_marginal(self):
        for kind,state in [('hidden','HIDDEN'),('visible','VISIBLE'),('marginal','MARGINAL')]:self.assertEqual(visibility_state(visible_row(1,kind)),state)
    def test_first_event_no_favorable_substitution(self):
        result=first_reveal([visible_row(i,k) for i,k in enumerate(['hidden','marginal','visible','hidden','visible'],1)])
        self.assertEqual((result['old_request_id'],result['fresh_request_id']),(2,3));self.assertFalse(result['valid_transition'])
    def test_hidden_to_first_visible_and_missing(self):
        result=first_reveal([visible_row(1,'hidden'),visible_row(2,'visible')]);self.assertTrue(result['valid_transition'])
        self.assertFalse(first_reveal([visible_row(1,'visible')])['valid_transition'])
        self.assertIsNone(first_reveal([visible_row(1,'hidden')])['fresh_request_id'])
    def test_prereveal_difference_audit(self):
        a=np.ones((100,100,3),dtype='uint8')*100
        self.assertTrue(leakage_pass(a,a)['pass']);self.assertFalse(leakage_pass(a,a+20)['pass'])
        self.assertEqual(rgb_difference(a,a)['mean_abs_0_255'],0)
    def test_stationary_pose_and_velocity(self):
        p=np.tile([3.,4.,0.],(100,1));self.assertTrue(assert_stationary(p))
        np.testing.assert_array_equal(stationary_velocity(p[0],p[1],1/60),[0,0,0])
        p[-1,0]+=.01
        with self.assertRaises(ValueError):assert_stationary(p)
    def test_corridor_candidates_independent_of_native_waypoint(self):
        es=[{'agent_pose_at_observation':[0,0,0]},{'agent_pose_at_observation':[1,2,1]}]
        a=candidate_positions(es,[]);b=candidate_positions(es,[])
        self.assertEqual(a,b);self.assertEqual(a,sorted(a));self.assertGreater(len(a),1)
    def test_old_conflict_and_fresh_clearance(self):
        self.assertLess(polyline_clearance([[0,0],[3,0]],[1,0],.85),0)
        self.assertGreater(polyline_clearance([[0,1],[3,1]],[1,0],.85),0)
    def test_stationary_actor_no_pose_mutation(self):
        import ast
        tree=ast.parse(Path('research/stationary_human.py').read_text())
        update=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='update')
        self.assertFalse(any(isinstance(n,ast.Call) for n in ast.walk(update)))
        self.assertNotIn('AcceptanceTrigger',Path('research/stationary_human.py').read_text())

    def test_stationary_run_gate_rejects_failed_baseline(self):
        from research.nova_bright_run import verify
        from research.hospital_episode import digest
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'config.json';p.write_text('{}')
            receipt={'authorized_run_id':'fixture','config_sha256':digest(p),'mode':'stationary',
                'calibration_visual_pass':True,'bright_baseline_pass':False,'occlusion_preflight_pass':True}
            with self.assertRaises(ValueError):verify({'simulation':{'predictions':48}},p,receipt,'fixture','stationary')

    def test_viewer_no_model_no_physics_no_following_light(self):
        for filename in ['research/view_nova_occlusion.py','research/nova_occlusion_replay.py']:
            tree=ast.parse(Path(filename).read_text())
            imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
            self.assertFalse(any(x and any(k in x for k in ['inference_service','nova_simulation','bright_simulation','ipc']) for x in imports))
            calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
            self.assertNotIn('step',calls);self.assertNotIn('play',calls)
        self.assertNotIn('fill_position',Path('research/view_nova_occlusion.py').read_text())

    def test_stationary_viewer_and_first_reveal_jump(self):
        from research.nova_occlusion_replay import OcclusionReplayData
        from research.hospital_episode import digest
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);run=p/'run';analysis=p/'analysis'
            (run/'raw/requests').mkdir(parents=True);(run/'raw/visibility').mkdir();analysis.mkdir()
            (run/'metadata.json').write_text(json.dumps({'config':{'stationary_human':{}}}))
            (run/'human_asset.json').write_text('{}')
            (analysis/'primary_reveal_pair.json').write_text(json.dumps({'old_request_id':1,'fresh_request_id':2}))
            (analysis/'request_metrics.json').write_text('[]')
            for rid in [1,2]:
                e={'request_id':rid,'agent_pose_at_observation':[rid,0,0],
                    'observation':{'sim_time':float(rid)},'application':{'sim_time':rid+.1},
                    'controller_command':[1,.2],'rgb_observation_reference':f'rgb{rid}.png'}
                (run/f'raw/requests/request_{rid:06d}.json').write_text(json.dumps(e))
                np.save(run/f'raw/requests/request_{rid:06d}.npy',np.tile([1.,0.],(30,1)))
                v=visible_row(rid,'hidden' if rid==1 else 'visible');v['state']=visibility_state(v)
                (run/f'raw/visibility/rgb{rid}.png.json').write_text(json.dumps(v))
            robot=[{'tick':i,'sim_time':i*.5,'x':i,'y':0,'z':0,'qw':1,'qx':0,'qy':0,'qz':0,'yaw':0} for i in range(7)]
            human=[{'sim_time':i*.5,'x':5,'y':1,'z':0,'yaw':0} for i in range(7)]
            for file,rows in [(run/'robot_state.csv',robot),(run/'raw/human_state.csv',human)]:
                with file.open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            data=OcclusionReplayData(run,analysis)
            self.assertEqual(data.jump_time(),.75);self.assertEqual(data.sample(2.05)['visibility_state'],'FIRST VISIBLE')
            self.assertEqual(data.sample(0)['human_position'],data.sample(3)['human_position'])
            self.assertTrue(data.verify_immutable())
            (run/'human_asset.json').write_text('{"changed":true}')
            with self.assertRaises(RuntimeError):data.verify_immutable()

if __name__=='__main__':unittest.main()
