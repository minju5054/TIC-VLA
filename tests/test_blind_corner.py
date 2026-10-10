import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
import yaml
from research.blind_corner import reference_route,select_route,baseline_crossing,plotted_world
from research.hospital_episode import official_episode,validate_config
from research.reveal_window import first_clear,conflict_time,validate_destination

RULES=json.loads(Path('configs/research/blind_corner_protocol.json').read_text())


class BlindCornerTests(unittest.TestCase):
    def test_reference_starts_at_official_pose_and_turns_right(self):
        ep=official_episode('episode_21')['episode'];p=reference_route(ep,19.25,RULES)
        np.testing.assert_allclose(p[0],[*ep['start'][:2],np.pi]);self.assertLess(p[-1,2],p[0,2]);self.assertAlmostEqual(p[-1,1],5.)
        np.testing.assert_array_equal(p,reference_route(ep,19.25,RULES))

    def test_deterministic_geometry_selection(self):
        a={'id':1,'geometry_pass':True,'lane_x':19.25,'human_position':[19.25,1,0],'transition':{'new_samples':15,'distance_m':2.}}
        b=copy.deepcopy(a);b['id']=2;b['transition']['new_samples']=14
        self.assertEqual(select_route([b,a])['id'],1);self.assertEqual(select_route([a,b])['id'],1)
        a['geometry_pass']=False;self.assertEqual(select_route([a,b])['id'],2)

    def test_official_episode21_explicit_opt_in(self):
        cfg=yaml.safe_load(Path(RULES['base_config']).read_text());src=official_episode('episode_21');ep=src['episode']
        cfg.update(blind_corner_episode_id='episode_21',official_episode=src,instruction=ep['instruction'])
        cfg['scene']['goal']=ep['goal'];cfg['robot']['start_position']=[*ep['start'][:2],.11];cfg['robot']['start_yaw']=src['resolved_yaw_radians']
        self.assertEqual(validate_config(cfg),src)
        cfg.pop('blind_corner_episode_id')
        with self.assertRaises(ValueError):validate_config(cfg)

    def test_first_clear_never_substituted(self):
        rows=[{'request_id':i+1,'state':s} for i,s in enumerate(['MARGINAL','CLEAR','HIDDEN','HIDDEN','HIDDEN','CLEAR'])]
        event=first_clear(rows);self.assertEqual(event['fresh_request_id'],2);self.assertFalse(event['three_prior_hidden'])
        rows=[{'request_id':i+1,'state':s} for i,s in enumerate(['HIDDEN']*3+['CLEAR'])]
        self.assertTrue(first_clear(rows)['three_prior_hidden'])

    def test_remaining_conflict_is_nominal_segment_entry(self):
        hit=conflict_time([2,3],[[0,0],[2,0]],[2,0],.5)
        self.assertAlmostEqual(hit['time'],2.75);np.testing.assert_allclose(hit['point'],[1.5,0])

    def test_plot_uses_own_observation_not_application(self):
        chunk=np.array([[1.,0.],[2.,0.]]);before=chunk.copy()
        observation={'pose':[10.,20.,np.pi/2]}
        np.testing.assert_allclose(plotted_world(chunk,observation),[[10,21],[10,22]],atol=1e-12)
        np.testing.assert_array_equal(chunk,before)

    def test_crossing_requires_following_observation(self):
        es=[{'agent_pose_at_observation':[19.,1.,np.pi/2]}]
        self.assertEqual(baseline_crossing(es,RULES['baseline_gate']),(0,None))
        self.assertEqual(baseline_crossing(es*2,RULES['baseline_gate']),(0,1))

    def test_output_cannot_overwrite_or_nest_in_source(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'source';source.mkdir();raw=source/'raw';raw.write_bytes(b'unchanged')
            with self.assertRaises(ValueError):validate_destination(source/'derived',[source])
            with self.assertRaises(FileExistsError):validate_destination(source,[])
            self.assertEqual(raw.read_bytes(),b'unchanged')

    def test_stopped_baseline_plot_preserves_saved_world_coordinates(self):
        from research.analyze_blind_corner import trajectory_plot
        with tempfile.TemporaryDirectory() as d:
            out=Path(d);inventory=out/'colliders.json';inventory.write_text('[]')
            xy=np.array([[23.3,-.9],[20.,0.],[19.,1.]])
            run={'path':Path('test-baseline'),'xy':xy,'times':np.array([0.,1.,2.]),
                 'cfg':{'blind_corner':{'baseline_gate':RULES['baseline_gate']}}}
            trajectory_plot(out,run,{'human_position':[20.,2.5,0.]},
                            {'decision':'BASELINE FAIL','failed_gates':['useful_rgb']},inventory)
            coords=json.loads((out/'trajectory_coordinates.json').read_text())
            np.testing.assert_array_equal(coords['executed_robot_world_xy'],xy)
            self.assertFalse(coords['human_executed']);self.assertNotIn('fresh_world_xy',coords)
            self.assertEqual(inventory.read_text(),'[]')
            self.assertTrue((out/'trajectory_result.pdf').read_bytes().startswith(b'%PDF'))
            self.assertTrue((out/'trajectory_result.png').is_file())


if __name__=='__main__':unittest.main()
