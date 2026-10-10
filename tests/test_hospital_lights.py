"""Pure lighting contracts; actual USD/renders are separately audited evidence."""
import ast,copy,json,tempfile,unittest
from unittest.mock import patch
from pathlib import Path
from research.hospital_lights import native_profile,fixture_sources
from research.hospital_spatial_gate import spatial_gate,wall_inventory,bound_clearance,first_contact,require_human_prerequisites


class HospitalLightsTests(unittest.TestCase):
    def light(self,path,kind='SphereLight',value=100):
        return {'path':path,'type':kind,'attributes':{'inputs:intensity':value}}

    def test_native_multiplier_deterministic_all_indoor(self):
        lights=[self.light('/a'),self.light('/b',value=200),self.light('/sky','DomeLight')]
        a=native_profile(lights,{'exposure':.02},2);self.assertEqual(a,native_profile(lights,{'exposure':.02},2))
        self.assertEqual([l['path'] for l in a['native_lights']],['/a','/b']);self.assertEqual(a['native_multiplier'],2)
        self.assertEqual([l['base_intensity'] for l in a['native_lights']],[100,200])

    def test_original_renderer_exposure_copied(self):
        original={'exposure':.02,'op':6,'white':[1,1,1]};a=native_profile([],original,1)
        self.assertEqual(original,a['renderer_settings']);a['renderer_settings']['white'][0]=0;self.assertEqual(original['white'][0],1)
        self.assertTrue(a['camera_exposure_unchanged'])

    def test_fixture_selection_exact_asset_families(self):
        fs=[{'mesh':'/Root/Light_test1/Light_test/Light_test'},{'mesh':'/Root/Geo_M_Light2_low1/Geo_M_Light2_low/mesh'},
            {'mesh':'/Root/CeilingTile'},{'mesh':'/Root/Xraylightbox'},{'mesh':'/World/Robot/Light'}]
        self.assertEqual(len(fixture_sources(fs)),2);self.assertEqual(fixture_sources(fs),fixture_sources(list(reversed(fs))))

    def test_light_placement_no_route_input(self):
        tree=ast.parse(Path('research/hospital_lights.py').read_text())
        selection=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='fixture_sources')
        self.assertEqual([a.arg for a in selection.args.args],['fixtures'])
        s=Path('research/prepare_hospital_fixture_lights.py').read_text();self.assertNotIn('agent_pose_at_observation',s);self.assertNotIn('worlds',s)
        for key in ['fixture_world_matrix_row_vector','light_world_matrix_row_vector']:self.assertIn(key,s)

    def test_overrides_session_layer_no_physics_authoring(self):
        s=Path('research/hospital_lights.py').read_text();self.assertIn('GetSessionLayer',s)
        self.assertNotIn('CollisionAPI.Apply',s);self.assertNotIn('RigidBodyAPI.Apply',s)
        self.assertIn("HasAPI(UsdPhysics.CollisionAPI)",s);self.assertNotIn('SetWorldPose',s)

    def gate(self):return {'exit_center_x_max_m':2.5,'center_y_min_m':9.5,'center_y_max_m':11.7}
    def events(self,poses):return [{'request_id':i+1,'agent_pose_at_observation':p} for i,p in enumerate(poses)]

    def test_spatial_crossing_plus_subsequent_observation(self):
        e=self.events([[5,10,2],[2.4,10,3],[1.7,10,3]])
        r=spatial_gate(e,self.gate());self.assertEqual(r['crossing_request_id'],2);self.assertEqual(r['through_request_id'],3);self.assertTrue(r['crossing_plus_one_observation'])

    def test_gate_not_tied_to_request_id(self):
        a=self.events([[5,10,2],[2.4,10,3],[1.7,10,3]])
        b=self.events([[6,8,2],*[e['agent_pose_at_observation'] for e in a]])
        self.assertEqual(spatial_gate(b,self.gate())['crossing_request_id'],spatial_gate(a,self.gate())['crossing_request_id']+1)

    def test_unreached_or_no_subsequent_fails(self):
        for poses in [[[2,12,3],[1,12,3]],[[5,10,2],[2.4,10,3]]]:self.assertFalse(spatial_gate(self.events(poses),self.gate())['crossing_plus_one_observation'])

    def test_first_contact_chronological(self):
        self.assertIsNone(first_contact([]));self.assertEqual(first_contact([{'start_sim_time':3},{'start_sim_time':1}])['start_sim_time'],1)

    def test_clearance_proxy_and_scene_filter(self):
        wall={'path':'/Root/Wall','enabled':True,'world_aabb_min':[0,0,0],'world_aabb_max':[1,1,3]}
        floor={**wall,'path':'/Root/Floor'};walls=wall_inventory([wall,floor]);self.assertEqual(walls,[wall])
        self.assertAlmostEqual(bound_clearance([2,.5],walls,.4)['wall_bound_clearance_m'],.6)
        self.assertLess(bound_clearance([.5,.5],walls,.4)['wall_bound_clearance_m'],0)

    def test_human_requires_both_actual_gates(self):
        for a,b in [(False,False),(False,True),(True,False)]:
            with self.assertRaises(ValueError):require_human_prerequisites({'baseline_spatial_gate_pass':a,'occlusion_preflight_pass':b})
        require_human_prerequisites({'baseline_spatial_gate_pass':True,'occlusion_preflight_pass':True})

    def test_viewer_contact_control_and_v2_lighting(self):
        s=Path('research/view_nova_occlusion.py').read_text();self.assertIn('Jump to first contact',s);self.assertIn('Jump to first reveal',s);self.assertIn('apply_hospital_lights',s)
        from research.nova_occlusion_replay import OcclusionReplayData
        d=object.__new__(OcclusionReplayData);d.times=[0,2];d.first_contact={'start_sim_time':1.2}
        self.assertAlmostEqual(d.contact_jump_time(),.7);d.first_contact=None;self.assertIsNone(d.contact_jump_time())

    def test_authored_audit_includes_inactive_and_instance_lights(self):
        s=Path('research/audit_hospital_lights.py').read_text()
        for contract in ['Usd.PrimAllPrimsPredicate','Usd.TraverseInstanceProxies','UsdLux.LightAPI',
                         'ComputeVisibility','GetAttributes','parent_fixture_path','ComputeLocalToWorldTransform']:
            self.assertIn(contract,s)
        from research.audit_hospital_lights import serial
        import numpy as np
        self.assertEqual(serial(np.eye(2)),[[1.,0.],[0.,1.]])

    def test_versioned_fixture_profile_has_only_scene_fixed_sources(self):
        import yaml,numpy as np
        cfg=yaml.safe_load(Path('configs/research/nova_e16_hospital_lights_on.yaml').read_text())
        p=cfg['hospital_lighting'];lights=p['fallback_fixture_lights']
        self.assertEqual(len(lights),153);self.assertEqual(len({x['source_mesh'] for x in lights}),153)
        self.assertEqual({x['intensity'] for x in lights},{180000.})
        self.assertNotIn('bright_profile',cfg)
        for light in lights:
            self.assertTrue(light['source_mesh'].startswith('/Root/'))
            m=np.array(light['light_world_matrix_row_vector'])
            np.testing.assert_allclose(m[:3,:3]@m[:3,:3].T,np.eye(3),atol=1e-6)
            self.assertGreater(m[2,2],.99)  # area emission local -Z points down

    def test_candidate_grid_is_independent_of_predictions_and_old_runs(self):
        from research.preflight_hospital_occlusion import candidate_grid
        run={'cfg':{'spatial_turn_gate':{'source_south_corner':{'world_aabb_max':[3.159,8.85,3.]},
                                        'source_north_wall':{'world_aabb_min':[.96,12.348,0.]}}}}
        a=candidate_grid(run);run['chunks']='arbitrary unrelated predictions';run['events']=[]
        self.assertEqual(a,candidate_grid(run));self.assertEqual(len(a),192)
        self.assertTrue(all(8.85+.25<=p[1]<12.348-.25 for p in a))

    def test_saved_turn_comparison_uses_physical_region_and_preserves_sources(self):
        from research.analyze_hospital_lights import analyze
        from research.hospital_episode import digest
        gate={'exit_center_x_max_m':2.5,'center_y_min_m':9.5,'center_y_max_m':11.7,
              'robot_conservative_radius_m':.6,'source_south_corner':{'world_aabb_max':[3.1,8.8,3]},
              'source_north_wall':{'world_aabb_min':[0,12.3,0]}}
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=root/'source';original=root/'original';audit=root/'audit'
            for d in [source,original,audit]:d.mkdir()
            (source/'raw/brightness').mkdir(parents=True)
            from research.nova_bright import BRIGHT_RULES
            runs={}
            for d in [source,original]:
                events=[{'request_id':i,'agent_pose_at_observation':pose,'observation':{'sim_time':i},
                         'application':{'sim_time':i+.1},'rgb_observation_reference':f'C{i}.png'}
                        for i,pose in enumerate([[5,10,2],[2.4,10,3],[1.7,10,3]],1)]
                runs[d]={'path':d,'cfg':{'spatial_turn_gate':gate,'bright_validation':BRIGHT_RULES},'events':events,
                         'robot':[{'tick':0,'sim_time':0,'x':5,'y':10,'yaw':2,'target_v':1.5,'target_w':.2}]}
            for i in range(1,4):
                (source/f'raw/brightness/C{i}.png.json').write_text(json.dumps({'mean':120,'std':30,'p50':120,'p95':170,'dark_pixel_fraction':0,'saturated_pixel_fraction':0}))
            (audit/'collision_geometry.json').write_text(json.dumps([{'path':'/Root/Wall','enabled':True,'world_aabb_min':[0,8,0],'world_aabb_max':[9,8.1,3]}]))
            hashes={p:digest(p) for d in [source,original,audit] for p in d.rglob('*') if p.is_file()}
            def turn(d,out):
                out.mkdir();rid=12 if d==source else 15;value=.2 if d==source else .4
                rows=[{'request_id':rid,'robot_world_x':5,'robot_world_y':10,
                       'native_endpoint_left_m':value,'lookahead_left_m':value,'lookahead_bearing_deg':10,'controller_w':value},
                      {'request_id':99,'robot_world_x':20,'robot_world_y':30,'native_endpoint_left_m':99,
                       'lookahead_left_m':99,'lookahead_bearing_deg':99,'controller_w':99}]
                (out/'request_metrics.json').write_text(json.dumps(rows))
                return {'camera_proximity_request_ids':[],'stage_b_decision':'NOVA CARTER E16 TURN EXECUTION VALIDATED',
                        'actual_left_interval_groups_by_fresh_id':[[2,3]],'signed_yaw_change_deg':80,'maximum_signed_yaw_change_deg':90}
            with patch('research.analyze_hospital_lights.turn_analysis',side_effect=turn),\
                 patch('research.analyze_hospital_lights.load_saved',side_effect=lambda d:runs[d]),\
                 patch('research.analyze_hospital_lights.contacts',return_value=[]),\
                 patch('research.analyze_hospital_lights.request_metrics',return_value=[]),\
                 patch('research.analyze_hospital_lights.pair_metrics',return_value=({},{})),\
                 patch('research.analyze_hospital_lights.provenance',return_value={}):
                result=analyze(source,original,audit,root/'output')
            self.assertEqual(result['comparison']['v2']['turn_region_request_ids'],[12])
            self.assertEqual(result['comparison']['original']['turn_region_request_ids'],[15])
            self.assertEqual(result['comparison']['v2']['turn_region_statistics']['lookahead_left_m']['median'],.2)
            self.assertFalse(result['baseline_spatial_gate_pass'])  # three requests are not a 48-request baseline
            self.assertTrue(all(digest(p)==h for p,h in hashes.items()))


if __name__=='__main__':unittest.main()
