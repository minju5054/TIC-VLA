import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
from research.cart_geometry import freeze_pose,rotation
from research.e16_c26_tools import preflight_gate,camera_projection,instruction_audit,precontact
from research.analyze_chunk_geometry import project_world
from research.cart_bypass import plot_coordinates
from research.reveal_window import evidence_hashes,verify_hashes


class C26Tests(unittest.TestCase):
    def test_exact_c26_pivot_long_axis_and_immutability(self):
        events=[{'request_id':i,'agent_pose_at_observation':[0.,0.,0.]} for i in range(1,49)]
        a=[-2.4961745738983154,10.860149383544922,-2.9561992846467255]
        b=[-3.2318756580352783,10.711772918701172,-2.9310779489526193]
        c=[-3.965388536453247,10.552887916564941,-2.930181199669514]
        for i,p in [(24,a),(25,b),(26,c)]:events[i]['agent_pose_at_observation']=p
        bounds=[[-.46186652104854886,-.2624729097826872,0.],[.6399123239536806,.2649667299173757,1.1127104701240995]]
        before=copy.deepcopy((events,bounds));cart=freeze_pose(events,bounds,3.8743019104003906e-7,26)
        self.assertEqual(cart['center_world_xyz'],b[:2]+[3.8743019104003906e-7]);self.assertEqual((events,bounds),before)
        self.assertAlmostEqual(cart['yaw_rad'],-2.9354307603346976)
        pivot=[*(np.mean(bounds,axis=0)[:2]),0.,1.]
        np.testing.assert_allclose((pivot@np.array(cart['matrix_row_vector']))[:3],cart['center_world_xyz'],rtol=0,atol=1e-14)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'asset.json';p.write_text(json.dumps({'asset':'SM_SupplyCart_01e.usd','cart':cart}));h=evidence_hashes([p]);freeze_pose(events,bounds,0,26);verify_hashes(h)
            p.write_text('{}')
            with self.assertRaises(ValueError):verify_hashes(h)
        with self.assertRaises(ValueError):freeze_pose(events,bounds,0,25)

    def test_c1_including_single_pixel_blocks_and_event_order(self):
        p={'pass':True,'checks':{'local_bypass':True},'baseline':{'first_conflict':{'sim_time':3.}}}
        v=[{'request_id':1,'state':'HIDDEN','cart_visible_pixel_count':0,'sim_time':0.},
           {'request_id':2,'state':'CLEAR','cart_visible_pixel_count':5000,'sim_time':1.}]
        r={'events':[{'application':{'sim_time':.1}},{'application':{'sim_time':1.1}}]}
        self.assertTrue(preflight_gate(p,v,r)['model_run_permitted'])
        v[0].update(state='MARGINAL',cart_visible_pixel_count=1)
        self.assertIn('C1_CART_VISIBLE_INCLUDING_MARGINAL',preflight_gate(p,v,r)['stop_reasons'])
        v[0].update(state='HIDDEN',cart_visible_pixel_count=0);r['events'][1]['application']['sim_time']=3.
        self.assertFalse(preflight_gate(p,v,r)['model_run_permitted'])
        r['events'][1]['application']['sim_time']=1.1;p['pass']=False;p['checks']['local_bypass']=False
        self.assertFalse(preflight_gate(p,v,r)['model_run_permitted'])

    def test_camera_row_transform_and_fisheye_scaling(self):
        m=np.eye(4);m[3,:3]=[2,3,4]
        c={'model':'fisheyePolynomial','polynomial':[0,.001,0,0,0,0],
           'nominal_resolution':[1920.,1200.],'optical_center':[960.,600.],'max_fov_deg':150.}
        theta=.3;xyz=np.array([[0,0,-2],[2*np.sin(theta),0,-2*np.cos(theta)],[0,1,-2],[0,0,1]])+[2,3,4]
        result=camera_projection(xyz,m,c)
        np.testing.assert_allclose(result['pixels'][:2],[[960,540],[1260,540]],atol=1e-9)
        self.assertLess(result['pixels'][2][1],540);self.assertFalse(result['inside_fov'][3])

    def test_own_observation_anchors_and_exact_plotting_coordinates(self):
        world=np.array([[4.,2.],[5.,3.]]);poses=[[1.,2.,0.],[2.,1.,np.pi/2]]
        for pose in poses:
            native=(world-pose[:2])@rotation(pose[2]);np.testing.assert_allclose(project_world(native,pose),world,atol=1e-14)
        run={'xy':world,'times':np.array([0.,1.]),'robot':[{'x':'4.0','y':'2.0','yaw':'0.7'}],
             'events':[{'request_id':1,'agent_pose_at_observation':[4.,2.,.7],'observation':{'sim_time':0.,'tick':0}}]}
        self.assertEqual(plot_coordinates({'baseline':run})['baseline']['world_xy'],world.tolist())
        run['events'][0]['agent_pose_at_observation'][0]+=.01
        with self.assertRaises(ValueError):plot_coordinates({'baseline':run})

    def test_instruction_equality_rejects_right_directive_and_mismatch(self):
        text='Move forward toward the staircase, then turn left to enter the hallway. Continue straight ahead and stop in front of the blue hospital bed on the right side of the hallway.'
        r={'cfg':{'instruction':text,'official_episode':{'episode':{'instruction':text}}}}
        self.assertFalse(instruction_audit({'baseline':r,'C19':r,'plus_Y':r})['right_on_obstacle_directive'])
        wrong=copy.deepcopy(r);wrong['cfg']['instruction']='Turn right when encountering an obstacle.'
        with self.assertRaises(ValueError):instruction_audit({'baseline':r,'new':wrong})
        wrong['cfg']['official_episode']['episode']['instruction']=wrong['cfg']['instruction']
        with self.assertRaises(ValueError):instruction_audit({'new':wrong})

    def test_contact_at_application_is_not_precontact(self):
        row={'fresh_observation_sim_time':1.,'application_sim_time':1.1}
        self.assertTrue(precontact(row,None));self.assertTrue(precontact(row,1.2));self.assertFalse(precontact(row,1.1));self.assertFalse(precontact(row,1.05))


if __name__=='__main__':unittest.main()
