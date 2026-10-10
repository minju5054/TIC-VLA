import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
from research.cart_geometry import freeze_pose,plus_world_y,rotation
from research.cart_bypass import outcome,predicted_side,plot_coordinates
from research.reveal_window import evidence_hashes,verify_hashes


class ShiftTests(unittest.TestCase):
    def test_exact_world_y_preserves_yaw_asset_and_source(self):
        events=[{'request_id':i+1,'agent_pose_at_observation':[2.,11.,3.]} for i in range(20)]
        events[17]['agent_pose_at_observation']=[2.7,11.,3.]
        events[18]['agent_pose_at_observation']=[1.9812549352645874,11.180365562438965,3.030531579778966]
        events[19]['agent_pose_at_observation']=[1.2,11.3,3.1]
        old={'asset':'SM_SupplyCart_01e.usd','cart':freeze_pose(events,[[-.46,-.26,0],[.64,.27,1.11]],0.)};before=copy.deepcopy(old)
        new=copy.deepcopy(old);new['cart']=plus_world_y(old['cart'],events,0.)
        self.assertEqual(new['asset'],old['asset']);self.assertEqual(old,before)
        self.assertEqual(new['cart']['center_world_xyz'],[events[18]['agent_pose_at_observation'][0],events[18]['agent_pose_at_observation'][1]+.4,0.])
        for key in ['yaw_rad','tangent_rad','footprint_half_m','local_bounds']:self.assertEqual(new['cart'][key],old['cart'][key])
        np.testing.assert_array_equal(np.array(new['cart']['matrix_row_vector'])[:3],np.array(old['cart']['matrix_row_vector'])[:3])
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'old.json';p.write_text(json.dumps(old));hashes=evidence_hashes([p]);plus_world_y(old['cart'],events,0.);verify_hashes(hashes)

    def test_side_clearance_and_contact_order(self):
        c={'center_world_xyz':[0.,0.,0.],'yaw_rad':np.pi,'tangent_rad':np.pi,'footprint_half_m':[.5,.25]}
        t=[0.,1.,2.]
        for y,side in [(1.,'NORTH/RIGHT BYPASS'),(-1.,'SOUTH/LEFT BYPASS'),(0.,'CENTER / NO CLEAR BYPASS')]:
            xy=[[2.,y],[0.,y],[-2.,y]];o=outcome(xy,t,c,.5,[],1.5)
            self.assertEqual(o['actual_bypass_side'],side);self.assertAlmostEqual(o['clear_region']['sim_time'],1.5)
            self.assertAlmostEqual(o['distance_travelled_after_clearing_m'],1.)
            o=outcome(xy,t,c,.5,[{'start_sim_time':.9}],1.5)
            self.assertEqual(o['actual_bypass_side'],'CONTACT BEFORE BYPASS');self.assertFalse(o['clears_before_any_contact'])
        self.assertEqual(predicted_side([[3,1],[2,1]],[0,1],c,.5)['side'],'UNKNOWN / DOES NOT REACH')

    def test_comparison_coordinates_are_exact_observations(self):
        run={'xy':np.array([[1.,2.],[3.,4.]]),'times':np.array([0.,1.]),'robot':[{'x':'1','y':'2','yaw':'.7'}],
            'events':[{'request_id':1,'agent_pose_at_observation':[1.,2.,.7],'observation':{'sim_time':0.,'tick':0}}]}
        data=plot_coordinates({'reference':run,'previous':run,'new':run})
        for r in data.values():self.assertEqual(r['world_xy'],[[1.,2.],[3.,4.]]);self.assertEqual(r['observations'][0]['pose'],[1.,2.,.7])
        run['events'][0]['agent_pose_at_observation'][0]=10.
        with self.assertRaises(ValueError):plot_coordinates({'new':run})


if __name__=='__main__':unittest.main()
