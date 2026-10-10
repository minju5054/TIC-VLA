import copy,json,tempfile,unittest
from pathlib import Path
import numpy as np
from research.cart_geometry import freeze_pose,clearance,path_clearance,outline,rotation
from research.reveal_window import evidence_hashes,verify_hashes
from research.view_reveal_window_search import restart_state
from unittest.mock import patch


class CartTests(unittest.TestCase):
    def setUp(self):
        self.events=[{'request_id':i+1,'agent_pose_at_observation':[0.,0.,0.]} for i in range(20)]
        self.events[17]['agent_pose_at_observation']=[2.7,11.,3.]
        self.events[18]['agent_pose_at_observation']=[1.9812549352645874,11.180365562438965,3.030531579778966]
        self.events[19]['agent_pose_at_observation']=[1.2,11.3,3.1]
        self.bounds=[[-.46,-.26,0],[.64,.27,1.11]]

    def test_exact_center_deterministic_tangent_and_unchanged_source(self):
        original=copy.deepcopy(self.events);cart=freeze_pose(self.events,self.bounds,0.)
        self.assertEqual(cart['center_world_xyz'][:2],self.events[18]['agent_pose_at_observation'][:2])
        self.assertEqual(cart,freeze_pose(self.events,self.bounds,0.));self.assertEqual(self.events,original)
        self.assertAlmostEqual(cart['yaw_rad'],np.arctan2(.3,-1.5))
        lo,hi=np.array(self.bounds);center=np.r_[(lo[:2]+hi[:2])/2,lo[2],1.]
        np.testing.assert_allclose(center@cart['matrix_row_vector'],[*cart['center_world_xyz'],1.],atol=1e-14)

    def test_original_asset_hash_detects_mutation(self):
        with tempfile.TemporaryDirectory() as tmp:
            asset=Path(tmp)/'cart.usda';asset.write_text('#usda 1.0\ndef Xform "Cart" {}')
            before=asset.read_bytes();frozen=evidence_hashes([asset]);verify_hashes(frozen)
            self.assertEqual(before,asset.read_bytes());asset.write_text('changed')
            with self.assertRaises(ValueError):verify_hashes(frozen)

    def test_freeze_preserves_evidence_and_code_provenance_separately(self):
        from research import e16_cart_run
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);pre=root/'preflight';pre.mkdir()
            for name,data in [('physical.json',{'pass':True}),('summary.json',{'model_run_permitted':True}),('freeze.json',{'source':'cart'})]:
                (pre/name).write_text(json.dumps(data))
            base=root/'outputs/nova-e16-hospital-lights-baseline-20261010-01';base.mkdir(parents=True)
            (base/'metadata.json').write_text(json.dumps({'config':{}}))
            before=evidence_hashes([pre]);out=root/'receipt'
            with patch.object(e16_cart_run,'ROOT',root),patch.object(e16_cart_run,'validate_config'),patch.object(e16_cart_run,'provenance',return_value={'source_sha256':{'code.py':'code-hash'}}):
                e16_cart_run.freeze(pre,out,'one-run')
            receipt=json.loads((out/'receipt.json').read_text())
            self.assertEqual(receipt['source_evidence_sha256'],before)
            self.assertEqual(receipt['source_sha256'],{'code.py':'code-hash'})
            verify_hashes(before)

    def test_oriented_rectangle_clearance_and_continuous_conflict(self):
        cart={'center_world_xyz':[0.,0.,0.],'yaw_rad':0.,'footprint_half_m':[1.,.5]}
        np.testing.assert_allclose(clearance([[0,0],[2,0],[2,1.5]],cart,.2),[-.7,.8,np.sqrt(2)-.2])
        hit=path_clearance([[-3,0],[3,0]],cart,.2,[0.,6.])
        self.assertAlmostEqual(hit['minimum_clearance_m'],-.7)
        self.assertAlmostEqual(hit['first_conflict']['sim_time'],1.8)
        self.assertAlmostEqual(path_clearance([[-3,1],[3,1]],cart,.2)['minimum_clearance_m'],.3)
        cart['yaw_rad']=np.pi/2
        self.assertAlmostEqual(path_clearance(np.array([[-3,0],[3,0]])@rotation(np.pi/2).T,cart,.2)['minimum_clearance_m'],-.7)

    def test_plot_outline_exact_center_and_radius(self):
        cart=freeze_pose(self.events,self.bounds,0.);p=outline(cart)
        local=(p[:-1]-cart['center_world_xyz'][:2])@rotation(cart['yaw_rad'])
        np.testing.assert_allclose(np.max(local,axis=0),cart['footprint_half_m'],atol=1e-14)
        np.testing.assert_allclose(clearance(outline(cart,.6),cart,0.),.6,atol=1e-14)

    def test_replay_starts_at_zero(self):
        self.assertEqual(restart_state([0.,1.]),{'time':0.,'playing':False})


if __name__=='__main__':unittest.main()
