import copy
import unittest
from unittest.mock import patch
from research.hospital_no_cart_speed import validate_speed,stage_audit,DIFF
from research.e16_c26_speed_ablation import config_diff


class NoCartSpeedTests(unittest.TestCase):
    def test_outcomes_do_not_confuse_no_contact_with_success(self):
        from research.analyze_no_cart_speed import classify
        self.assertEqual(classify(False,None,None,None,False),'EXECUTION FAILED')
        self.assertEqual(classify(True,None,None,None,False),'INCOMPLETE — CORNER NOT REACHED')
        self.assertEqual(classify(True,{},None,None,False),'OTHER NAVIGATION FAILURE')
        self.assertEqual(classify(True,{},10.,None,False),'OTHER NAVIGATION FAILURE')
        self.assertEqual(classify(True,{},10.,None,True),'CORNER PASSED WITHOUT CONTACT')
        for contact in [9.,10.]:
            self.assertEqual(classify(True,{},10.,contact,True),'STRUCTURAL CONTACT BEFORE CORNER PASSAGE')
        self.assertEqual(classify(True,{},10.,11.,True),'OTHER NAVIGATION FAILURE')

    def test_corner_entry_uses_original_spatial_bounds(self):
        import numpy as np
        from research.analyze_no_cart_speed import in_corner
        cfg={'robot':{'start_position':[7,1,0]},'spatial_turn_gate':{'exit_center_x_max_m':2.,
             'robot_conservative_radius_m':.6,'center_y_min_m':9.,'center_y_max_m':12.}}
        np.testing.assert_array_equal(in_corner([[2,9],[7.6,12],[1.99,10],[3,12.01],[7.61,10]],cfg),[True,True,False,False,False])

    def test_only_speed_and_no_actor_guard(self):
        old={'controller':{'max_v':1.5,'max_w':1.2,'lookahead':1.,'filter_gain':.35},
             'simulation':{'predictions':48},'scene':{'goal':[1,2]},'seed':36}
        saved=copy.deepcopy(old);new=copy.deepcopy(old);new['controller']['max_v']=1.
        with patch('research.hospital_no_cart_speed.validate_config'):
            validate_speed(old,new);self.assertEqual(old,saved);self.assertEqual(config_diff(old,new),DIFF)
            for key in ['stationary_cart','stationary_human','dynamic_human','pedestrian','obstacles']:
                with self.assertRaises(ValueError):validate_speed(old,dict(new,**{key:{}}))
            for section,key,value in [('controller','max_w',1.),('controller','lookahead',.9),('controller','filter_gain',.4),('simulation','predictions',60),('scene','obstacles',[])]:
                wrong=copy.deepcopy(new);wrong[section][key]=value
                with self.assertRaises(ValueError):validate_speed(old,wrong)
            for key,value in [('seed',37),('extra',None)]:
                with self.assertRaises(ValueError):validate_speed(old,dict(new,**{key:value}))

    def test_stage_audit_rejects_cart_even_if_actor_is_none(self):
        import tempfile,json,hashlib
        from pathlib import Path
        from types import SimpleNamespace
        inventory=[{'original':'collider'}];h=hashlib.sha256(json.dumps(inventory,sort_keys=True).encode()).hexdigest()
        with tempfile.TemporaryDirectory() as tmp,patch('research.audit_hospital_lights.inspect_stage',return_value=[inventory]):
            p=Path(tmp);(p/'hospital_lighting_applied.json').write_text(json.dumps({'collision_inventory_sha256':h}))
            sim=SimpleNamespace(stage=SimpleNamespace(GetPrimAtPath=lambda path:False),pedestrian=None)
            self.assertTrue(stage_audit(sim,p)['no_world_e16cart'])
            sim.stage.GetPrimAtPath=lambda path:True
            with self.assertRaises(ValueError):stage_audit(sim,p)
            sim.stage.GetPrimAtPath=lambda path:False;sim.pedestrian=object()
            with self.assertRaises(ValueError):stage_audit(sim,p)


if __name__=='__main__':unittest.main()
