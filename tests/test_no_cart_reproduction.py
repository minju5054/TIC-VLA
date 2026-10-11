import copy
import unittest
from unittest.mock import patch
from research.hospital_no_cart_reproduction import validate_reference,DIFF


class ReproductionConfigTests(unittest.TestCase):
    def test_divergence_requires_two_adjacent_common_bins(self):
        from research.analyze_no_cart_reproduction import first_persistent_disagreement
        rows=[]
        for low in [2,3,4]:
            for name in ['historical','reproduction']:
                sign=-1 if name=='reproduction' and low in [2,4] else 1
                rows.append(dict(run=name,low_y_m=low,count=2,median_w=sign*.1,median_lookahead_left=sign*.1))
        self.assertIsNone(first_persistent_disagreement(rows,1e-6,1e-6))
        rows[3].update(median_w=-.1,median_lookahead_left=-.1)
        self.assertEqual(first_persistent_disagreement(rows,1e-6,1e-6),2)
        rows[1]['count']=0
        self.assertEqual(first_persistent_disagreement(rows,1e-6,1e-6),3)

    def test_progress_selection_uses_positions_not_equal_request_ids(self):
        from research.analyze_no_cart_reproduction import compare_progress
        cfg={'spatial_turn_gate':{'exit_center_x_max_m':2.},'hospital_validation':{
            'command_sign_tolerance_rad_s':1e-6,'position_sign_tolerance_m':1e-6}}
        runs={n:{'cfg':cfg} for n in ['historical','reproduction']};results={}
        for name,ids,sign in [('historical',[3,4],1),('reproduction',[8,9],-1)]:
            obs=[dict(request_id=rid,x_world_m=7.,y_world_m=2.5+i,controller_w=sign*.1,
                      native_lookahead_left_m=sign*.2,application_sim_time=float(rid)) for i,rid in enumerate(ids)]
            results[name]={'observations':obs,'summary':{'first_nonfloor_contact':None,'selected_pair':None}}
        _,d=compare_progress(runs,results)
        self.assertEqual(d['selected']['historical']['request_id'],3)
        self.assertEqual(d['selected']['reproduction']['request_id'],8)
        self.assertFalse(d['selected']['reproduction']['fallback'])

    def test_exact_reference_only(self):
        old={'controller':{'max_v':1.5,'max_w':1.2},'simulation':{'predictions':48},
             'scene':{'goal':[1,2]},'instruction':'Original instruction.','seed':36}
        saved=copy.deepcopy(old)
        with patch('research.hospital_no_cart_reproduction.validate_config'):
            validate_reference(old,copy.deepcopy(old));self.assertEqual(old,saved);self.assertEqual(DIFF,[])
            for section,key,value in [('controller','max_v',1.),('controller','max_w',1.),('simulation','predictions',49),('scene','goal',[2,1])]:
                new=copy.deepcopy(old);new[section][key]=value
                with self.assertRaises(ValueError):validate_reference(old,new)
            for key,value in [('instruction','Original instruction. '),('seed',37),('extra',None)]:
                with self.assertRaises(ValueError):validate_reference(old,dict(old,**{key:value}))

    def test_matching_configs_cannot_authorize_actor_or_speed10(self):
        old={'controller':{'max_v':1.5},'simulation':{'predictions':48},'scene':{}}
        with patch('research.hospital_no_cart_reproduction.validate_config'):
            for key in ['stationary_cart','stationary_human','dynamic_human','pedestrian','obstacles']:
                new=dict(old,**{key:{}})
                with self.assertRaises(ValueError):validate_reference(new,new)
            new=copy.deepcopy(old);new['controller']['max_v']=1.
            with self.assertRaises(ValueError):validate_reference(new,new)


if __name__=='__main__':unittest.main()
