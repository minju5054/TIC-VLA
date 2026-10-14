import unittest
import math
import numpy as np
from tools.c30_oncoming_c10_wheelchair.scene import motion_state


class MotionTest(unittest.TestCase):
    def setUp(self):
        self.h={'path_world_xyz':[[0.,0.,0.],[3.,0.,0.],[3.,4.,0.]],'speed_m_s':1.,'start_sim_s':2.}

    def test_holds_then_moves_at_prescribed_arc_speed(self):
        for t in (0.,1.99):
            p,yaw,active=motion_state(self.h,t)
            np.testing.assert_allclose(p,[0,0,0]);self.assertFalse(active)
        p,yaw,active=motion_state(self.h,4.5)
        np.testing.assert_allclose(p,[2.5,0,0]);self.assertTrue(active);self.assertAlmostEqual(yaw,math.pi/2)

    def test_corner_and_final_clamp_without_teleport(self):
        left=motion_state(self.h,5.-1e-7)[0];right=motion_state(self.h,5.+1e-7)[0]
        self.assertLess(np.linalg.norm(left-right),2e-7)
        p,yaw,active=motion_state(self.h,7.)
        np.testing.assert_allclose(p,[3,2,0]);self.assertAlmostEqual(yaw,math.pi);self.assertTrue(active)
        for t in (9.,20.):
            p,_,active=motion_state(self.h,t);np.testing.assert_allclose(p,[3,4,0]);self.assertFalse(active)

    def test_schedule_uses_prespecified_reference_and_static_gate(self):
        from pathlib import Path
        from unittest.mock import patch
        from tools.c30_oncoming_c10_wheelchair.scene import scenario_config
        fixture={'fixed_wheelchairs':[{'id':'C10'},{'id':'C24'}],'controller':{'max_v':.8}}
        def reader(path):
            if path.name=='summary.json':return {'status':'PASS_THIS_SCENARIO','nonfloor_contact_spans_through_goal_or_end':0}
            if 'two-wheelchairs-v08' in str(path):return {'observation':{'position':[18.,2.,0.],'sim_time':15.}}
            rid=int(path.stem.split('_')[-1]);return {'observation':{'position':[float(rid),2.,0.]}}
        with patch('tools.c30_oncoming_c10_wheelchair.scene.static_config',return_value=fixture),patch('tools.c30_oncoming_c10_wheelchair.scene.read',side_effect=reader):
            cfg=scenario_config(Path('/fixture'),{'stationary_human':{}},Path('original'))
        h=cfg['oncoming_human']
        self.assertEqual([w['id'] for w in cfg['fixed_wheelchairs']],['C10'])
        self.assertEqual(cfg['controller']['max_v'],.8)
        p,_,active=motion_state(h,h['planned_encounter_sim_s'])
        np.testing.assert_allclose(p,h['planned_encounter_world_xyz'],atol=1e-12)
        self.assertTrue(active);self.assertEqual(h['reference_request_ids'],list(range(30,11,-1)))
        self.assertGreater(h['start_sim_s'],0.)

    def test_no_unbound_runtime_globals(self):
        import importlib,builtins,symtable
        from pathlib import Path
        for name in ('scene','runner','analysis'):
            mod=importlib.import_module('tools.c30_oncoming_c10_wheelchair.'+name);path=Path(mod.__file__)
            known=set(vars(mod))|set(vars(builtins));missing=[]
            def check(scope):
                if scope.get_type()=='function':
                    missing.extend(s.get_name() for s in scope.get_symbols() if s.is_global() and s.is_referenced() and s.get_name() not in known)
                for child in scope.get_children():check(child)
            check(symtable.symtable(path.read_text(),str(path),'exec'));self.assertEqual(missing,[],name)

    def test_rotated_boots_keep_left_right_partition(self):
        from tools.c30_oncoming_c10_wheelchair.scene import sole_minima
        boots=np.array([[-.2,-.1,0.],[-.2,.1,.002],[.2,-.1,.000015],[.2,.1,.002]])
        for angle in (0.,math.pi/2,1.737):
            c,s=math.cos(angle),math.sin(angle);matrix=np.eye(4)
            matrix[:2,:2]=[[c,s],[-s,c]];matrix[3,:2]=[-6.17,10.13]
            world=(np.column_stack([boots,np.ones(len(boots))])@matrix)[:,:3]
            np.testing.assert_allclose(sole_minima(world,matrix),[0.,.000015],atol=1e-12)


if __name__=='__main__':unittest.main()
