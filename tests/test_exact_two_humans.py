"""Exact original-frame placement and safe preflight preparation contracts."""
import builtins
import copy
import importlib
from pathlib import Path
import symtable
import unittest
from unittest.mock import patch
from tools.exact_two_humans.scene import exact_config


class ExactHumanConfig(unittest.TestCase):
    def config(self):
        return {'stationary_human': {'prim':'/old','position':[0,0,0],'yaw':0.,'radius_m':.25}, 'controller':{'max_v':1.5}}

    def reader(self,path):
        if path.name=='metadata.json':return {'config':{'scene':{}}}
        rid=int(path.stem.split('_')[-1]);pose=[float(rid),float(rid*2),.2]
        return {'request_id':rid,'agent_pose_at_observation':pose,'observation':{'pose':pose,'position':[pose[0],pose[1],.000003]}}

    def test_exact_xy_no_legacy_actor_and_no_mutation(self):
        cfg=self.config();old=copy.deepcopy(cfg)
        with patch('tools.exact_two_humans.scene.read',side_effect=self.reader):
            out=exact_config(Path('/r'),cfg,Path('outputs/no-obstacle'))
        self.assertNotIn('stationary_human',out)
        self.assertEqual([h['position'] for h in out['fixed_humans']],[[10.,20.,0.],[24.,48.,0.]])
        self.assertEqual([h['xy_offset_m'] for h in out['fixed_humans']],[[0.,0.],[0.,0.]])
        self.assertEqual([h['id'] for h in out['fixed_humans']],['C10','C24'])
        self.assertEqual(out['controller'],old['controller']);self.assertEqual(cfg,old)

    def test_reference_with_human_rejected(self):
        with patch('tools.exact_two_humans.scene.read',return_value={'config':{'stationary_human':{}}}):
            with self.assertRaisesRegex(ValueError,'no added obstacle'):
                exact_config(Path('/r'),self.config(),Path('outputs/wrong'))

    def test_mesh_bounds_uses_visible_skinned_boot_points(self):
        import numpy as np
        from tools.exact_two_humans.scene import mesh_bounds
        data=[{'path':'/human/opaque__leather__workboots','world_points':np.array([[-.1,0,.15],[.1,0,.1501],[-.1,0,.3],[.1,0,.3]]),'skinned':True}]
        b=mesh_bounds(data);self.assertEqual(b['sole_minima_world_z'],[.15,.1501]);self.assertTrue(b['shoe_skinned'])

    def test_no_unbound_runtime_globals(self):
        for name in ('runner','scene','analysis'):
            mod=importlib.import_module('tools.exact_two_humans.'+name);path=Path(mod.__file__)
            available=set(vars(mod))|set(vars(builtins));missing=[]
            def check(scope):
                if scope.get_type()=='function':
                    missing.extend((scope.get_name(),s.get_name()) for s in scope.get_symbols() if s.is_global() and s.is_referenced() and s.get_name() not in available)
                for child in scope.get_children():check(child)
            check(symtable.symtable(path.read_text(),str(path),'exec'))
            self.assertEqual(missing,[],name)


if __name__=='__main__':unittest.main()
