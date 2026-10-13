import builtins
import importlib
import symtable
from pathlib import Path
import unittest
from unittest.mock import patch
from tools.c24_wheelchair.scene import scenario_config


class WheelchairScenario(unittest.TestCase):
    def test_replace_only_original_c24(self):
        def read(path):
            if path.name=='metadata.json':return {'config':{}}
            rid=int(path.stem.split('_')[-1]);pose=[float(rid),float(rid*2),0.]
            return {'request_id':rid,'agent_pose_at_observation':pose,'observation':{'pose':pose}}
        original={'stationary_human':{'radius_m':.25,'yaw':0.},'controller':{'max_v':1.5}}
        with patch('tools.exact_two_humans.scene.read',side_effect=read):cfg=scenario_config(Path('/r'),original,Path('outputs/ref'))
        self.assertEqual([h['id'] for h in cfg['fixed_humans']],['C10'])
        self.assertEqual(cfg['fixed_humans'][0]['position'],[10.,20.,0.])
        self.assertEqual(cfg['fixed_wheelchair']['position'],[24.,48.,0.])
        self.assertEqual(cfg['fixed_wheelchair']['xy_offset_m'],[0.,0.])
        self.assertEqual(cfg['controller'],original['controller']);self.assertIn('stationary_human',original)

    def test_runtime_has_no_unbound_globals(self):
        for name in ('runner','scene','analysis'):
            mod=importlib.import_module('tools.c24_wheelchair.'+name);path=Path(mod.__file__);missing=[]
            available=set(vars(mod))|set(vars(builtins))
            def check(scope):
                if scope.get_type()=='function':missing.extend((scope.get_name(),s.get_name()) for s in scope.get_symbols() if s.is_global() and s.is_referenced() and s.get_name() not in available)
                for child in scope.get_children():check(child)
            check(symtable.symtable(path.read_text(),str(path),'exec'));self.assertEqual(missing,[],name)
