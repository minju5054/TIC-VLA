"""Isaac-independent source/adapter/response tests; fixtures are not evidence."""
import ast
import copy
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
import yaml
from research.nova_source import source_audit, slew, BEHAVIOR
from research.nova_response import extract_schedule, trace, response_metrics
from research.robots.nova_carter import NovaCarter
from research.control import wheel_speeds
from research.records import RunRecords

ROOT=Path(__file__).resolve().parents[1]
DIFF=Path('/home/gpuadmin/isaacsim/extsDeprecated/isaacsim.robot.wheeled_robots/isaacsim/robot/wheeled_robots/controllers/differential_controller.py')


def installed_controller():
    tree=ast.parse(DIFF.read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    class Base:
        def __init__(self,name): self.name=name
    env={"np":np,"BaseController":Base,"ArticulationAction":lambda **kw:SimpleNamespace(**kw)}
    exec(compile(ast.Module(body=[cls],type_ignores=[]),str(DIFF),'exec'),env)
    return env['DifferentialController']('test',.14,.4132)


class NovaTests(unittest.TestCase):
    def test_source_constants_and_asset(self):
        s=source_audit();c=s['constants']
        self.assertEqual(c['_wheel_joints'],['joint_wheel_left','joint_wheel_right'])
        self.assertEqual((c['_wheel_radius'],c['_wheel_base']),(.152,.413))
        self.assertTrue(s['asset'].endswith('/4.5/Isaac/Robots/Carter/nova_carter_sensors.usd'))
        cfg=yaml.safe_load((ROOT/'configs/research/nova_carter_hospital_episode16.yaml').read_text())
        self.assertEqual(cfg['nova_source'],s)
        self.assertEqual(cfg['camera']['prim'],s['front_camera'])
        self.assertEqual(cfg['simulation']['inference_hz'],2)

    @unittest.skipUnless(DIFF.is_file(),'Installed Isaac source unavailable; source oracle is a local integration test')
    def test_installed_differential_mapping_and_positive_sign(self):
        d=installed_controller()
        np.testing.assert_allclose(d.forward([.4,0]).joint_velocities,[.4/.14]*2)
        wheels=d.forward([0,.6]).joint_velocities
        np.testing.assert_allclose(wheels,[-.6*.4132/(2*.14),.6*.4132/(2*.14)])
        self.assertGreater(wheels[1],wheels[0])

    def test_slew_exact_official_source_parity(self):
        n=next(n for n in ast.walk(ast.parse(BEHAVIOR.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='_slew')
        n=copy.deepcopy(n);n.decorator_list=[];env={}
        exec(compile(ast.Module(body=[n],type_ignores=[]),str(BEHAVIOR),'exec'),env)
        for cur,tgt in [(0,.6),(.6,0),(.3,-.4),(-.5,-.2),(.0001,0)]:
            for dt in [0,1/60,.1]:self.assertEqual(slew(cur,tgt,3,3.5,dt,.0005),env['_slew'](cur,tgt,3,3.5,dt,.0005))

    @unittest.skipUnless(DIFF.is_file(),'Installed source unavailable')
    def test_adapter_direct_and_slew_tick_only(self):
        robot=SimpleNamespace(apply_wheel_actions=lambda a:None)
        cfg={'type':'nova_carter','wheel_radius':.14,'wheel_base':.4132}
        direct=NovaCarter(robot,cfg,source_audit()['constants'],controller=installed_controller())
        direct.apply_target(.4,.6);np.testing.assert_array_equal(direct.applied,[.4,.6])
        limited=NovaCarter(robot,cfg,source_audit()['constants'],'official_slew',installed_controller())
        limited.apply_target(.4,.6);limited.apply_target(.4,.6)
        np.testing.assert_array_equal(limited.applied,[0,0])
        limited.before_step(1/60);np.testing.assert_allclose(limited.applied,[2/60,3/60])
        limited.stop();np.testing.assert_array_equal(limited.wheel_targets,[0,0])

    def test_jackal_adapter_regression(self):
        cfg=yaml.safe_load((ROOT/'configs/research/dynanav_hospital_episode16_static.yaml').read_text())
        self.assertNotIn('type',cfg['robot'])
        r=cfg['robot'];actual=wheel_speeds(.4,.6,r['wheel_radius'],r['track_width'],r['joint_signs'])
        self.assertEqual(len(actual),4);self.assertEqual(actual[0],actual[1]);self.assertEqual(actual[2],actual[3])
        self.assertGreater(actual[2],actual[0])

    def test_integral_yaw_unwrap_response_ratio(self):
        t=np.arange(11)*.1;yaw=3.1+.2*t
        rows=[{'sim_time':ti,'x':0,'y':0,'yaw':math.atan2(math.sin(yi),math.cos(yi)),
            'target_w':.4,'applied_w':.2,'body_yaw_rate_measured':.2} for ti,yi in zip(t,yaw)]
        data=trace(rows,'nova');m=response_metrics(data,1,10)
        self.assertAlmostEqual(m['integral_target_w_rad'],.4);self.assertAlmostEqual(m['integral_applied_w_rad'],.2)
        self.assertAlmostEqual(m['actual_delta_yaw_rad'],.2);self.assertAlmostEqual(m['response_ratio_target'],.5)
        self.assertAlmostEqual(m['response_ratio_applied'],1);self.assertEqual(m['sign_agreement_fraction'],1)
        self.assertLess(m['physics_vs_pose_rate_RMSE'],1e-12)

    def test_zero_integral_undefined(self):
        data={'time':np.array([0.,1.]),'xy':np.zeros((2,2)),'yaw':np.zeros(2),
              'target_w':np.zeros(2),'applied_w':np.zeros(2),'omega':np.zeros(2)}
        self.assertIsNone(response_metrics(data,1,1)['response_ratio_target'])

    def test_exact_replay_extraction_and_hashes(self):
        import csv,json
        from research.hospital_episode import digest
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/'raw/requests').mkdir(parents=True);(p/'metadata.json').write_text('{}')
            for rid in range(1,9):
                e={'application':{'tick':8,'sim_time':8/60},'observation':{'tick':16,'sim_time':16/60}}
                (p/f'raw/requests/request_{rid:06d}.json').write_text(json.dumps(e))
            with (p/'robot_state.csv').open('w') as f:
                w=csv.DictWriter(f,fieldnames=['tick','sim_time','active_control_source_request_id','command_v','command_w']);w.writeheader()
                for i in range(17):w.writerow({'tick':i,'sim_time':i/60,'active_control_source_request_id':min(7,max(1,(i-1)//2)),
                    'command_v':1.5,'command_w':i*.01})
            before=digest(p/'robot_state.csv');s=extract_schedule(p)
            self.assertEqual(s['first_applied_primary_tick'],9);self.assertEqual(s['primary_tick_count'],8)
            self.assertEqual(s['commands'][-1]['target_w'],.16)
            self.assertEqual(s['source_sha256'][str(p/'robot_state.csv')],before)
            self.assertEqual(digest(p/'robot_state.csv'),before)

    def test_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'run';RunRecords(p,{})
            with self.assertRaises(FileExistsError):RunRecords(p,{})

    def test_accepted_command_interval_boundaries(self):
        from research.analyze_nova_hospital import accepted_responses
        rows=[];yaw=0.
        for i in range(7):
            w=.2 if i<=3 else .4
            if i:yaw+=w*.1
            rows.append({'tick':str(i),'sim_time':i*.1,'x':0,'y':0,'yaw':yaw,'target_w':w,'applied_w':w,
                         'body_yaw_rate_measured':w,'active_control_source_request_id':str(1 if i<=3 else 2) if i else ''})
        events=[{'request_id':1,'application':{'tick':0,'sim_time':0},'controller_command':[1,.2]},
                {'request_id':2,'application':{'tick':3,'sim_time':.3},'controller_command':[1,.4]}]
        with patch('research.analyze_nova_hospital.contacts',return_value=[]):
            result,_=accepted_responses({'robot':rows,'events':events,'path':Path('fixture')})
        self.assertAlmostEqual(result[0]['integral_target_w_rad'],.06)
        self.assertAlmostEqual(result[1]['integral_target_w_rad'],.12)
        self.assertAlmostEqual(result[0]['response_ratio_target'],1)
        self.assertAlmostEqual(result[1]['hold_duration_s'],.3)

    def test_two_wheel_pending_retention_validator(self):
        import csv,json
        from test_continuous import event_fixture,tick_fixture
        from research.nova_evidence import load_nova,NovaRecords
        from research.records import write_json
        cfg=yaml.safe_load((ROOT/'configs/research/nova_carter_hospital_episode16.yaml').read_text())
        cfg['simulation']['predictions']=3
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'run';records=NovaRecords(path,{'config':cfg,'command_variant':'direct'})
            write_json(path/'inference-runtime.json',{'strict_checkpoint':True,'simulation_app_started':False})
            rows=[]
            for rid in range(1,4):
                e=event_fixture(rid);e['wheel_commands']=[15.,15.];e['old_wheel_commands']=[15.,15.] if rid>1 else None
                e['rgb_observation_reference']=f'diagnostics/rgb_{rid:06d}.png'
                (path/e['rgb_observation_reference']).write_bytes(b'synthetic unit fixture')
                records.request(rid,np.zeros((30,2),dtype=float),e)
                for r in tick_fixture(e):
                    row={k:r[k] for k in ['tick','monotonic_ns','physics_step_start_monotonic_ns','pending_request_id','active_control_source_request_id']}
                    row.update(target_v=r['command_v'],target_w=r['command_w'],applied_v=r['command_v'],applied_w=r['command_w'],wheel_target_left=15. if rid>1 else 0.,wheel_target_right=15. if rid>1 else 0.)
                    rows.append(row)
            def save():
                with (path/'robot_state.csv').open('w') as f:
                    w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            save();self.assertEqual(load_nova(path)['inside_action_ticks'],[2,2,2])
            rows[-1]['target_w']='.1';save()
            with self.assertRaises(ValueError):load_nova(path)


if __name__=='__main__':unittest.main()
