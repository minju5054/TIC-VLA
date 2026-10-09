"""Pure dynamic-human/replay tests. Synthetic fixtures are not experimental evidence."""
import ast
import copy
import csv
import json
import math
from pathlib import Path
import tempfile
import unittest
import numpy as np
import yaml
from research.nova_dynamic_geometry import (AcceptanceTrigger,human_position,baseline_design,
    polyline_clearance,remaining_curve,temporal_human_clearance,validate_frozen_config)
from research.route_geometry import temporal_overlap,boundary_seam,interpolate,wrap
from research.nova_replay import ReplayData,validate_output
from research.hospital_episode import digest
ROOT=Path(__file__).resolve().parents[1]


class DynamicTests(unittest.TestCase):
    def test_trigger_only_request15_acceptance(self):
        trigger=AcceptanceTrigger()
        for rid in range(1,15):self.assertFalse(trigger.accept(rid,rid*.5))
        self.assertIsNone(trigger.time);self.assertTrue(trigger.accept(15,8.4))
        self.assertEqual(trigger.time,8.4);self.assertFalse(trigger.accept(16,8.9))
        with self.assertRaises(ValueError):trigger.accept(15,9.)

    def test_no_teleport_continuous_fixed_speed(self):
        cfg={"start_position":[1,2,0],"end_position":[2,2,0],"speed_m_s":1.}
        np.testing.assert_array_equal(human_position(cfg,100,None),[1,2,0])
        np.testing.assert_array_equal(human_position(cfg,8.4,8.4),[1,2,0])
        points=np.array([human_position(cfg,t,8.4) for t in np.arange(8.,10.,1/60)])
        self.assertLessEqual(np.linalg.norm(np.diff(points,axis=0),axis=1).max(),1/60+1e-12)
        np.testing.assert_array_equal(human_position(cfg,20,8.4),[2,2,0])

    def test_motion_deterministic(self):
        cfg={"start_position":[-1,0,0],"end_position":[1,2,0],"speed_m_s":1.}
        np.testing.assert_array_equal(human_position(cfg,3.1,2.),human_position(cfg,3.1,2.))
        self.assertAlmostEqual(np.linalg.norm(human_position(cfg,3.1,2.)-cfg["start_position"]),1.1)

    def test_baseline_derived_geometry_and_source_immutability(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);(p/"raw/requests").mkdir(parents=True);(p/"metadata.json").write_text("{}");(p/"robot_state.csv").write_text("fixture")
            chunk=np.column_stack([np.arange(1,31)*.1,np.zeros(30)])
            for rid,x in [(15,0),(16,.5),(17,1.)]:
                e={"agent_pose_at_observation":[x,0,0],"observation":{"sim_time":8+(rid-15)*.5},"application":{"sim_time":8.1+(rid-15)*.5}}
                (p/f"raw/requests/request_{rid:06d}.json").write_text(json.dumps(e));np.save(p/f"raw/requests/request_{rid:06d}.npy",chunk)
            result=baseline_design(p,.6)
            np.testing.assert_allclose(result["center_world_xy"],[1.5,0])
            self.assertAlmostEqual(np.dot(result["baseline_turn_tangent"],result["crossing_normal"]),0)
            for c in result["candidates"]:
                self.assertGreater(c["staged_old_clearance_m"],0);self.assertLess(c["expected_fresh_observation_old_clearance_m"],0)
            self.assertTrue(all(digest(path)==h for path,h in result["source_sha256"].items()))

    def test_frozen_base_and_primary(self):
        cfg=yaml.safe_load((ROOT/"configs/research/nova_e16_dynamic_human.yaml").read_text())
        base=yaml.safe_load((ROOT/"configs/research/nova_carter_hospital_episode16.yaml").read_text())
        validate_frozen_config(cfg,base)
        for key in ["controller","camera","simulation","robot"]:
            changed=copy.deepcopy(cfg);changed[key]["unauthorized_change"]=True
            with self.assertRaises(ValueError):validate_frozen_config(changed,base)
        changed=copy.deepcopy(cfg);changed["dynamic_human"]["trigger_request_id"]=14
        with self.assertRaises(ValueError):validate_frozen_config(changed,base)

    def test_premodel_receipt_rejects_mutation(self):
        from research.nova_dynamic import verify_receipt
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);base=p/"base";base.mkdir()
            cfg=yaml.safe_load((ROOT/"configs/research/nova_e16_dynamic_human.yaml").read_text())
            baseline=yaml.safe_load((ROOT/"configs/research/nova_carter_hospital_episode16.yaml").read_text())
            (base/"metadata.json").write_text(json.dumps({"config":baseline}))
            cfg["dynamic_analysis"]["baseline_run"]=str(base);f=p/"config.yaml";f.write_text(yaml.safe_dump(cfg))
            code=p/"runtime.py";code.write_text("fixed")
            receipt={"authorized_run_id":"fixture","config_sha256":digest(f),"preflight_pass":True,"visual_review_pass":True,
                     "code_sha256":{str(code):digest(code)},"source_sha256":{}}
            verify_receipt(cfg,f,receipt,"fixture")
            code.write_text("changed")
            with self.assertRaises(ValueError):verify_receipt(cfg,f,receipt,"fixture")

    def test_segment_human_clearance_not_nearest_waypoint(self):
        self.assertAlmostEqual(polyline_clearance([[0,0],[2,0]],[1,.2],.5),-.3)
        self.assertAlmostEqual(polyline_clearance([[0,0],[2,0]],[3,0],.5),.5)

    def test_temporal_overlap_frame_invariance(self):
        # Same absolute world straight curve expressed at different observation times.
        t=np.arange(1,31)*.1
        old=np.column_stack([t,np.zeros(30)]);fresh=np.column_stack([t+.5,np.zeros(30)])
        result=temporal_overlap(old,fresh,0,.5)
        self.assertEqual(len(result["absolute_times"]),25);self.assertLess(result["rmse"],1e-12)
        shifted=fresh+[0,.25];self.assertAlmostEqual(temporal_overlap(old,shifted,0,.5)["rmse"],.25)

    def test_dynamic_clearance_between_knots(self):
        result=temporal_human_clearance(np.array([0.,1.]),np.array([[0.,0.],[1.,0.]]),
            np.array([0.,1.]),np.array([[.5,1.],[.5,-1.]]),.2)
        self.assertAlmostEqual(result["minimum_clearance_m"],-.2)

    def test_remaining_curve_and_no_extrapolation(self):
        times,points=remaining_curve(np.column_stack([np.arange(1,31)*.1,np.zeros(30)]),0,.55)
        self.assertAlmostEqual(times[0],.55);self.assertAlmostEqual(points[0,0],.55)
        with self.assertRaises(ValueError):interpolate([0,1],[[0,0],[1,0]],1.1)
        self.assertEqual(remaining_curve(np.zeros((30,2)),0,4),(None,None))

    def test_raw_seam_and_undefined_outside_horizon(self):
        world=np.column_stack([np.arange(1,31)*.1,np.zeros(30)])
        obs={"sim_time":0,"position":[0,0,0]}
        app={"sim_time":.25,"position":[.4,0,0],"pose":[.4,0,0]}
        result=boundary_seam(world,obs,app,np.array([0,.25]),np.array([[0,0],[.4,0]]))
        self.assertAlmostEqual(result["raw_boundary_position_gap_m"],.15)
        self.assertAlmostEqual(result["raw_executed_to_fresh_tangent_gap_deg"],0)
        self.assertAlmostEqual(result["executed_window_start_sim_time"],.15)
        app["sim_time"]=3.5
        self.assertIsNone(boundary_seam(world,obs,app,np.array([0,4]),np.array([[0,0],[4,0]]))["raw_boundary_position_gap_m"])

    def test_tangent_wrap_boundary(self):
        self.assertAlmostEqual(abs(float(wrap(math.radians(-179)-math.radians(179)))),math.radians(2))

    def test_viewer_recorded_transforms_primary_and_immutability(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory);run=p/"run";analysis=p/"analysis";(run/"raw/requests").mkdir(parents=True);analysis.mkdir()
            (run/"metadata.json").write_text('{"config": {}}');(run/"human_asset.json").write_text("{}")
            (analysis/"primary_pair.json").write_text('{"old_request_id":1,"fresh_request_id":2}')
            (analysis/"request_metrics.json").write_text("[]")
            for rid in [1,2]:
                e={"request_id":rid,"agent_pose_at_observation":[rid,0,math.pi/2],"observation":{"sim_time":rid-1},
                   "application":{"sim_time":rid-.8},"controller_command":[1,.2],"rgb_observation_reference":"image.png"}
                (run/f"raw/requests/request_{rid:06d}.json").write_text(json.dumps(e));np.save(run/f"raw/requests/request_{rid:06d}.npy",np.tile([1.,0.],(30,1)))
            robot=[{"tick":i,"sim_time":i*.5,"x":i,"y":0,"z":.1,"qw":1,"qx":0,"qy":0,"qz":0,"yaw":0} for i in range(5)]
            human=[{"sim_time":i*.5,"x":0,"y":i,"z":0,"yaw":.2} for i in range(5)]
            for file,rows in [(run/"robot_state.csv",robot),(run/"raw/human_state.csv",human)]:
                with file.open("w") as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            data=ReplayData(run,analysis);s=data.sample(.7)
            self.assertEqual((data.old,data.fresh),(1,2));self.assertEqual(s["robot_position"],[1.,0.,.1])
            self.assertEqual(s["human_position"],[0.,1.,0.]);np.testing.assert_allclose(data.worlds[0][0],[1,1])
            self.assertEqual(data.sample(1.1)["pending_request"],2)
            self.assertTrue(data.verify_immutable())
            with self.assertRaises(FileExistsError):validate_output(run,analysis,run)
            with self.assertRaises(ValueError):validate_output(run,analysis,run/"viewer")

    def test_viewer_no_model_or_physics_execution(self):
        for filename in ["research/view_nova_dynamic.py","research/nova_replay.py"]:
            tree=ast.parse((ROOT/filename).read_text())
            imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
            self.assertFalse(any(x and any(word in x for word in ["inference_service","continuous","ipc","nova_simulation"]) for x in imports))
            calls=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
            self.assertNotIn("step",calls);self.assertNotIn("play",calls)


if __name__=="__main__":unittest.main()
