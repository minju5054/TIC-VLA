"""Staged simulated Nova validation: A0 sanity, saved command replay, gated model."""
import argparse
import json
import os
from pathlib import Path
import sys
import traceback
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.records import write_json, provenance, clocks
from research.nova_evidence import NovaRecords
from research.hospital_episode import validate_config, digest
from research.nova_source import source_audit
from research.nova_response import extract_schedule, read_csv, trace, response_metrics


def sanity(sim, records, cfg):
    rules = cfg["nova_validation"]
    sim.capture("a0_initial.png")
    bounds = {}
    for phase, v, w, seconds in [("stationary",0,0,rules["stationary_seconds"]),
        ("straight",rules["straight_v"],0,rules["straight_seconds"]),
        ("stop",0,0,rules["stop_seconds"]), ("positive_turn",0,rules["turn_w"],rules["turn_seconds"])]:
        start = sim.tick
        sim.phase = phase; sim.apply(v, w)
        for _ in range(round(seconds/sim.dt)): sim.step()
        bounds[phase] = [start+1, sim.tick]
    sim.capture("a0_after_turn.png"); sim.state_file.flush()
    data = trace(read_csv(records.path/"robot_state.csv"), "nova")
    metrics = {name: response_metrics(data, a, b) for name, (a,b) in bounds.items()}
    n0,n1=bounds["stationary"]
    drift = float(np.max(np.abs(data["yaw"][n0-1:n1+1]-data["yaw"][n0-1])))
    valid = (metrics["straight"]["actual_forward_displacement_m"] > rules["minimum_forward_displacement_m"]
             and metrics["positive_turn"]["actual_delta_yaw_rad"] > rules["minimum_constant_turn_rad"])
    return {"a0_valid": valid, "phase_metrics": metrics, "phase_tick_bounds": bounds,
            "stationary_max_yaw_excursion_rad": drift, "model_calls": 0,
            "decision": "A0 PASS" if valid else "A0 FAIL — STOP BEFORE REPLAY/MODEL"}


def replay(sim, records, source):
    schedule = extract_schedule(source)
    write_json(records.path/"replay_schedule.json", schedule)
    sim.capture("replay_initial.png")
    for command in schedule["commands"]:
        sim.source_tick = command["source_tick"]
        sim.phase = "C4-C7" if command["primary_window"] else "saved_preroll"
        sim.primary_window = command["primary_window"]
        sim.active_control_source_request_id = command["source_request_id"]
        sim.apply(command["target_v"], command["target_w"]); sim.step()
    sim.capture("replay_final.png"); sim.state_file.flush()
    result = response_metrics(trace(read_csv(records.path/"robot_state.csv"),"nova"),
                              schedule["first_applied_primary_tick"],schedule["end_tick_inclusive"])
    return {"model_calls":0,"replayed_tick_count":len(schedule["commands"]),"primary_metrics":result}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",default=str(ROOT/"configs/research/nova_carter_hospital_episode16.yaml"))
    parser.add_argument("--mode",choices=["a0","replay_direct","replay_slew","closed_loop"],required=True)
    parser.add_argument("--run-id",required=True)
    parser.add_argument("--source-run",default=str(ROOT/"outputs/hospital-e16-static-20261009-01"))
    parser.add_argument("--a0-run");parser.add_argument("--stage-b-receipt")
    args=parser.parse_args()
    if Path(args.run_id).name!=args.run_id: parser.error("run-id must be one directory name")
    cfg=yaml.safe_load(Path(args.config).read_text()); validate_config(cfg)
    if cfg["nova_source"]!=source_audit(): parser.error("Nova source identity changed")
    receipt=None
    if args.mode.startswith("replay"):
        prior=json.loads((Path(args.a0_run)/"summary.json").read_text()) if args.a0_run else {}
        if not prior.get("a0_valid"): parser.error("A0 PASS required before replay")
    if args.mode=="closed_loop":
        if not args.stage_b_receipt: parser.error("Stage A/frozen Stage B receipt required")
        receipt=json.loads(Path(args.stage_b_receipt).read_text())
        if (receipt["stage_a_decision"]!="NOVA CARTER ANGULAR RESPONSE VALIDATED" or not receipt["front_rgb_visual_review_pass"]
                or receipt["config_sha256"]!=digest(args.config) or receipt["authorized_run_id"]!=args.run_id
                or any(digest(ROOT/p)!=h for p,h in receipt["code_sha256"].items())):
            parser.error("Stage B gate/freeze mismatch")
    variant="official_slew" if args.mode=="replay_slew" else "direct"
    records=NovaRecords(ROOT/"outputs"/args.run_id,{"mode":args.mode,"config":cfg,"command_variant":variant,
        "command_argv":sys.argv,"stage_b_receipt":receipt,"config_sha256":digest(args.config),
        "all_research_code_sha256":{str(p.relative_to(ROOT)):digest(p) for p in (ROOT/"research").rglob("*.py")},**provenance(ROOT)})
    from isaacsim import SimulationApp
    app=SimulationApp({"headless":True,"renderer":"RayTracedLighting","width":1920,"height":1080})
    sim=None;code=1
    try:
        from research.nova_simulation import NovaSimulation
        import inspect
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController
        controller_source=Path(inspect.getfile(DifferentialController))
        write_json(records.path/"simulation-runtime.json",{"python":sys.version,"numpy":np.__version__,
            "isaac_version":(Path(os.environ["ISAAC_PATH"])/"VERSION").read_text().strip(),
            "differential_controller_source":str(controller_source),"controller_sha256":digest(controller_source)})
        sim=NovaSimulation(cfg,records,variant)
        if args.mode=="a0": result=sanity(sim,records,cfg)
        elif args.mode.startswith("replay"): result=replay(sim,records,args.source_run)
        else:
            from research.continuous import run_model_loop
            from research.nova_evidence import validate_continuous
            sim.phase="closed_loop"
            result=run_model_loop(sim,cfg,records,"static",evidence_analyzer=validate_continuous)
        write_json(records.path/"summary.json",{"status":"PASS", "mode":args.mode,**result})
        print("NOVA_RUN_COMPLETE",json.dumps(result),flush=True);code=0
    except Exception as exc:
        traceback.print_exc();write_json(records.path/"failure.json",{"status":"FAIL","mode":args.mode,"error":repr(exc),**clocks()})
    finally:
        if sim: sim.close()
        app.close(exit_code=code)


if __name__=="__main__": main()
