"""Two-wheel saved evidence contracts; no aliases to fictitious four-wheel data."""
import json
from pathlib import Path
import numpy as np
from research.control import validate_chunk
from research.records import RunRecords, write_json
from research.nova_response import read_csv


class NovaRecords(RunRecords):
    def request(self, request_id, chunk, metadata):
        v,w=metadata["controller_command"]
        return super().request(request_id,chunk,{**metadata,"target_v":v,"target_w":w,"applied_v":v,"applied_w":w,
            "wheel_order":["joint_wheel_left","joint_wheel_right"],"command_variant":"direct",
            "target_applied_note":"Direct primary variant; no command slew or recovery"})


def load_nova(run_dir, require_complete=True):
    path=Path(run_dir).resolve();meta=json.loads((path/"metadata.json").read_text());cfg=meta["config"]
    runtime=json.loads((path/"inference-runtime.json").read_text())
    if (not runtime["strict_checkpoint"] or runtime["simulation_app_started"] or cfg["simulation"]["pause_physics_during_inference"]
            or meta["command_variant"]!="direct" or cfg["robot"]["type"]!="nova_carter"):
        raise ValueError("Invalid Nova real-model/continuous/direct contract")
    rows=read_csv(path/"robot_state.csv")
    files=sorted((path/"raw/requests").glob("*.json"))
    if require_complete and len(files)!=cfg["simulation"]["predictions"]: raise ValueError("Incomplete Nova predictions")
    if not files or {p.stem for p in files}!={p.stem for p in (path/"raw/requests").glob("*.npy")}:
        raise ValueError("Empty or unpaired Nova requests")
    events=[];chunks=[];inside=[]
    for rid,p in enumerate(files,1):
        e=json.loads(p.read_text());chunk=validate_chunk(np.load(p.with_suffix(".npy"),allow_pickle=False))
        if (e["request_id"]!=rid or e["bootstrap"]!=(rid==1) or e["old_control_source_request_id"]!=(rid-1 if rid>1 else None)
                or e["native_frame"]!="observation_body" or e["native_axes"]!=["forward","left"] or e["native_units"]!="meters"
                or e["native_tensor_shape"]!=[1,30,2] or e["agent_pose_at_observation"]!=e["observation"]["pose"]
                or e["agent_pose_at_ready"] is not None or len(e["wheel_commands"])!=2):
            raise ValueError("Invalid Nova native/anchor/ID contract")
        if rid>1 and (e["old_controller_command"]!=events[-1]["controller_command"] or e["old_wheel_commands"]!=events[-1]["wheel_commands"]):
            raise ValueError("OLD target is not previous accepted FRESH")
        times=[e[k]["monotonic_ns"] for k in ["observation","request_submit","inference_start","ready","response_detected","application"]]
        if times!=sorted(times) or not (path/e["rgb_observation_reference"]).is_file(): raise ValueError("Invalid clocks/RGB")
        selected=[r for r in rows if r["pending_request_id"]==str(rid)]
        obs,det=e["observation"],e["response_detected"]
        if [int(r["tick"]) for r in selected]!=list(range(obs["tick"]+1,det["tick"]+1)):
            raise ValueError("Missing pending physics ticks")
        command=e["old_controller_command"] if rid>1 else [0.,0.]
        wheels=e["old_wheel_commands"] if rid>1 else [0.,0.]
        count=0
        for row in selected:
            expected=str(rid-1) if rid>1 else ""
            actual=[float(row[k]) for k in ["target_v","target_w","applied_v","applied_w","wheel_target_left","wheel_target_right"]]
            if row["active_control_source_request_id"]!=expected or not np.allclose(actual,[*command,*command,*wheels],rtol=0,atol=1e-12):
                raise ValueError("Nova OLD targets changed while pending")
            a,b=int(row["physics_step_start_monotonic_ns"]),int(row["monotonic_ns"])
            if not obs["monotonic_ns"]<=a<=b<=det["monotonic_ns"]: raise ValueError("Pending clocks invalid")
            count+=e["inference_start"]["monotonic_ns"]<=a and b<=e["ready"]["monotonic_ns"]
        inside.append(int(count));events.append(e);chunks.append(chunk)
    return {"path":path,"metadata":meta,"cfg":cfg,"events":events,"chunks":chunks,"robot":rows,"inside_action_ticks":inside}


def validate_continuous(run_dir):
    run=load_nova(run_dir)
    valid=sum(n>0 for n in run["inside_action_ticks"][1:])>=3
    out=Path(run_dir)/"derived/handoff";out.mkdir(parents=True,exist_ok=False)
    summary={"continuous_handoff_validated":valid,"physics_ticks_inside_action_by_request":run["inside_action_ticks"],
             "transport_decision":"NOVA CONTINUOUS OLD TARGET RETENTION VERIFIED; no conflict/correction claim"}
    write_json(out/"summary.json",summary)
    return summary
