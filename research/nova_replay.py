"""Pure recorded-pose replay data; no model, IPC, renderer or physics imports."""
import csv
import json
from pathlib import Path
import numpy as np
from research.analyze_chunk_geometry import project_world
from research.hospital_episode import digest


class ReplayData:
    def __init__(self,run_dir,analysis_dir):
        self.source=Path(run_dir).resolve();self.analysis=Path(analysis_dir).resolve()
        self.cfg=json.loads((self.source/"metadata.json").read_text())["config"]
        self.pair=json.loads((self.analysis/"primary_pair.json").read_text())
        self.old,self.fresh=self.pair["old_request_id"],self.pair["fresh_request_id"]
        self.events=[json.loads(p.read_text()) for p in sorted((self.source/"raw/requests").glob("*.json"))]
        self.worlds=[project_world(np.load(self.source/f"raw/requests/request_{e['request_id']:06d}.npy",allow_pickle=False),e["agent_pose_at_observation"]) for e in self.events]
        with (self.source/"robot_state.csv").open() as f:self.robot=list(csv.DictReader(f))
        with (self.source/"raw/human_state.csv").open() as f:self.human=list(csv.DictReader(f))
        self.times=np.array([float(r["sim_time"]) for r in self.robot])
        if len(self.human)!=len(self.robot):raise ValueError("Replay requires aligned measured actors")
        if not np.allclose(self.times,[float(h["sim_time"]) for h in self.human],rtol=0,atol=1e-9):raise ValueError("Human timing differs")
        self.hashes={str(p):digest(p) for p in self.source.rglob("*") if p.is_file()}
        self.metrics=json.loads((self.analysis/"request_metrics.json").read_text())
        self.asset=json.loads((self.source/"human_asset.json").read_text())

    def sample(self,sim_time):
        t=float(np.clip(sim_time,self.times[0],self.times[-1]));i=max(0,int(np.searchsorted(self.times,t,side="right")-1))
        robot,human=self.robot[i],self.human[i]
        observed=[e for e in self.events if e["observation"]["sim_time"]<=t+1e-9]
        applied=[e for e in self.events if e["application"]["sim_time"]<=t+1e-9]
        latest=observed[-1] if observed else None;active=applied[-1] if applied else None
        pending=latest["request_id"] if latest and (not active or latest["request_id"]>active["request_id"]) else None
        return {"time":t,"recorded_tick":int(robot["tick"]),"recorded_pose_time":float(robot["sim_time"]),
            "robot_position":[float(robot[k]) for k in ["x","y","z"]],
            "robot_quaternion_wxyz":[float(robot[k]) for k in ["qw","qx","qy","qz"]],
            "robot_yaw":float(robot["yaw"]),"human_position":[float(human[k]) for k in ["x","y","z"]],
            "human_yaw":float(human["yaw"]),"latest_request":latest["request_id"] if latest else None,
            "active_request":active["request_id"] if active else None,"pending_request":pending,
            "command":active["controller_command"] if active else [0.,0.],
            "rgb":str(self.source/latest["rgb_observation_reference"]) if latest else None,
            "raw_switch":bool(active and t-active["application"]["sim_time"]<.08)}

    def jump_time(self):
        event=self.events[self.old-1]
        return max(float(self.times[0]),event["application"]["sim_time"]-.25)

    def verify_immutable(self):
        if any(digest(p)!=h for p,h in self.hashes.items()):raise RuntimeError("Replay source modified")
        return True


def validate_output(source,analysis,output):
    paths=[Path(p).resolve() for p in [source,analysis]];out=Path(output).resolve()
    if out.exists():raise FileExistsError(out)
    if any(p==out or p in out.parents or out in p.parents for p in paths):raise ValueError("Separate fresh viewer output required")
    return out
