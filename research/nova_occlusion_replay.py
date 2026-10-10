"""Saved bright baseline/stationary-human replay data, without model or physics."""
import json
from pathlib import Path
import numpy as np
from research.nova_response import read_csv
from research.analyze_chunk_geometry import project_world
from research.hospital_episode import digest
from research.nova_replay import validate_output


class OcclusionReplayData:
    def __init__(self,run_dir,analysis_dir):
        self.source=Path(run_dir).resolve();self.analysis=Path(analysis_dir).resolve()
        self.cfg=json.loads((self.source/'metadata.json').read_text())['config']
        pair=self.analysis/'primary_reveal_pair.json'
        self.is_reveal=pair.is_file()
        self.pair=json.loads((pair if pair.exists() else self.analysis/'primary_pair.json').read_text())
        self.has_event=bool(self.pair.get('old_request_id') and self.pair.get('fresh_request_id'))
        self.old=self.pair.get('old_request_id') or 1;self.fresh=self.pair.get('fresh_request_id') or 2
        self.events=[json.loads(p.read_text()) for p in sorted((self.source/'raw/requests').glob('*.json'))]
        self.worlds=[project_world(np.load(self.source/f"raw/requests/request_{e['request_id']:06d}.npy",allow_pickle=False),e['agent_pose_at_observation']) for e in self.events]
        self.robot=read_csv(self.source/'robot_state.csv');self.times=np.array([float(r['sim_time']) for r in self.robot])
        self.human=read_csv(self.source/'raw/human_state.csv') if 'stationary_human' in self.cfg else None
        self.asset=json.loads((self.source/'human_asset.json').read_text()) if self.human else None
        if self.human:
            from research.nova_occlusion import assert_stationary
            np.testing.assert_allclose(self.times,[float(h['sim_time']) for h in self.human],atol=1e-9,rtol=0)
            assert_stationary([[float(h[k]) for k in ['x','y','z']] for h in self.human])
        self.visibility={}
        for e in self.events:
            p=self.source/'raw/visibility'/(Path(e['rgb_observation_reference']).name+'.json')
            if p.exists():self.visibility[e['request_id']]=json.loads(p.read_text())
        self.metrics=json.loads((self.analysis/'request_metrics.json').read_text())
        summary_path=self.analysis/'summary.json'
        self.summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
        self.first_contact=self.summary.get('first_contact')
        self.first_contact_pose=self.summary.get('first_contact_nearest_robot_tick')
        self.hashes={str(p):digest(p) for p in self.source.rglob('*') if p.is_file()}

    def sample(self,sim_time):
        t=float(np.clip(sim_time,self.times[0],self.times[-1]));i=max(0,int(np.searchsorted(self.times,t,side='right')-1));robot=self.robot[i]
        observed=[e for e in self.events if e['observation']['sim_time']<=t+1e-9];applied=[e for e in self.events if e['application']['sim_time']<=t+1e-9]
        latest=observed[-1] if observed else None;active=applied[-1] if applied else None
        rid=latest['request_id'] if latest else None;visibility=self.visibility.get(rid,{})
        state=visibility.get('state','HUMAN ABSENT' if not self.human else 'UNOBSERVED')
        if self.is_reveal and rid==self.pair.get('fresh_request_id') and state=='VISIBLE':state='FIRST VISIBLE'
        return {'time':t,'recorded_tick':int(robot['tick']),'recorded_pose_time':float(robot['sim_time']),
            'robot_position':[float(robot[k]) for k in ['x','y','z']],
            'robot_quaternion_wxyz':[float(robot[k]) for k in ['qw','qx','qy','qz']],'robot_yaw':float(robot['yaw']),
            'human_position':[float(self.human[i][k]) for k in ['x','y','z']] if self.human else None,
            'latest_request':rid,'active_request':active['request_id'] if active else None,
            'pending_request':rid if latest and (not active or rid>active['request_id']) else None,
            'command':active['controller_command'] if active else [0.,0.],
            'rgb':str(self.source/latest['rgb_observation_reference']) if latest else None,
            'raw_switch':bool(active and t-active['application']['sim_time']<.08),
            'visibility_state':state,'human_pixels':visibility.get('human_visible_pixel_count')}

    def jump_time(self):return max(float(self.times[0]),self.events[self.old-1]['observation']['sim_time']-.25)

    def contact_jump_time(self):
        return max(float(self.times[0]),self.first_contact['start_sim_time']-.5) if self.first_contact else None

    def verify_immutable(self):
        if any(digest(p)!=h for p,h in self.hashes.items()):raise RuntimeError('Source modified')
        return True
