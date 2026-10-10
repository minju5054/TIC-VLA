import copy
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np
from research.e16_c19_manual import placement,check_frozen
from research.reveal_window import first_clear,conflict_time,evidence_hashes,verify_hashes
from research.view_reveal_window_search import restart_state,replay_frame_times


class ManualC19Tests(unittest.TestCase):
    def test_exact_immutable_world_y_offset_and_facing(self):
        event={'request_id':19,'agent_pose_at_observation':[1.9812549352645874,11.180365562438965,3.030531579778966]}
        before=copy.deepcopy(event);h=placement(event)
        self.assertEqual(h['position'],[event['agent_pose_at_observation'][0],event['agent_pose_at_observation'][1]+.40,0.])
        self.assertAlmostEqual(h['yaw'],-.11106107381082708,places=14)
        self.assertEqual(event,before)
        # World offset remains +Y even when robot heading changes.
        event['agent_pose_at_observation'][2]=np.pi/2
        self.assertEqual(placement(event)['position'],h['position'])
        self.assertAlmostEqual(placement(event)['yaw'],-np.pi/2)
        event['request_id']=20
        with self.assertRaises(ValueError):placement(event)

    def test_first_clear_failure_cannot_select_later_conflict(self):
        rows=[{'request_id':i+1,'state':s} for i,s in enumerate(['HIDDEN','MARGINAL','CLEAR','HIDDEN','HIDDEN','HIDDEN','CLEAR'])]
        first=first_clear(rows)
        self.assertEqual((first['old_request_id'],first['fresh_request_id']),(2,3))
        self.assertFalse(first['three_prior_hidden']);self.assertFalse(first['immediate_old_hidden'])
        self.assertIsNone(conflict_time([2.,3.],[[0.,0.],[1.,0.]],[4.,0.],.5))
        self.assertAlmostEqual(conflict_time([6.,7.],[[2.,0.],[4.,0.]],[4.,0.],.5)['time'],6.75)

    def test_minus_world_y_receipt_and_source_immutability(self):
        event={'request_id':19,'agent_pose_at_observation':[1.9812549352645874,11.180365562438965,3.030531579778966]}
        before=copy.deepcopy(event);h=placement(event,-.4)
        self.assertEqual(h['position'],[1.9812549352645874,10.780365562438964,0.])
        self.assertEqual(h['yaw'],placement(event,.4)['yaw']);self.assertEqual(event,before)
        with self.assertRaises(ValueError):placement(event,-.5)
        with tempfile.TemporaryDirectory() as tmp:
            source=Path(tmp)/'source';raw=source/'raw/requests';raw.mkdir(parents=True)
            (raw/'request_000019.json').write_text(json.dumps(event));np.save(raw/'request_000019.npy',np.ones((30,2)))
            hashes=evidence_hashes([source]);out=Path(tmp)/'derived';out.mkdir()
            frozen={'human':h,'world_y_offset_m':-.4,'source_evidence_sha256':hashes}
            (out/'freeze.json').write_text(json.dumps(frozen))
            with patch('research.e16_c19_manual.BASELINE',source),patch('research.e16_c19_manual.load_saved',return_value={'path':source}):
                self.assertEqual(check_frozen(out)[0]['human'],h)
                self.assertEqual(evidence_hashes([source]),hashes);verify_hashes(hashes)
                frozen['human']['position'][1]+=.01;(out/'freeze.json').write_text(json.dumps(frozen))
                with self.assertRaisesRegex(ValueError,'Frozen placement changed'):check_frozen(out)
            (raw/'request_000019.npy').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError,'Source changed'):verify_hashes(hashes)

    def test_saved_replay_starts_at_zero_and_exports_whole_run(self):
        times=np.linspace(0,25.300001319,1519);state={'time':10.3,'playing':True}
        state.update(restart_state(times));self.assertEqual(state,{'time':0.,'playing':False})
        frames=replay_frame_times(times)
        self.assertEqual(frames[0],0.);self.assertEqual(frames[-1],times[-1])
        self.assertTrue(np.all(np.diff(frames)>0));self.assertLessEqual(max(np.diff(frames)),.10001)
        with self.assertRaises(ValueError):restart_state([1.,2.])


if __name__=='__main__':unittest.main()
