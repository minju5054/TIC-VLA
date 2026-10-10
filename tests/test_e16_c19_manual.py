import copy
import unittest
import numpy as np
from research.e16_c19_manual import placement
from research.reveal_window import first_clear, conflict_time


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


if __name__=='__main__':unittest.main()
