import csv
import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from research.plot_e16_observation_path import load_observations, draw_map
from research.reveal_window import evidence_hashes, verify_hashes


class ObservationPlotTest(unittest.TestCase):
    def test_markers_use_exact_observation_pose_not_application_or_waypoint(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)
            requests = source/'raw/requests'; requests.mkdir(parents=True)
            (source/'summary.json').write_text(json.dumps({'status':'PASS', 'all_finite':True, 'successful_predictions':2}))
            poses = [[7.38, 6., 1.8], [6., 8., 2.1]]
            with (source/'robot_state.csv').open('w') as f:
                writer = csv.writer(f); writer.writerow(['tick', 'sim_time', 'x', 'y', 'yaw'])
                for i, pose in enumerate(poses):
                    writer.writerow([i, i*.5, *pose])
                    event = {'request_id':i+1, 'agent_pose_at_observation':pose,
                             'observation':{'tick':i, 'sim_time':i*.5, 'pose':pose, 'position':[*pose[:2], 0.]},
                             'agent_pose_at_application':[100., 100., 0.]}
                    (requests/f'request_{i+1:06d}.json').write_text(json.dumps(event))
                    np.save(requests/f'request_{i+1:06d}.npy', np.full((30,2), 200.))
            before = evidence_hashes([source])
            rows, dense = load_observations(source)
            for zoom in [False, True]:
                fig, markers, info = draw_map(rows, dense, [], zoom)
                np.testing.assert_array_equal(markers.get_offsets(), np.array(poses)[:,:2])
                self.assertEqual(info['request_ids'], [1,2])
                self.assertEqual([r['yaw_rad'] for r in rows], [p[2] for p in poses])
                import matplotlib.pyplot as plt
                plt.close(fig)
            verify_hashes(before)


if __name__ == '__main__':
    unittest.main()
