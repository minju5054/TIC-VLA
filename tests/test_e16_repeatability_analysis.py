"""Pure saved-data diagnostics, separate from experimental control."""
import json
import unittest
import numpy as np
from research.analyze_e16_repeatability import persistent_divergence,serial,stats


class DiagnosticTests(unittest.TestCase):
    def test_persistent_two_bin_rule_and_missing_data(self):
        def data(signs):return dict(bins=[dict(low_y_m=i+8,count=int(s is not None),median_w=s,median_lookahead_left=s) for i,s in enumerate(signs)])
        tol=dict(command_sign_tolerance_rad_s=1e-6,position_sign_tolerance_m=1e-6)
        self.assertEqual(persistent_divergence(data([1,1,1]),data([1,-1,-1]),tol),9)
        self.assertIsNone(persistent_divergence(data([1,1,1]),data([-1,None,-1]),tol))
        self.assertIsNone(persistent_divergence(data([1,1,1]),data([1,1,-1]),tol))

    def test_serialization_and_empty_distribution(self):
        result=serial(dict(flag=np.bool_(True),value=np.float32(2),array=np.array([1,2])))
        self.assertEqual(json.loads(json.dumps(result,allow_nan=False)),dict(flag=True,value=2.,array=[1,2]))
        self.assertIsNone(stats([])['median']);self.assertEqual(stats([1,3])['median'],2.)


if __name__=='__main__':unittest.main()
