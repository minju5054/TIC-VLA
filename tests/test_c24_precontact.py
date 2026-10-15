"""Synthetic geometry checks only, never evidence of an actual safe route."""
import unittest
import numpy as np
from tools.e16_wheelchair_delay.c24_precontact import assess_pair, POLICY
from research.audit_e16_wheelchair_handoffs import proxy_clearance, is_clear


def clearance(value):
    return dict(valid=True,certified_lower_m=value,proxy_overlap_observed=value<0)


class C24QualificationTests(unittest.TestCase):
    def test_geometrically_clear_candidate_does_not_certify_execution(self):
        r=assess_pair(clearance(-.1),clearance(.2),clearance(.2),clearance(.2),{'status':'CROSSES'})
        self.assertTrue(r['old_conflict_to_fresh_clear'])
        self.assertTrue(r['connector_and_boundary_clear'])
        self.assertEqual(r['physical_feasibility'],'NOT_ESTABLISHED_BY_SAVED_POLYLINES')

    def test_short_horizon_not_claimed_as_bypass(self):
        r=assess_pair(clearance(-.1),clearance(.2),clearance(.2),clearance(.2),{'status':'NO_PREDICTED_CROSSING'})
        self.assertFalse(r['old_conflict_to_fresh_clear'])

    def test_safe_fresh_does_not_certify_unsafe_connector(self):
        r=assess_pair(clearance(-.1),clearance(.2),clearance(-.1),clearance(.2),{'status':'CROSSES'})
        self.assertTrue(r['old_conflict_to_fresh_clear'])
        self.assertFalse(r['connector_and_boundary_clear'])

    def test_clear_centerline_not_clear_robot_footprint(self):
        boxes=[dict(name='synthetic chair',center=np.array([0.,0.]),half=np.array([.5,.4]),yaw=0.)]
        line=np.array([[-2.,.6],[2.,.6]])
        self.assertTrue(is_clear(proxy_clearance(line,boxes,0.,.01),POLICY))
        self.assertFalse(is_clear(proxy_clearance(line,boxes,.61,.01),POLICY))


if __name__=='__main__':unittest.main()
