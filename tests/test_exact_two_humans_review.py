import unittest

from tools.exact_two_humans_review import visibility_quality


class VisibilityReview(unittest.TestCase):
    def test_visible_combined_with_empty_individuals_is_unusable(self):
        total = [{'human_visible_pixel_count': 19623}]
        people = [{'people': {k: {'human_visible_pixel_count': 0} for k in ('C10', 'C24')}}]
        result = visibility_quality(total, people)
        self.assertEqual(result['status'], 'UNUSABLE')
        self.assertEqual(result['combined_individual_count_mismatch_requests'], [1])

    def test_consistent_counts_do_not_prove_identity(self):
        total = [{'human_visible_pixel_count': 100}]
        people = [{'people': {'C10': {'human_visible_pixel_count': 40}, 'C24': {'human_visible_pixel_count': 60}}}]
        result = visibility_quality(total, people)
        self.assertEqual(result['status'], 'COUNTS_CONSISTENT_ONLY')
        self.assertEqual(result['combined_individual_count_mismatch_requests'], [])
