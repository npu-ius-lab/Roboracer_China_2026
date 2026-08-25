import unittest

from f1tenth_dynamic_mpcc.timestamp_guard import benign_reorder


class TestTimestampGuard(unittest.TestCase):
    def test_sub_millisecond_pose_continuous_reorder_is_benign(self):
        self.assertTrue(benign_reorder(
            10.0, 9.9997325, 0.0, 0.0, 0.001, 0.05, 0.05
        ))

    def test_large_reorder_is_not_benign(self):
        self.assertFalse(benign_reorder(
            10.0, 9.995, 0.0, 0.0, 0.001, 0.05, 0.05
        ))

    def test_pose_discontinuity_is_not_benign(self):
        self.assertFalse(benign_reorder(
            10.0, 9.9998, 0.20, 0.0, 0.001, 0.05, 0.05
        ))

    def test_forward_sample_is_not_a_reorder(self):
        self.assertFalse(benign_reorder(
            10.0, 10.03, 0.01, 0.01, 0.001, 0.05, 0.05
        ))


if __name__ == "__main__":
    unittest.main()
