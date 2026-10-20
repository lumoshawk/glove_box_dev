"""Offline checks for the enable_test helpers; no ROS node or hardware connection is created."""
import unittest
from types import SimpleNamespace

from glove_box_bringup.enable_test import drift_deg, next_control_word

CW = SimpleNamespace(CW_SHUTDOWN=0x06, CW_SWITCH_ON=0x07, CW_ENABLE_OP=0x0F, CW_FAULT_RESET=0x80)


class EnableTestHelpers(unittest.TestCase):
    def test_state_machine_walks_to_operation_enabled(self):
        for status_word, expected in ((0x0250, 0x06), (0x0231, 0x07), (0x0233, 0x0F), (0x0237, 0x0F)):
            with self.subTest(status_word=status_word):
                self.assertEqual(next_control_word(CW, status_word), expected)

    def test_fault_is_reset_first(self):
        self.assertEqual(next_control_word(CW, 0x0208), 0x80)
        self.assertEqual(next_control_word(CW, 0x0218), 0x80)

    def test_drift_deg(self):
        self.assertEqual(drift_deg(100, 100, 1000), 0)
        self.assertAlmostEqual(drift_deg(90, 100, 3600), 1.0)
        self.assertAlmostEqual(drift_deg(110, 100, 3600), 1.0)


if __name__ == '__main__':
    unittest.main()
