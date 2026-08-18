#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_SRC = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src"))
if PACKAGE_SRC not in sys.path:
    sys.path.insert(0, PACKAGE_SRC)

from purepursuit_mgeo.merge_gate import MergeDecisionGate


class MergeDecisionGateTest(unittest.TestCase):
    def test_waits_fail_safe_before_first_message(self):
        gate = MergeDecisionGate("left", 0.5)
        self.assertEqual(
            gate.stop_required(10.0),
            (True, "waiting_for_merge_status"),
        )

    def test_selected_side_controls_permission(self):
        gate = MergeDecisionGate("left", 0.5)
        gate.update(True, True, False, 10.0)
        self.assertEqual(gate.stop_required(10.1), (False, "merge_available"))

        gate.update(True, False, True, 10.2)
        self.assertEqual(gate.stop_required(10.3), (True, "merge_blocked"))

    def test_either_accepts_one_available_side(self):
        gate = MergeDecisionGate("either", 0.5)
        gate.update(True, False, True, 10.0)
        self.assertEqual(gate.stop_required(10.1), (False, "merge_available"))

    def test_invalid_and_stale_messages_stop(self):
        gate = MergeDecisionGate("right", 0.5)
        gate.update(False, True, True, 10.0)
        self.assertEqual(gate.stop_required(10.1), (True, "merge_status_invalid"))

        gate.update(True, True, True, 10.0)
        self.assertEqual(gate.stop_required(10.6), (True, "merge_status_stale"))

    def test_rejects_invalid_configuration(self):
        with self.assertRaises(ValueError):
            MergeDecisionGate("center", 0.5)
        with self.assertRaises(ValueError):
            MergeDecisionGate("left", 0.0)


if __name__ == "__main__":
    unittest.main()
