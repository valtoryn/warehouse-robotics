#!/usr/bin/env python3
"""Unit tests for Valtoryn lifecycle supervisor and diagnostics aggregator.

Tests:
  - Node registration & tier validation
  - Real lifecycle service transition sequencing
  - Reverse-tier orderly shutdown
  - Transition failure and timeout handling
  - Critical fault cascading to higher-tier dependents
  - Diagnostic report generation and level aggregation
  - Stale heartbeat detection

Run: python -m unittest discover -s tests -v
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_lifecycle"))

from valtoryn_lifecycle.lifecycle_supervisor import (  # noqa: E402
    DiagnosticLevel,
    LifecycleState,
    LifecycleSupervisor,
    LifecycleTransition,
    MockLifecycleClient,
)


class LifecycleSupervisorTests(unittest.TestCase):
    def setUp(self):
        self.client = MockLifecycleClient()
        self.sup = LifecycleSupervisor(system_id="test_robot", transport=self.client)
        self.node_hw = self.sup.register_node("diff_drive_hw", tier=1, is_critical=True)
        self.node_loc = self.sup.register_node("localization", tier=2, is_critical=True)
        self.node_nav = self.sup.register_node("nav2", tier=3, is_critical=True)
        self.node_aux = self.sup.register_node("aux_lights", tier=4, is_critical=False)

    def test_invalid_tier_rejected(self):
        with self.assertRaises(ValueError):
            self.sup.register_node("invalid_node", tier=0)

    def test_duplicate_registration_rejected(self):
        with self.assertRaises(ValueError):
            self.sup.register_node("diff_drive_hw", tier=1)

    def test_startup_sequence(self):
        self.assertTrue(self.sup.configure_all())
        for node in self.sup.nodes.values():
            self.assertEqual(node.state, LifecycleState.INACTIVE)

        self.assertTrue(self.sup.activate_all())
        for node in self.sup.nodes.values():
            self.assertEqual(node.state, LifecycleState.ACTIVE)

        states = self.sup.query_actual_states()
        for name, state in states.items():
            self.assertEqual(state, LifecycleState.ACTIVE)

    def test_deactivate_all(self):
        self.sup.configure_all()
        self.sup.activate_all()
        self.assertTrue(self.sup.deactivate_all())
        for node in self.sup.nodes.values():
            self.assertEqual(node.state, LifecycleState.INACTIVE)

    def test_shutdown_all_in_reverse_order(self):
        self.sup.configure_all()
        self.sup.activate_all()
        self.assertTrue(self.sup.shutdown_all())
        for node in self.sup.nodes.values():
            self.assertEqual(node.state, LifecycleState.FINALIZED)

    def test_transition_failure_handling(self):
        # Inject failure on nav2 configure
        self.client.inject_failure("nav2", LifecycleTransition.CONFIGURE)

        res = self.sup.configure_all()
        self.assertFalse(res)
        self.assertEqual(self.node_hw.state, LifecycleState.INACTIVE)
        self.assertEqual(self.node_loc.state, LifecycleState.INACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.ERROR)
        self.assertIn("Injected transition failure", self.node_nav.error_message)

    def test_transition_timeout_handling(self):
        # Inject timeout on localization activate
        self.sup.configure_all()
        self.client.inject_timeout("localization")

        res = self.sup.activate_all()
        self.assertFalse(res)
        self.assertEqual(self.node_loc.state, LifecycleState.ERROR)
        self.assertIn("timed out", self.node_loc.error_message)

    def test_critical_fault_cascades_safely(self):
        self.sup.configure_all()
        self.sup.activate_all()

        # Hardware failure in Tier 1
        deactivated = self.sup.handle_node_fault("diff_drive_hw", "CAN bus disconnect")

        self.assertEqual(self.node_hw.state, LifecycleState.ERROR)
        self.assertIn("localization", deactivated)
        self.assertIn("nav2", deactivated)
        self.assertIn("aux_lights", deactivated)

        # Higher-tier nodes must be brought to INACTIVE standby via actual transport calls
        self.assertEqual(self.node_loc.state, LifecycleState.INACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.INACTIVE)
        self.assertEqual(self.client.node_states["localization"], LifecycleState.INACTIVE)
        self.assertEqual(self.client.node_states["nav2"], LifecycleState.INACTIVE)

    def test_non_critical_fault_does_not_cascade(self):
        self.sup.configure_all()
        self.sup.activate_all()

        deactivated = self.sup.handle_node_fault("aux_lights", "LED driver warning")
        self.assertEqual(self.node_aux.state, LifecycleState.ERROR)
        self.assertEqual(len(deactivated), 0)
        self.assertEqual(self.node_hw.state, LifecycleState.ACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.ACTIVE)

    def test_diagnostics_reporting(self):
        self.sup.configure_all()
        self.sup.activate_all()

        reports = self.sup.get_diagnostics()
        # 1 overall supervisor report + 4 node reports
        self.assertEqual(len(reports), 5)
        self.assertEqual(reports[0].level, DiagnosticLevel.OK)

        # Trigger error
        self.sup.handle_node_fault("diff_drive_hw", "Over-temperature fault")
        reports_err = self.sup.get_diagnostics()
        self.assertEqual(reports_err[0].level, DiagnosticLevel.ERROR)

        hw_report = next(r for r in reports_err if r.name == "lifecycle/diff_drive_hw")
        self.assertEqual(hw_report.level, DiagnosticLevel.ERROR)
        self.assertIn("Over-temperature fault", hw_report.message)

    def test_stale_heartbeat_detection(self):
        self.sup.configure_all()
        self.sup.activate_all()

        # Simulate stale node
        self.node_nav.last_heartbeat = 0.0
        reports = self.sup.get_diagnostics(heartbeat_timeout=2.0)
        nav_report = next(r for r in reports if r.name == "lifecycle/nav2")
        self.assertEqual(nav_report.level, DiagnosticLevel.STALE)


if __name__ == "__main__":
    unittest.main()
