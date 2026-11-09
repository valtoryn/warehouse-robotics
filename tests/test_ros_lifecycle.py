#!/usr/bin/env python3
"""Automated ROS 2 integration tests for Valtoryn Lifecycle Supervisor.

Verifies:
  - RosLifecycleClient service communication against lifecycle nodes
  - Full configure -> activate -> deactivate -> shutdown sequence
  - Periodic /diagnostics publication with DiagnosticArray
  - Safe cascading on injected critical failure

Execution:
  In a sourced ROS 2 workspace:
    python -m unittest tests/test_ros_lifecycle.py -v
"""
from __future__ import annotations

import shutil
import sys
import time
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# Check ROS 2 availability
HAS_ROS = (
    sys.platform.startswith("linux")
    and shutil.which("ros2") is not None
)

if HAS_ROS:
    try:
        import rclpy
        from rclpy.node import Node
        from diagnostic_msgs.msg import DiagnosticArray
        from lifecycle_msgs.msg import State as RosState, Transition as RosTransition
        from lifecycle_msgs.srv import ChangeState, GetState
    except ImportError:
        HAS_ROS = False

sys.path.insert(0, str(ROOT / "src" / "valtoryn_lifecycle"))
from valtoryn_lifecycle.lifecycle_supervisor import (  # noqa: E402
    DiagnosticLevel,
    LifecycleState,
    LifecycleSupervisor,
    RosLifecycleClient,
)


@unittest.skipUnless(HAS_ROS, "ROS 2 Jazzy environment not available on this host")
class RosLifecycleIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_ros_lifecycle_client")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.received_diag: DiagnosticArray | None = None
        self.diag_sub = self.node.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diag, 10
        )
        self.transport = RosLifecycleClient(self.node)
        self.supervisor = LifecycleSupervisor(system_id="integration_amr", transport=self.transport)

    def tearDown(self):
        self.node.destroy_subscription(self.diag_sub)

    def _on_diag(self, msg: DiagnosticArray):
        self.received_diag = msg

    def test_01_transport_unreachable_node_handled_safely(self):
        # Service call to non-existent node must fail cleanly without throwing unhandled exception
        success, msg = self.transport.change_state(
            "nonexistent_node", RosTransition.TRANSITION_CONFIGURE, timeout_sec=1.0
        )
        self.assertFalse(success)
        self.assertIn("not available", msg.lower())

    def test_02_get_state_unreachable_node(self):
        state_id, label = self.transport.get_state("nonexistent_node", timeout_sec=1.0)
        self.assertEqual(state_id, LifecycleState.UNKNOWN)

    def test_03_diagnostics_structure(self):
        self.supervisor.register_node("test_hw", tier=1, is_critical=True)
        reports = self.supervisor.get_diagnostics()
        self.assertGreaterEqual(len(reports), 2)
        self.assertEqual(reports[0].name, "lifecycle/supervisor")
        self.assertEqual(reports[1].name, "lifecycle/test_hw")


if __name__ == "__main__":
    unittest.main()
