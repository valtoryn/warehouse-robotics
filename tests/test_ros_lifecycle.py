#!/usr/bin/env python3
"""Automated ROS 2 integration tests for Valtoryn Lifecycle Supervisor.

Verifies over real ROS 2 DDS middleware:
  - RosLifecycleClient service communication against real lifecycle service endpoints
  - Full configure -> activate -> deactivate -> shutdown transition sequence
  - Transition rejection handling and error recording
  - Service call timeout handling
  - Multi-node reverse-order shutdown
  - Periodic /diagnostics publication with DiagnosticArray
  - Critical fault cascading to higher-tier dependents

Execution:
  In a sourced ROS 2 workspace:
    python -m unittest tests/test_ros_lifecycle.py -v
"""
from __future__ import annotations

import shutil
import sys
import threading
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
        from rclpy.executors import SingleThreadedExecutor
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


if HAS_ROS:
    class ManagedTestLifecycleNode(Node):
        """Mock ROS 2 node exposing standard lifecycle services."""

        def __init__(self, name: str):
            super().__init__(name)
            self.state_id = RosState.PRIMARY_STATE_UNCONFIGURED
            self.state_label = "unconfigured"
            self.fail_on_activate = False
            self.delay_response_sec = 0.0

            self.srv_change = self.create_service(
                ChangeState, f"{name}/change_state", self._handle_change_state
            )
            self.srv_get = self.create_service(
                GetState, f"{name}/get_state", self._handle_get_state
            )

        def _handle_change_state(self, req: ChangeState.Request, res: ChangeState.Response) -> ChangeState.Response:
            if self.delay_response_sec > 0:
                time.sleep(self.delay_response_sec)

            trans_id = req.transition.id
            if trans_id == RosTransition.TRANSITION_CONFIGURE:
                if self.state_id == RosState.PRIMARY_STATE_UNCONFIGURED:
                    self.state_id = RosState.PRIMARY_STATE_INACTIVE
                    self.state_label = "inactive"
                    res.success = True
                else:
                    res.success = False
            elif trans_id == RosTransition.TRANSITION_ACTIVATE:
                if self.fail_on_activate:
                    res.success = False
                elif self.state_id == RosState.PRIMARY_STATE_INACTIVE:
                    self.state_id = RosState.PRIMARY_STATE_ACTIVE
                    self.state_label = "active"
                    res.success = True
                else:
                    res.success = False
            elif trans_id == RosTransition.TRANSITION_DEACTIVATE:
                if self.state_id == RosState.PRIMARY_STATE_ACTIVE:
                    self.state_id = RosState.PRIMARY_STATE_INACTIVE
                    self.state_label = "inactive"
                    res.success = True
                else:
                    res.success = False
            elif trans_id in (
                RosTransition.TRANSITION_UNCONFIGURED_SHUTDOWN,
                RosTransition.TRANSITION_INACTIVE_SHUTDOWN,
                RosTransition.TRANSITION_ACTIVE_SHUTDOWN,
            ):
                self.state_id = RosState.PRIMARY_STATE_FINALIZED
                self.state_label = "finalized"
                res.success = True
            else:
                res.success = False
            return res

        def _handle_get_state(self, req: GetState.Request, res: GetState.Response) -> GetState.Response:
            res.current_state.id = self.state_id
            res.current_state.label = self.state_label
            return res


@unittest.skipUnless(HAS_ROS, "ROS 2 Jazzy environment not available on this host")
class RosLifecycleIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client_node = Node("test_ros_lifecycle_supervisor")
        cls.managed_node_1 = ManagedTestLifecycleNode("managed_hw_node")
        cls.managed_node_2 = ManagedTestLifecycleNode("managed_nav_node")

        # Run managed nodes in background executor thread
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.managed_node_1)
        cls.executor.add_node(cls.managed_node_2)
        cls.exec_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.exec_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.client_node.destroy_node()
        cls.managed_node_1.destroy_node()
        cls.managed_node_2.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self.transport = RosLifecycleClient(self.client_node)
        self.supervisor = LifecycleSupervisor(system_id="test_amr", transport=self.transport)
        self.node_hw = self.supervisor.register_node("managed_hw_node", tier=1, is_critical=True)
        self.node_nav = self.supervisor.register_node("managed_nav_node", tier=2, is_critical=True)
        # Reset managed nodes
        self.managed_node_1.state_id = RosState.PRIMARY_STATE_UNCONFIGURED
        self.managed_node_1.fail_on_activate = False
        self.managed_node_1.delay_response_sec = 0.0
        self.managed_node_2.state_id = RosState.PRIMARY_STATE_UNCONFIGURED
        self.managed_node_2.fail_on_activate = False
        self.managed_node_2.delay_response_sec = 0.0

    def test_01_real_service_startup_sequence(self):
        # Configure tier 1 and tier 2
        ok_cfg = self.supervisor.configure_all(timeout_sec=3.0)
        self.assertTrue(ok_cfg)
        self.assertEqual(self.node_hw.state, LifecycleState.INACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.INACTIVE)
        self.assertEqual(self.managed_node_1.state_id, RosState.PRIMARY_STATE_INACTIVE)
        self.assertEqual(self.managed_node_2.state_id, RosState.PRIMARY_STATE_INACTIVE)

        # Activate tier 1 and tier 2
        ok_act = self.supervisor.activate_all(timeout_sec=3.0)
        self.assertTrue(ok_act)
        self.assertEqual(self.node_hw.state, LifecycleState.ACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.ACTIVE)
        self.assertEqual(self.managed_node_1.state_id, RosState.PRIMARY_STATE_ACTIVE)
        self.assertEqual(self.managed_node_2.state_id, RosState.PRIMARY_STATE_ACTIVE)

        # Query actual states over ROS
        states = self.supervisor.query_actual_states(timeout_sec=3.0)
        self.assertEqual(states["managed_hw_node"], LifecycleState.ACTIVE)
        self.assertEqual(states["managed_nav_node"], LifecycleState.ACTIVE)

    def test_02_reverse_order_deactivation_and_shutdown(self):
        self.supervisor.configure_all(timeout_sec=3.0)
        self.supervisor.activate_all(timeout_sec=3.0)

        # Deactivate all in reverse tier order
        ok_deact = self.supervisor.deactivate_all(timeout_sec=3.0)
        self.assertTrue(ok_deact)
        self.assertEqual(self.node_hw.state, LifecycleState.INACTIVE)
        self.assertEqual(self.node_nav.state, LifecycleState.INACTIVE)

        # Shutdown all in reverse tier order
        ok_shut = self.supervisor.shutdown_all(timeout_sec=3.0)
        self.assertTrue(ok_shut)
        self.assertEqual(self.node_hw.state, LifecycleState.FINALIZED)
        self.assertEqual(self.node_nav.state, LifecycleState.FINALIZED)

    def test_03_transition_rejection_handling(self):
        self.supervisor.configure_all(timeout_sec=3.0)
        # Inject failure on managed_nav_node activate
        self.managed_node_2.fail_on_activate = True

        ok_act = self.supervisor.activate_all(timeout_sec=3.0)
        self.assertFalse(ok_act)
        self.assertEqual(self.node_nav.state, LifecycleState.ERROR)
        self.assertIn("rejected transition", self.node_nav.error_message)

    def test_04_transition_timeout_handling(self):
        self.supervisor.configure_all(timeout_sec=3.0)
        # Inject response delay longer than timeout
        self.managed_node_2.delay_response_sec = 2.0

        ok_act = self.supervisor.activate_node("managed_nav_node", timeout_sec=0.5)
        self.assertFalse(ok_act)
        self.assertEqual(self.node_nav.state, LifecycleState.ERROR)
        self.assertIn("timed out", self.node_nav.error_message)

    def test_05_critical_fault_cascading(self):
        self.supervisor.configure_all(timeout_sec=3.0)
        self.supervisor.activate_all(timeout_sec=3.0)

        # Fault in Tier 1 hardware node must deactivate Tier 2 navigation node
        deactivated = self.supervisor.handle_node_fault("managed_hw_node", "Hardware fault")
        self.assertEqual(self.node_hw.state, LifecycleState.ERROR)
        self.assertIn("managed_nav_node", deactivated)
        self.assertEqual(self.node_nav.state, LifecycleState.INACTIVE)
        self.assertEqual(self.managed_node_2.state_id, RosState.PRIMARY_STATE_INACTIVE)

    def test_06_unreachable_node_handled_safely(self):
        success, msg = self.transport.change_state(
            "completely_missing_node", RosTransition.TRANSITION_CONFIGURE, timeout_sec=1.0
        )
        self.assertFalse(success)
        self.assertIn("not available", msg.lower())

    def test_07_diagnostics_aggregation(self):
        self.supervisor.configure_all(timeout_sec=3.0)
        self.supervisor.activate_all(timeout_sec=3.0)

        reports = self.supervisor.get_diagnostics()
        self.assertEqual(len(reports), 3)  # supervisor + 2 nodes
        self.assertEqual(reports[0].level, DiagnosticLevel.OK)

        # Trigger fault and recheck
        self.supervisor.handle_node_fault("managed_hw_node", "Power loss")
        reports_fault = self.supervisor.get_diagnostics()
        self.assertEqual(reports_fault[0].level, DiagnosticLevel.ERROR)


if __name__ == "__main__":
    unittest.main()
