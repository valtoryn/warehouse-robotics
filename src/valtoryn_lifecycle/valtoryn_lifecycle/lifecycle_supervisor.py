"""Valtoryn subsystem lifecycle manager and diagnostics supervisor.

Implements the Tier 0 / Tier 1 lifecycle state machine and diagnostic
reporting per docs/architecture/SYSTEM_ARCHITECTURE.md sections 6 and 3.

Lifecycle State Machine:
  UNCONFIGURED (1) -> INACTIVE (2) -> ACTIVE (3)
       ^                    |             |
       |                    v             v
       +----------------- FINALIZED (4) <-+
                            |
                         ERROR (99)

Subsystem Sequencing Tiers:
  Tier 1: Drivetrain and hardware abstraction (controller_manager, diff_drive_controller)
  Tier 2: Localization (robot_localization, slam_toolbox)
  Tier 3: Navigation (Nav2 planner_server, controller_server, bt_navigator)
  Tier 4: Task Execution (valtoryn_task_bt)
"""
from __future__ import annotations

import enum
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    import rclpy
    from rclpy.node import Node as RosNode
    from lifecycle_msgs.msg import State as RosState, Transition as RosTransition
    from lifecycle_msgs.srv import ChangeState, GetState
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False
    RosNode = object  # type: ignore


class LifecycleState(enum.IntEnum):
    """Lifecycle primary states per lifecycle_msgs/msg/State."""
    UNKNOWN = 0
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4
    ERROR = 99


class LifecycleTransition(enum.IntEnum):
    """Lifecycle transition triggers per lifecycle_msgs/msg/Transition."""
    CREATE = 0
    CONFIGURE = 1
    CLEANUP = 2
    ACTIVATE = 3
    DEACTIVATE = 4
    UNCONFIGURED_SHUTDOWN = 5
    INACTIVE_SHUTDOWN = 6
    ACTIVE_SHUTDOWN = 7
    DESTROY = 8


class DiagnosticLevel(enum.IntEnum):
    """Diagnostic severity levels per diagnostic_msgs/msg/DiagnosticStatus."""
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3


@dataclass
class DiagnosticStatusReport:
    name: str
    level: DiagnosticLevel
    message: str
    hardware_id: str = "valtoryn_amr"
    values: Dict[str, str] = field(default_factory=dict)


@dataclass
class ManagedNode:
    name: str
    tier: int
    is_critical: bool = True
    state: LifecycleState = LifecycleState.UNCONFIGURED
    last_heartbeat: float = 0.0
    error_message: str = ""


class LifecycleClientTransport:
    """Abstract transport interface for node lifecycle transitions."""

    def change_state(
        self, node_name: str, transition_id: int, timeout_sec: float = 5.0
    ) -> Tuple[bool, str]:
        raise NotImplementedError

    def get_state(self, node_name: str, timeout_sec: float = 3.0) -> Tuple[int, str]:
        raise NotImplementedError


class MockLifecycleClient(LifecycleClientTransport):
    """Test transport providing deterministic simulation of node states and service failures."""

    def __init__(self) -> None:
        self.node_states: Dict[str, int] = {}
        self.fail_transitions: Dict[str, set[int]] = {}
        self.timeout_nodes: set[str] = set()

    def set_node_state(self, node_name: str, state_id: int) -> None:
        self.node_states[node_name] = state_id

    def inject_failure(self, node_name: str, transition_id: Optional[int] = None) -> None:
        if transition_id is None:
            self.fail_transitions.setdefault(node_name, set()).update(
                [
                    LifecycleTransition.CONFIGURE,
                    LifecycleTransition.ACTIVATE,
                    LifecycleTransition.DEACTIVATE,
                    LifecycleTransition.ACTIVE_SHUTDOWN,
                    LifecycleTransition.INACTIVE_SHUTDOWN,
                ]
            )
        else:
            self.fail_transitions.setdefault(node_name, set()).add(transition_id)

    def inject_timeout(self, node_name: str) -> None:
        self.timeout_nodes.add(node_name)

    def clear_faults(self, node_name: Optional[str] = None) -> None:
        if node_name:
            self.fail_transitions.pop(node_name, None)
            self.timeout_nodes.discard(node_name)
        else:
            self.fail_transitions.clear()
            self.timeout_nodes.clear()

    def change_state(
        self, node_name: str, transition_id: int, timeout_sec: float = 5.0
    ) -> Tuple[bool, str]:
        if node_name in self.timeout_nodes:
            return False, f"Call to '{node_name}/change_state' timed out after {timeout_sec}s"

        if transition_id in self.fail_transitions.get(node_name, set()):
            return False, f"Injected transition failure on '{node_name}' for transition {transition_id}"

        current = self.node_states.get(node_name, LifecycleState.UNCONFIGURED)

        if transition_id == LifecycleTransition.CONFIGURE:
            if current == LifecycleState.UNCONFIGURED:
                self.node_states[node_name] = LifecycleState.INACTIVE
                return True, "Transition to INACTIVE succeeded"
            return False, f"Invalid state for CONFIGURE: current state {current}"

        elif transition_id == LifecycleTransition.ACTIVATE:
            if current == LifecycleState.INACTIVE:
                self.node_states[node_name] = LifecycleState.ACTIVE
                return True, "Transition to ACTIVE succeeded"
            return False, f"Invalid state for ACTIVATE: current state {current}"

        elif transition_id == LifecycleTransition.DEACTIVATE:
            if current == LifecycleState.ACTIVE:
                self.node_states[node_name] = LifecycleState.INACTIVE
                return True, "Transition to INACTIVE succeeded"
            return False, f"Invalid state for DEACTIVATE: current state {current}"

        elif transition_id in (
            LifecycleTransition.ACTIVE_SHUTDOWN,
            LifecycleTransition.INACTIVE_SHUTDOWN,
            LifecycleTransition.UNCONFIGURED_SHUTDOWN,
        ):
            self.node_states[node_name] = LifecycleState.FINALIZED
            return True, "Transition to FINALIZED succeeded"

        return False, f"Unsupported transition {transition_id}"

    def get_state(self, node_name: str, timeout_sec: float = 3.0) -> Tuple[int, str]:
        if node_name in self.timeout_nodes:
            return LifecycleState.UNKNOWN, "Timed out"
        state = self.node_states.get(node_name, LifecycleState.UNCONFIGURED)
        labels = {
            LifecycleState.UNCONFIGURED: "unconfigured",
            LifecycleState.INACTIVE: "inactive",
            LifecycleState.ACTIVE: "active",
            LifecycleState.FINALIZED: "finalized",
            LifecycleState.ERROR: "error",
        }
        return state, labels.get(state, "unknown")


class RosLifecycleClient(LifecycleClientTransport):
    """Real ROS 2 transport calling lifecycle_msgs ChangeState and GetState services."""

    def __init__(self, ros_node: Any) -> None:
        if not HAS_RCLPY:
            raise RuntimeError("rclpy is not available in the current environment")
        self.ros_node = ros_node
        self._change_clients: Dict[str, Any] = {}
        self._state_clients: Dict[str, Any] = {}

    def _get_change_client(self, node_name: str) -> Any:
        if node_name not in self._change_clients:
            srv_name = f"{node_name}/change_state"
            self._change_clients[node_name] = self.ros_node.create_client(ChangeState, srv_name)
        return self._change_clients[node_name]

    def _get_state_client(self, node_name: str) -> Any:
        if node_name not in self._state_clients:
            srv_name = f"{node_name}/get_state"
            self._state_clients[node_name] = self.ros_node.create_client(GetState, srv_name)
        return self._state_clients[node_name]

    def change_state(
        self, node_name: str, transition_id: int, timeout_sec: float = 5.0
    ) -> Tuple[bool, str]:
        client = self._get_change_client(node_name)
        if not client.wait_for_service(timeout_sec=min(timeout_sec, 2.0)):
            return False, f"Service '{node_name}/change_state' is not available"

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        # Spin until complete with timeout
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                return False, f"Call to '{node_name}/change_state' timed out after {timeout_sec}s"

        if future.done():
            try:
                res = future.result()
                if res.success:
                    return True, "Transition succeeded"
                return False, f"Node '{node_name}' rejected transition {transition_id}"
            except Exception as exc:
                return False, f"Service call exception on '{node_name}': {exc}"
        return False, "Interrupted"

    def get_state(self, node_name: str, timeout_sec: float = 3.0) -> Tuple[int, str]:
        client = self._get_state_client(node_name)
        if not client.wait_for_service(timeout_sec=min(timeout_sec, 2.0)):
            return LifecycleState.UNKNOWN, "Service unavailable"

        req = GetState.Request()
        future = client.call_async(req)
        start = time.time()
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self.ros_node, timeout_sec=0.1)
            if time.time() - start > timeout_sec:
                return LifecycleState.UNKNOWN, "Timed out"

        if future.done():
            try:
                res = future.result()
                return res.current_state.id, res.current_state.label
            except Exception:
                return LifecycleState.UNKNOWN, "Service error"
        return LifecycleState.UNKNOWN, "Interrupted"


class LifecycleSupervisor:
    """Orchestrates tiered node transitions and aggregates subsystem diagnostics."""

    def __init__(
        self,
        system_id: str = "valtoryn_ref_amr",
        transport: Optional[LifecycleClientTransport] = None,
    ) -> None:
        self.system_id = system_id
        self.transport = transport or MockLifecycleClient()
        self.nodes: Dict[str, ManagedNode] = {}

    def register_node(self, name: str, tier: int, is_critical: bool = True) -> ManagedNode:
        if tier < 1:
            raise ValueError(f"Tier must be >= 1, got {tier}")
        if name in self.nodes:
            raise ValueError(f"Node '{name}' is already registered")
        node = ManagedNode(name=name, tier=tier, is_critical=is_critical, last_heartbeat=time.time())
        self.nodes[name] = node
        return node

    def configure_node(self, name: str, timeout_sec: float = 5.0) -> bool:
        """Transitions a single node from UNCONFIGURED to INACTIVE via ROS service."""
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        node = self.nodes[name]
        success, msg = self.transport.change_state(
            name, LifecycleTransition.CONFIGURE, timeout_sec=timeout_sec
        )
        if success:
            node.state = LifecycleState.INACTIVE
            node.error_message = ""
            node.last_heartbeat = time.time()
            return True
        else:
            node.state = LifecycleState.ERROR
            node.error_message = msg
            return False

    def activate_node(self, name: str, timeout_sec: float = 5.0) -> bool:
        """Transitions a single node from INACTIVE to ACTIVE via ROS service."""
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        node = self.nodes[name]
        success, msg = self.transport.change_state(
            name, LifecycleTransition.ACTIVATE, timeout_sec=timeout_sec
        )
        if success:
            node.state = LifecycleState.ACTIVE
            node.error_message = ""
            node.last_heartbeat = time.time()
            return True
        else:
            node.state = LifecycleState.ERROR
            node.error_message = msg
            return False

    def deactivate_node(self, name: str, timeout_sec: float = 5.0) -> bool:
        """Transitions a single node from ACTIVE to INACTIVE via ROS service."""
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        node = self.nodes[name]
        success, msg = self.transport.change_state(
            name, LifecycleTransition.DEACTIVATE, timeout_sec=timeout_sec
        )
        if success:
            node.state = LifecycleState.INACTIVE
            node.error_message = ""
            node.last_heartbeat = time.time()
            return True
        else:
            node.state = LifecycleState.ERROR
            node.error_message = msg
            return False

    def shutdown_node(self, name: str, timeout_sec: float = 5.0) -> bool:
        """Transitions a node to FINALIZED via appropriate shutdown transition."""
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        node = self.nodes[name]
        if node.state == LifecycleState.ACTIVE:
            trans = LifecycleTransition.ACTIVE_SHUTDOWN
        elif node.state == LifecycleState.INACTIVE:
            trans = LifecycleTransition.INACTIVE_SHUTDOWN
        else:
            trans = LifecycleTransition.UNCONFIGURED_SHUTDOWN

        success, msg = self.transport.change_state(name, trans, timeout_sec=timeout_sec)
        if success:
            node.state = LifecycleState.FINALIZED
            node.error_message = ""
            node.last_heartbeat = time.time()
            return True
        else:
            node.state = LifecycleState.ERROR
            node.error_message = msg
            return False

    def configure_all(self, timeout_sec: float = 5.0) -> bool:
        """Transitions registered nodes from UNCONFIGURED to INACTIVE in tier order."""
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier):
            node = self.nodes[name]
            if node.state == LifecycleState.UNCONFIGURED:
                if not self.configure_node(name, timeout_sec=timeout_sec):
                    return False
            elif node.state in (LifecycleState.ERROR, LifecycleState.FINALIZED):
                return False
        return True

    def activate_all(self, timeout_sec: float = 5.0) -> bool:
        """Transitions registered nodes from INACTIVE to ACTIVE in tier order."""
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier):
            node = self.nodes[name]
            if node.state != LifecycleState.INACTIVE:
                return False
            if not self.activate_node(name, timeout_sec=timeout_sec):
                # Critical activation failure triggers safe rollback
                self.handle_node_fault(name, node.error_message)
                return False
        return True

    def deactivate_all(self, timeout_sec: float = 5.0) -> bool:
        """Transitions registered nodes from ACTIVE to INACTIVE in REVERSE tier order."""
        all_ok = True
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier, reverse=True):
            node = self.nodes[name]
            if node.state == LifecycleState.ACTIVE:
                if not self.deactivate_node(name, timeout_sec=timeout_sec):
                    all_ok = False
        return all_ok

    def shutdown_all(self, timeout_sec: float = 5.0) -> bool:
        """Transitions all registered nodes to FINALIZED in REVERSE tier order."""
        all_ok = True
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier, reverse=True):
            node = self.nodes[name]
            if node.state != LifecycleState.FINALIZED:
                if not self.shutdown_node(name, timeout_sec=timeout_sec):
                    all_ok = False
        return all_ok

    def handle_node_fault(self, name: str, error_message: str) -> List[str]:
        """Handles a failure in a managed node.

        If the failing node is critical, all higher-tier dependent nodes are safely
        deactivated via real ROS service calls in reverse tier order.
        Returns the list of node names that were deactivated.
        """
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")

        faulting_node = self.nodes[name]
        faulting_node.state = LifecycleState.ERROR
        faulting_node.error_message = error_message
        deactivated = []

        if faulting_node.is_critical:
            # Safely deactivate any active node in a higher tier (reverse tier order)
            for node_name, node in sorted(self.nodes.items(), key=lambda item: item[1].tier, reverse=True):
                if node.tier > faulting_node.tier and node.state == LifecycleState.ACTIVE:
                    if self.deactivate_node(node_name):
                        deactivated.append(node_name)

        return deactivated

    def update_heartbeat(self, name: str) -> None:
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        self.nodes[name].last_heartbeat = time.time()

    def query_actual_states(self, timeout_sec: float = 3.0) -> Dict[str, LifecycleState]:
        """Queries actual node states from the ROS lifecycle services."""
        results = {}
        for name, node in self.nodes.items():
            state_id, _ = self.transport.get_state(name, timeout_sec=timeout_sec)
            try:
                results[name] = LifecycleState(state_id)
            except ValueError:
                results[name] = LifecycleState.UNKNOWN
        return results

    def get_diagnostics(self, heartbeat_timeout: float = 5.0) -> List[DiagnosticStatusReport]:
        """Generates structured diagnostics for all subsystems."""
        reports = []
        now = time.time()
        any_error = False

        for name, node in sorted(self.nodes.items(), key=lambda item: item[1].tier):
            time_since = now - node.last_heartbeat
            values = {
                "tier": str(node.tier),
                "state": node.state.name,
                "critical": str(node.is_critical),
                "last_heartbeat_age_sec": f"{time_since:.2f}",
            }

            if node.state == LifecycleState.ERROR:
                level = DiagnosticLevel.ERROR
                msg = f"Node error: {node.error_message}"
                any_error = True
            elif time_since > heartbeat_timeout and node.state == LifecycleState.ACTIVE:
                level = DiagnosticLevel.STALE
                msg = f"Heartbeat timed out ({time_since:.1f}s > {heartbeat_timeout}s)"
                any_error = True
            elif node.state == LifecycleState.ACTIVE:
                level = DiagnosticLevel.OK
                msg = "Node active and healthy"
            elif node.state == LifecycleState.INACTIVE:
                level = DiagnosticLevel.WARN
                msg = "Node inactive (standby)"
            else:
                level = DiagnosticLevel.WARN
                msg = f"Node {node.state.name}"

            reports.append(
                DiagnosticStatusReport(
                    name=f"lifecycle/{name}",
                    level=level,
                    message=msg,
                    hardware_id=self.system_id,
                    values=values,
                )
            )

        # Overall supervisor diagnostic
        supervisor_level = DiagnosticLevel.ERROR if any_error else DiagnosticLevel.OK
        supervisor_msg = (
            "Subsystems operating normally"
            if not any_error
            else "Subsystem fault or stale heartbeat detected"
        )
        reports.insert(
            0,
            DiagnosticStatusReport(
                name="lifecycle/supervisor",
                level=supervisor_level,
                message=supervisor_msg,
                hardware_id=self.system_id,
                values={"registered_nodes": str(len(self.nodes))},
            ),
        )

        return reports


if HAS_RCLPY:
    class RosLifecycleSupervisorNode(RosNode):
        """ROS 2 Node providing lifecycle supervision daemon and diagnostics publishing."""

        def __init__(self) -> None:
            super().__init__("lifecycle_supervisor")
            self.declare_parameter("system_id", "valtoryn_ref_amr")
            self.declare_parameter("publish_rate", 1.0)
            self.declare_parameter("heartbeat_timeout", 5.0)

            system_id = self.get_parameter("system_id").value
            publish_rate = float(self.get_parameter("publish_rate").value)
            self.heartbeat_timeout = float(self.get_parameter("heartbeat_timeout").value)

            self.transport = RosLifecycleClient(self)
            self.supervisor = LifecycleSupervisor(system_id=system_id, transport=self.transport)

            # Register reference AMR default subsystem nodes
            self.supervisor.register_node("controller_manager", tier=1, is_critical=True)
            self.supervisor.register_node("diff_drive_controller", tier=1, is_critical=True)
            self.supervisor.register_node("robot_state_publisher", tier=1, is_critical=False)
            self.supervisor.register_node("robot_localization", tier=2, is_critical=True)
            self.supervisor.register_node("bt_navigator", tier=3, is_critical=True)

            self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
            self.timer = self.create_timer(1.0 / max(publish_rate, 0.1), self._on_timer)

            self.get_logger().info(
                f"Valtoryn Lifecycle Supervisor started for system '{system_id}' "
                f"with {len(self.supervisor.nodes)} managed nodes"
            )

        def _on_timer(self) -> None:
            reports = self.supervisor.get_diagnostics(heartbeat_timeout=self.heartbeat_timeout)
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()

            for r in reports:
                status = DiagnosticStatus()
                status.name = r.name
                status.level = int(r.level)
                status.message = r.message
                status.hardware_id = r.hardware_id
                for k, v in r.values.items():
                    status.values.append(KeyValue(key=k, value=v))
                diag_array.status.append(status)

            self.diag_pub.publish(diag_array)


def main(args: Optional[List[str]] = None) -> int:
    if not HAS_RCLPY:
        print("Valtoryn Lifecycle Supervisor: rclpy not installed in environment. Daemon cannot run.")
        return 1

    rclpy.init(args=args)
    node = RosLifecycleSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
