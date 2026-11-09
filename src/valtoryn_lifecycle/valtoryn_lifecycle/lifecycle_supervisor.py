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
from typing import Dict, List, Optional


class LifecycleState(enum.IntEnum):
    UNCONFIGURED = 1
    INACTIVE = 2
    ACTIVE = 3
    FINALIZED = 4
    ERROR = 99


class DiagnosticLevel(enum.IntEnum):
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


class LifecycleSupervisor:
    """Orchestrates tiered node transitions and aggregates subsystem diagnostics."""

    def __init__(self, system_id: str = "valtoryn_ref_amr"):
        self.system_id = system_id
        self.nodes: Dict[str, ManagedNode] = {}

    def register_node(self, name: str, tier: int, is_critical: bool = True) -> ManagedNode:
        if tier < 1:
            raise ValueError(f"Tier must be >= 1, got {tier}")
        if name in self.nodes:
            raise ValueError(f"Node '{name}' is already registered")
        node = ManagedNode(name=name, tier=tier, is_critical=is_critical, last_heartbeat=time.time())
        self.nodes[name] = node
        return node

    def configure_all(self) -> bool:
        """Transitions registered nodes from UNCONFIGURED to INACTIVE in tier order."""
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier):
            node = self.nodes[name]
            if node.state == LifecycleState.UNCONFIGURED:
                node.state = LifecycleState.INACTIVE
                node.last_heartbeat = time.time()
            elif node.state in (LifecycleState.ERROR, LifecycleState.FINALIZED):
                return False
        return True

    def activate_all(self) -> bool:
        """Transitions registered nodes from INACTIVE to ACTIVE in tier order."""
        # Must be in INACTIVE state
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier):
            node = self.nodes[name]
            if node.state != LifecycleState.INACTIVE:
                return False
            node.state = LifecycleState.ACTIVE
            node.last_heartbeat = time.time()
        return True

    def deactivate_all(self) -> bool:
        """Transitions registered nodes from ACTIVE to INACTIVE in reverse tier order."""
        for name in sorted(self.nodes.keys(), key=lambda k: self.nodes[k].tier, reverse=True):
            node = self.nodes[name]
            if node.state == LifecycleState.ACTIVE:
                node.state = LifecycleState.INACTIVE
                node.last_heartbeat = time.time()
        return True

    def handle_node_fault(self, name: str, error_message: str) -> List[str]:
        """Handles a failure in a managed node.

        If the failing node is critical, all higher-tier dependent nodes are safely
        deactivated to prevent runaway or unguided operation.
        Returns the list of nodes that were deactivated.
        """
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")

        faulting_node = self.nodes[name]
        faulting_node.state = LifecycleState.ERROR
        faulting_node.error_message = error_message
        deactivated = []

        if faulting_node.is_critical:
            # Safely deactivate any node in a higher tier (or same tier if dependent)
            for node_name, node in sorted(self.nodes.items(), key=lambda item: item[1].tier, reverse=True):
                if node.tier > faulting_node.tier and node.state == LifecycleState.ACTIVE:
                    node.state = LifecycleState.INACTIVE
                    deactivated.append(node_name)

        return deactivated

    def update_heartbeat(self, name: str) -> None:
        if name not in self.nodes:
            raise KeyError(f"Unknown node '{name}'")
        self.nodes[name].last_heartbeat = time.time()

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

            reports.append(DiagnosticStatusReport(
                name=f"lifecycle/{name}",
                level=level,
                message=msg,
                hardware_id=self.system_id,
                values=values,
            ))

        # Overall supervisor diagnostic
        supervisor_level = DiagnosticLevel.ERROR if any_error else DiagnosticLevel.OK
        supervisor_msg = "Subsystems operating normally" if not any_error else "Subsystem fault or stale heartbeat detected"
        reports.insert(0, DiagnosticStatusReport(
            name="lifecycle/supervisor",
            level=supervisor_level,
            message=supervisor_msg,
            hardware_id=self.system_id,
            values={"registered_nodes": str(len(self.nodes))},
        ))

        return reports


def main() -> int:
    print("Valtoryn Lifecycle Supervisor daemon placeholder.")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
