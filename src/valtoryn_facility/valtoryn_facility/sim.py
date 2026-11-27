#!/usr/bin/env python3
"""Deterministic fleet-coordination simulation over the facility lane graph.

This is a *software* simulation of the coordinator's traffic behaviour: robots
advance along their reserved lane routes at the facility speed limits, node by
node, with the clock advanced in fixed steps. It is NOT a physics simulation
and it does not exercise Gazebo, Nav2 or ros2_control; running two AMRs in the
Gazebo digital twin remains open Phase 4 work.

What it does verify is coordination logic: reservation conflicts, queueing,
speed-limit handling, comms-loss store-and-forward and order lifecycle.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .coordinator import FleetCoordinator
from .facility_model import FacilityMap


@dataclass
class Mover:
    robot_id: str
    node_ids: tuple[str, ...]
    edge_ids: tuple[str, ...]
    index: int = 0
    travelled_on_edge: float = 0.0
    distance: float = 0.0
    finished: bool = False


class FleetSimulator:
    def __init__(
        self,
        coordinator: FleetCoordinator,
        facility: FacilityMap | None = None,
        dt: float = 0.25,
    ) -> None:
        if dt <= 0:
            raise ValueError("dt must be positive")
        self.coordinator = coordinator
        self.facility = facility or coordinator.facility
        self.dt = dt
        self.time = 0.0
        self.movers: dict[str, Mover] = {}
        self.violations: list[str] = []

    # ---- bookkeeping ---------------------------------------------------
    def _sync_movers(self) -> None:
        for robot_id, route in self.coordinator.routes.items():
            if robot_id in self.movers:
                continue
            self.movers[robot_id] = Mover(
                robot_id=robot_id,
                node_ids=route,
                edge_ids=self.coordinator.route_edges[robot_id],
            )

    def _claim_violation(self, mover: Mover, edge_id: str) -> None:
        owner = self.coordinator.edge_owner.get(edge_id)
        if owner != mover.robot_id:
            self.violations.append(
                f"t={self.time:.2f} {mover.robot_id} traversing lane {edge_id!r} "
                f"held by {owner or 'nobody'}"
            )

    def _finish(self, mover: Mover) -> None:
        mover.finished = True
        self.coordinator.complete_route(mover.robot_id, self.time)

    def _step_mover(self, mover: Mover) -> None:
        if mover.finished:
            return
        if mover.index >= len(mover.node_ids) - 1:
            self._finish(mover)
            return

        edge_id = mover.edge_ids[mover.index]
        edge = self.facility.edges[edge_id]
        self._claim_violation(mover, edge_id)
        self.coordinator.advance_to(
            mover.robot_id, mover.node_ids[mover.index], self.time
        )

        length = self.facility.edge_length(edge)
        speed = self.facility.effective_speed(edge)
        advance = speed * self.dt
        mover.travelled_on_edge += advance
        mover.distance += advance
        self.coordinator.consume_distance(mover.robot_id, advance)

        if mover.travelled_on_edge >= length - 1e-9:
            mover.index += 1
            mover.travelled_on_edge = 0.0
            self.coordinator.advance_to(
                mover.robot_id, mover.node_ids[mover.index], self.time
            )
            if mover.index >= len(mover.node_ids) - 1:
                self._finish(mover)

    # ---- driving -------------------------------------------------------
    def step(self) -> None:
        self.coordinator.assign_pending(self.time)
        self._sync_movers()
        for mover in list(self.movers.values()):
            self._step_mover(mover)
        self.time += self.dt

    def run(self, max_seconds: float = 900.0) -> dict[str, Any]:
        while self.time < max_seconds:
            if not self.coordinator.unfinished_orders():
                break
            if (
                self.movers
                and not self.coordinator.pending_orders()
                and all(mover.finished for mover in self.movers.values())
            ):
                # Every robot has reached its last node; anything still open is
                # waiting for a human (verification) or a reconnecting robot, not
                # for simulation time to pass.
                break
            self.step()
        if self.coordinator.unfinished_orders() and self.time >= max_seconds:
            self.violations.append(
                f"simulation did not converge within {max_seconds:.0f}s"
            )
        return self.report()

    # ---- reporting -----------------------------------------------------
    def report(self) -> dict[str, Any]:
        snapshot = self.coordinator.snapshot()
        return {
            "sim_time": round(self.time, 3),
            "converged": not self.coordinator.unfinished_orders(),
            "violations": list(self.violations),
            "orders": snapshot["orders"],
            "robots": {
                robot_id: {
                    **state,
                    "distance": round(self.movers[robot_id].distance, 3)
                    if robot_id in self.movers
                    else 0.0,
                }
                for robot_id, state in snapshot["robots"].items()
            },
            "reservations": snapshot["reservations"],
            "events": snapshot["events"],
        }
