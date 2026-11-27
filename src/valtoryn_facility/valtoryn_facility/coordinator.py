#!/usr/bin/env python3
"""Minimal reservation/queue fleet coordinator baseline (FR-F4, ADR-008).

Coordination model implemented here:

* **Queueing.** Orders arrive, are dispatched by capability matching, and only
  start when their whole route can be reserved.
* **Reservation.** A robot reserves every node and every lane of its route
  atomically: all-or-nothing. It then releases each node/lane as it advances.
  Because a robot never holds reservations while waiting for more, the baseline
  cannot deadlock; the cost is reduced concurrency on shared aisles. This is
  the documented "minimal baseline", not the full negotiation/RMF behaviour.
* **Order lifecycle.** PENDING -> IN_PROGRESS -> COMPLETED, with REJECTED for
  default-deny/invalid orders, FAILED for a confirmed failed execution, and
  NEEDS_VERIFICATION for an ambiguous pick/drop outcome (FR-F6). Verification
  is never guessed: an ambiguous outcome parks the order until an operator
  resolves it.
* **Idempotency.** Re-submitting a request with the same idempotency key
  returns the existing order and never creates a duplicate (FR-F6).
* **Comms loss.** The whole route is released to the robot at assignment, so a
  robot that loses connectivity finishes its released base and reports on
  reconnect (FR-F3 store-and-forward). The order is only marked COMPLETED
  after that reconciliation, and it records that comms were lost.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

from .dispatch import (
    DEFAULT_MIN_BATTERY,
    Assignment,
    OrderRequest,
    Rejection,
    RobotState,
    select_robot,
)
from .facility_model import FacilityMap


class OrderState(str, Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    NEEDS_VERIFICATION = "NEEDS_VERIFICATION"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    REJECTED = "REJECTED"


TERMINAL_STATES = {
    OrderState.COMPLETED,
    OrderState.FAILED,
    OrderState.REJECTED,
}


@dataclass
class Order:
    order_id: str
    idempotency_key: str
    request: OrderRequest
    state: OrderState = OrderState.PENDING
    robot_id: str | None = None
    route_nodes: tuple[str, ...] = ()
    route_edges: tuple[str, ...] = ()
    rejection_reason: str = ""
    rejection_detail: str = ""
    queue_detail: str = ""
    comms_lost: bool = False
    created_at: float = 0.0
    started_at: float | None = None
    completed_at: float | None = None
    history: list[str] = field(default_factory=list)

    @property
    def duration(self) -> float | None:
        if self.started_at is None or self.completed_at is None:
            return None
        return self.completed_at - self.started_at

    def snapshot(self) -> dict[str, Any]:
        return {
            "order_id": self.order_id,
            "idempotency_key": self.idempotency_key,
            "task_type": self.request.task_type,
            "pickup_node": self.request.pickup_node,
            "dropoff_node": self.request.dropoff_node,
            "state": self.state.value,
            "robot_id": self.robot_id,
            "comms_lost": self.comms_lost,
            "rejection_reason": self.rejection_reason,
            "rejection_detail": self.rejection_detail,
            "queue_detail": self.queue_detail,
            "created_at": round(self.created_at, 3),
            "started_at": None if self.started_at is None else round(self.started_at, 3),
            "completed_at": (
                None if self.completed_at is None else round(self.completed_at, 3)
            ),
            "duration": None if self.duration is None else round(self.duration, 3),
        }


class FleetCoordinator:
    """Reservation/queue baseline over one facility model and a robot set."""

    def __init__(
        self,
        facility: FacilityMap,
        robots: dict[str, RobotState],
        min_battery: float = DEFAULT_MIN_BATTERY,
        battery_range_m: float = 400.0,
    ) -> None:
        self.facility = facility
        self.robots = robots
        self.min_battery = min_battery
        self.battery_range_m = battery_range_m
        self.orders: list[Order] = []
        self.node_owner: dict[str, str] = {}
        self.edge_owner: dict[str, str] = {}
        self.routes: dict[str, tuple[str, ...]] = {}
        self.route_edges: dict[str, tuple[str, ...]] = {}
        self.route_index: dict[str, int] = {}
        self.offline: set[str] = set()
        self._by_key: dict[str, Order] = {}
        self._offline_completions: dict[str, float] = {}
        self.events: list[str] = []

    # ---- intake -------------------------------------------------------
    def submit(self, request: OrderRequest, t: float = 0.0) -> Order:
        existing = self._by_key.get(request.idempotency_key)
        if existing is not None:
            self._log(t, f"idempotent replay for key {request.idempotency_key!r}")
            return existing
        if any(order.order_id == request.order_id for order in self.orders):
            raise ValueError(
                f"order_id {request.order_id!r} already exists with a different idempotency key"
            )

        order = Order(
            order_id=request.order_id,
            idempotency_key=request.idempotency_key,
            request=request,
            created_at=t,
        )
        self.orders.append(order)
        self._by_key[request.idempotency_key] = order
        order.history.append(f"t={t:.2f} submitted")
        for node_id in (request.pickup_node, request.dropoff_node):
            if node_id not in self.facility.nodes:
                order.state = OrderState.REJECTED
                order.rejection_reason = "INVALID_ORDER"
                order.rejection_detail = f"unknown facility node {node_id!r}"
                order.history.append(f"t={t:.2f} rejected: unknown node {node_id!r}")
                return order
        return order

    # ---- dispatch -----------------------------------------------------
    def assign_pending(self, t: float = 0.0) -> list[Order]:
        """Dispatch and reserve every PENDING order that can start now."""
        started: list[Order] = []
        pending = [o for o in self.orders if o.state is OrderState.PENDING]
        pending.sort(key=lambda o: (-o.request.priority, o.created_at, o.order_id))
        for order in pending:
            assignment = select_robot(
                order.request, self.robots, self.facility, self.min_battery
            )
            if isinstance(assignment, Rejection):
                order.state = OrderState.REJECTED
                order.rejection_reason = assignment.reason
                order.rejection_detail = assignment.detail
                order.history.append(
                    f"t={t:.2f} denied ({assignment.reason}): {assignment.detail}"
                )
                continue
            self._try_start(order, assignment, t, started)
        return started

    def _try_start(
        self,
        order: Order,
        assignment: Assignment,
        t: float,
        started: list[Order],
    ) -> None:
        robot_id = assignment.robot_id
        node_ids = assignment.node_ids
        edge_ids = assignment.edge_ids
        if not self._try_reserve(robot_id, node_ids, edge_ids):
            order.queue_detail = "route held by another robot"
            order.history.append(f"t={t:.2f} queued: route held by another robot")
            return

        order.state = OrderState.IN_PROGRESS
        order.robot_id = robot_id
        order.route_nodes = node_ids
        order.route_edges = edge_ids
        order.started_at = t
        order.queue_detail = ""
        order.history.append(
            f"t={t:.2f} started on {robot_id} ({len(node_ids)} nodes, "
            f"{len(edge_ids)} lanes reserved)"
        )
        robot = self.robots[robot_id]
        robot.available = False
        self.routes[robot_id] = node_ids
        self.route_edges[robot_id] = edge_ids
        self.route_index[robot_id] = 0
        self._log(t, f"{order.order_id} started on {robot_id}")
        started.append(order)

    def _try_reserve(
        self, robot_id: str, node_ids: tuple[str, ...], edge_ids: tuple[str, ...]
    ) -> bool:
        for node_id in node_ids:
            owner = self.node_owner.get(node_id)
            if owner is not None and owner != robot_id:
                return False
        for edge_id in edge_ids:
            owner = self.edge_owner.get(edge_id)
            if owner is not None and owner != robot_id:
                return False
        for node_id in node_ids:
            self.node_owner[node_id] = robot_id
        for edge_id in edge_ids:
            self.edge_owner[edge_id] = robot_id
        return True

    # ---- execution ----------------------------------------------------
    def active_order(self, robot_id: str) -> Order | None:
        for order in self.orders:
            if order.robot_id == robot_id and order.state is OrderState.IN_PROGRESS:
                return order
        return None

    def advance_to(self, robot_id: str, node_id: str, t: float = 0.0) -> None:
        """Record that a robot reached ``node_id``; release nodes it has left."""
        route = self.routes.get(robot_id)
        if route is None:
            raise KeyError(f"robot {robot_id!r} holds no route")
        if node_id not in route:
            raise KeyError(f"node {node_id!r} is not on the route of {robot_id!r}")
        index = route.index(node_id)
        if index < self.route_index[robot_id]:
            raise ValueError(f"{robot_id!r} cannot move backwards along its route")
        for passed in route[:index]:
            if self.node_owner.get(passed) == robot_id:
                del self.node_owner[passed]
        edges = self.route_edges[robot_id]
        for passed_edge in edges[:index]:
            if self.edge_owner.get(passed_edge) == robot_id:
                del self.edge_owner[passed_edge]
        self.route_index[robot_id] = index
        self.robots[robot_id].at_node = node_id

    def consume_distance(self, robot_id: str, distance: float) -> None:
        robot = self.robots[robot_id]
        robot.battery = max(0.0, robot.battery - distance / self.battery_range_m)

    def complete_route(self, robot_id: str, t: float = 0.0) -> Order | None:
        """Finish the active order when the robot reaches its last node."""
        order = self.active_order(robot_id)
        if order is None:
            return None
        if robot_id in self.offline:
            self._offline_completions[robot_id] = t
            order.history.append(
                f"t={t:.2f} released base completed while offline; awaiting reconciliation"
            )
            self._log(t, f"{order.order_id} base completed offline on {robot_id}")
            return None
        return self._finalise(order, t)

    def _finalise(self, order: Order, t: float) -> Order:
        order.state = OrderState.COMPLETED
        order.completed_at = t
        order.history.append(f"t={t:.2f} completed")
        robot_id = order.robot_id
        if robot_id is not None:
            self._release(robot_id)
            robot = self.robots[robot_id]
            robot.available = True
            self.routes.pop(robot_id, None)
            self.route_edges.pop(robot_id, None)
            self.route_index.pop(robot_id, None)
            self._offline_completions.pop(robot_id, None)
        self._log(t, f"{order.order_id} completed")
        return order

    def _release(self, robot_id: str) -> None:
        for node_id in [n for n, owner in self.node_owner.items() if owner == robot_id]:
            del self.node_owner[node_id]
        for edge_id in [e for e, owner in self.edge_owner.items() if owner == robot_id]:
            del self.edge_owner[edge_id]

    # ---- verification and connectivity --------------------------------
    def report_outcome(
        self, order_id: str, confirmed: bool | None, t: float = 0.0
    ) -> Order:
        """Report a pick/drop outcome; ``None`` means ambiguous (needs verification)."""
        order = self._order(order_id)
        if order.state not in (OrderState.IN_PROGRESS, OrderState.NEEDS_VERIFICATION):
            raise ValueError(
                f"order {order_id!r} is {order.state.value}; outcome cannot be reported"
            )
        if confirmed is None:
            order.state = OrderState.NEEDS_VERIFICATION
            order.history.append(
                f"t={t:.2f} ambiguous pick/drop outcome; parked for verification"
            )
            if order.robot_id is not None:
                self._release(order.robot_id)
                robot = self.robots[order.robot_id]
                robot.available = True
                self.routes.pop(order.robot_id, None)
                self.route_edges.pop(order.robot_id, None)
                self.route_index.pop(order.robot_id, None)
            self._log(t, f"{order_id} needs verification")
            return order
        if confirmed:
            if order.state is OrderState.NEEDS_VERIFICATION:
                order.history.append(f"t={t:.2f} verification confirmed")
                return self._finalise(order, t)
            order.history.append(f"t={t:.2f} outcome confirmed")
            return self._finalise(order, t)
        order.state = OrderState.FAILED
        order.completed_at = t
        order.history.append(f"t={t:.2f} outcome reported failed")
        if order.robot_id is not None:
            self._release(order.robot_id)
            self.robots[order.robot_id].available = True
        self._log(t, f"{order_id} failed")
        return order

    def go_offline(self, robot_id: str, t: float = 0.0) -> None:
        self.offline.add(robot_id)
        order = self.active_order(robot_id)
        if order is not None:
            order.comms_lost = True
            order.history.append(
                f"t={t:.2f} connectivity lost; robot continues its released base"
            )
        self._log(t, f"{robot_id} offline; released base continues")

    def go_online(self, robot_id: str, t: float = 0.0) -> Order | None:
        self.offline.discard(robot_id)
        order = self.active_order(robot_id)
        self._log(t, f"{robot_id} online")
        if robot_id in self._offline_completions and order is not None:
            completed = self._finalise(order, t)
            completed.history.append(f"t={t:.2f} reconciled after reconnect")
            return completed
        return order

    # ---- reporting ----------------------------------------------------
    def _order(self, order_id: str) -> Order:
        for order in self.orders:
            if order.order_id == order_id:
                return order
        raise KeyError(f"unknown order {order_id!r}")

    def _log(self, t: float, message: str) -> None:
        self.events.append(f"t={t:.2f} {message}")

    def pending_orders(self) -> list[Order]:
        return [o for o in self.orders if o.state is OrderState.PENDING]

    def unfinished_orders(self) -> list[Order]:
        return [
            o
            for o in self.orders
            if o.state not in TERMINAL_STATES
            and o.state is not OrderState.NEEDS_VERIFICATION
        ]

    def snapshot(self) -> dict[str, Any]:
        return {
            "orders": [order.snapshot() for order in self.orders],
            "robots": {
                robot_id: {
                    "at_node": robot.at_node,
                    "available": robot.available,
                    "battery": round(robot.battery, 4),
                    "offline": robot_id in self.offline,
                }
                for robot_id, robot in sorted(self.robots.items())
            },
            "reservations": {
                "nodes": dict(sorted(self.node_owner.items())),
                "edges": dict(sorted(self.edge_owner.items())),
            },
            "events": list(self.events),
        }
