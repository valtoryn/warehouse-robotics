#!/usr/bin/env python3
"""Capability-matching task dispatch with default-deny semantics (FR-F5).

A task is only assignable to a robot that *declares* every capability the task
requires, and — when ``require_verified`` is set — whose capability was
confirmed by a recorded verification method. A task that declares no
requirements is denied rather than treated as "anything can do it".

Verification here means a recorded verification reference exists in the
capability profile. It is not a physical-safety claim; the reference strings
this project can honestly produce today point at simulation evidence.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from .facility_model import FacilityMap, Route

# Rejection reasons (stable strings; reported in order records and the CLI).
DENY_UNDECLARED_TASK = "UNDECLARED_TASK_REQUIREMENTS"
INVALID_ORDER = "INVALID_ORDER"
NO_ELIGIBLE_ROBOT = "NO_ELIGIBLE_ROBOT"
NO_ROUTE = "NO_ROUTE"

ROBOT_LOW_BATTERY = "LOW_BATTERY"
ROBOT_UNAVAILABLE = "UNAVAILABLE"
ROBOT_MISSING_CAPABILITY = "MISSING_CAPABILITY"
ROBOT_UNVERIFIED_CAPABILITY = "UNVERIFIED_CAPABILITY"

DEFAULT_MIN_BATTERY = 0.25


@dataclass(frozen=True)
class Capability:
    """A capability declaration (mirrors valtoryn_interfaces/msg/Capability.msg)."""

    name: str
    kind: str
    verified: bool = False
    verification_ref: str = ""


@dataclass
class RobotState:
    robot_id: str
    capabilities: dict[str, Capability] = field(default_factory=dict)
    at_node: str = ""
    available: bool = True
    battery: float = 1.0


@dataclass(frozen=True)
class OrderRequest:
    order_id: str
    idempotency_key: str
    task_type: str
    required_capabilities: tuple[str, ...]
    pickup_node: str
    dropoff_node: str
    require_verified: bool = True
    priority: int = 0


@dataclass(frozen=True)
class Assignment:
    order_id: str
    robot_id: str
    route_to_pickup: Route
    route_to_dropoff: Route

    @property
    def node_ids(self) -> tuple[str, ...]:
        pickup = self.route_to_pickup.node_ids
        return pickup + self.route_to_dropoff.node_ids[1:]

    @property
    def edge_ids(self) -> tuple[str, ...]:
        return tuple(
            dict.fromkeys(
                self.route_to_pickup.edge_ids + self.route_to_dropoff.edge_ids
            )
        )

    @property
    def duration(self) -> float:
        return self.route_to_pickup.duration + self.route_to_dropoff.duration


@dataclass(frozen=True)
class Rejection:
    order_id: str
    reason: str
    detail: str


def select_robot(
    order: OrderRequest,
    robots: dict[str, RobotState],
    facility: FacilityMap,
    min_battery: float = DEFAULT_MIN_BATTERY,
) -> Assignment | Rejection:
    """Pick the closest eligible robot, or explain why dispatch must deny."""
    if not order.required_capabilities:
        return Rejection(
            order.order_id,
            DENY_UNDECLARED_TASK,
            "task declares no required capabilities; dispatch defaults to deny",
        )
    if order.pickup_node not in facility.nodes:
        return Rejection(
            order.order_id, INVALID_ORDER, f"unknown pickup node {order.pickup_node!r}"
        )
    if order.dropoff_node not in facility.nodes:
        return Rejection(
            order.order_id, INVALID_ORDER, f"unknown dropoff node {order.dropoff_node!r}"
        )

    reasons: list[str] = []
    best: tuple[float, str, Assignment] | None = None
    for robot_id in sorted(robots):
        robot = robots[robot_id]
        # The capability gate is intrinsic to the robot and is reported before
        # transient state: a task no robot could ever perform must read as a
        # capability gap, not as "everyone happens to be busy right now".
        capability_failure: str | None = None
        for name in order.required_capabilities:
            capability = robot.capabilities.get(name)
            if capability is None:
                capability_failure = (
                    f"{robot_id}: {ROBOT_MISSING_CAPABILITY} {name!r} not declared"
                )
                break
            if order.require_verified and not capability.verified:
                capability_failure = (
                    f"{robot_id}: {ROBOT_UNVERIFIED_CAPABILITY} {name!r} is declared only"
                )
                break
        if capability_failure is not None:
            reasons.append(capability_failure)
            continue
        if not robot.available:
            reasons.append(f"{robot_id}: {ROBOT_UNAVAILABLE}")
            continue
        if robot.battery < min_battery:
            reasons.append(
                f"{robot_id}: {ROBOT_LOW_BATTERY} ({robot.battery:.0%} < {min_battery:.0%})"
            )
            continue
        if robot.at_node not in facility.nodes:
            reasons.append(f"{robot_id}: {NO_ROUTE} (unknown position {robot.at_node!r})")
            continue

        to_pickup = facility.shortest_path(robot.at_node, order.pickup_node)
        if to_pickup is None:
            reasons.append(
                f"{robot_id}: {NO_ROUTE} to pickup {order.pickup_node!r}"
            )
            continue
        to_dropoff = facility.shortest_path(order.pickup_node, order.dropoff_node)
        if to_dropoff is None:
            reasons.append(
                f"{robot_id}: {NO_ROUTE} from {order.pickup_node!r} to {order.dropoff_node!r}"
            )
            continue

        assignment = Assignment(
            order_id=order.order_id,
            robot_id=robot_id,
            route_to_pickup=to_pickup,
            route_to_dropoff=to_dropoff,
        )
        score = to_pickup.duration
        if best is None or (score, robot_id) < (best[0], best[1]):
            best = (score, robot_id, assignment)

    if best is None:
        detail = "; ".join(reasons) if reasons else "no robots registered"
        return Rejection(order.order_id, NO_ELIGIBLE_ROBOT, detail)
    return best[2]
