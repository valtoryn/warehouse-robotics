#!/usr/bin/env python3
"""Run the two-robot fleet-coordination baseline over the reference facility.

Scenario (all software-simulated, no physics engine):
  * amr_01 picks up at pick_1 and drops at drop_1 (north aisle).
  * amr_02 picks up at pick_2 and drops at drop_2 (south aisle).
  * A payload order is submitted that the fleet cannot take: amr_01 declares
    payload.pick_place without verification and amr_02 does not declare it, so
    capability dispatch must deny it (default-deny, FR-F5).

Exit status is non-zero if a move order does not complete, the payload order is
not denied, or any reservation violation is recorded. Writes
generated/fleet_report.json.

Usage:
  python tools/run_fleet_sim.py
  python tools/run_fleet_sim.py --report /tmp/fleet_report.json --max-seconds 600
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_facility"))

from valtoryn_facility.coordinator import FleetCoordinator, OrderState  # noqa: E402
from valtoryn_facility.dispatch import Capability, OrderRequest, RobotState  # noqa: E402
from valtoryn_facility.facility_model import load_facility  # noqa: E402
from valtoryn_facility.sim import FleetSimulator  # noqa: E402

MOVE_CAPABILITIES = ("navigation.autonomous", "traffic.route_following")


def build_fleet() -> dict[str, RobotState]:
    """Two reference AMRs. Verification refs point at simulation evidence only."""
    navigation = Capability(
        "navigation.autonomous",
        "navigation",
        verified=True,
        verification_ref="ci:generated/scenario_report.json (Phase 3 simulation closure)",
    )
    traffic = Capability(
        "traffic.route_following",
        "traffic",
        verified=True,
        verification_ref="sim:generated/fleet_report.json",
    )
    payload_unverified = Capability(
        "payload.pick_place",
        "payload",
        verified=False,
        verification_ref="",
    )
    return {
        "amr_01": RobotState(
            robot_id="amr_01",
            capabilities={
                "navigation.autonomous": navigation,
                "traffic.route_following": traffic,
                "payload.pick_place": payload_unverified,
            },
            at_node="pick_1",
            battery=0.92,
        ),
        "amr_02": RobotState(
            robot_id="amr_02",
            capabilities={
                "navigation.autonomous": navigation,
                "traffic.route_following": traffic,
            },
            at_node="pick_2",
            battery=0.88,
        ),
    }


def build_orders() -> list[OrderRequest]:
    return [
        OrderRequest(
            order_id="move-1001",
            idempotency_key="wms:1001:v1",
            task_type="move_pallet",
            required_capabilities=MOVE_CAPABILITIES,
            pickup_node="pick_1",
            dropoff_node="drop_1",
        ),
        OrderRequest(
            order_id="move-1002",
            idempotency_key="wms:1002:v1",
            task_type="move_pallet",
            required_capabilities=MOVE_CAPABILITIES,
            pickup_node="pick_2",
            dropoff_node="drop_2",
        ),
        OrderRequest(
            order_id="transfer-2001",
            idempotency_key="wms:2001:v1",
            task_type="replenish_shelf",
            required_capabilities=("payload.pick_place",),
            pickup_node="pick_1",
            dropoff_node="drop_1",
        ),
    ]


def evaluate(report: dict) -> list[str]:
    """Return human-readable failures; empty means the scenario passed."""
    failures: list[str] = []
    if report["violations"]:
        failures.extend(report["violations"])
    if not report["converged"]:
        failures.append("simulation did not reach a terminal state for every order")

    orders = {order["order_id"]: order for order in report["orders"]}
    for order_id in ("move-1001", "move-1002"):
        order = orders.get(order_id)
        if order is None:
            failures.append(f"{order_id} was never submitted")
        elif order["state"] != OrderState.COMPLETED.value:
            failures.append(f"{order_id} ended in {order['state']}")

    denied = orders.get("transfer-2001")
    if denied is None:
        failures.append("transfer-2001 was never submitted")
    elif denied["state"] != OrderState.REJECTED.value:
        failures.append(
            f"transfer-2001 should be denied by capability dispatch, ended in {denied['state']}"
        )
    elif "MISSING_CAPABILITY" not in denied["rejection_detail"]:
        failures.append(
            "transfer-2001 denial does not cite the missing payload capability: "
            f"{denied['rejection_detail']}"
        )
    return failures


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=ROOT / "generated" / "fleet_report.json")
    parser.add_argument("--max-seconds", type=float, default=600.0)
    args = parser.parse_args(argv)

    facility = load_facility()
    coordinator = FleetCoordinator(facility, build_fleet())
    for request in build_orders():
        coordinator.submit(request)

    simulator = FleetSimulator(coordinator)
    report = simulator.run(max_seconds=args.max_seconds)
    report["facility"] = {
        "name": facility.name,
        "facility_map_version": facility.facility_map_version,
    }
    failures = evaluate(report)
    report["failures"] = failures

    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    for order in report["orders"]:
        detail = order["rejection_reason"] or order["state"]
        duration = "" if order["duration"] is None else f", {order['duration']:.2f}s"
        print(
            f"{order['order_id']:<14} {detail:<18} robot={order['robot_id'] or '-':<7}"
            f"{duration}"
        )
    print(
        f"sim_time={report['sim_time']:.2f}s converged={report['converged']} "
        f"violations={len(report['violations'])}"
    )
    for failure in failures:
        print("FAILURE:", failure)
    print(f"report written to {args.report.relative_to(ROOT) if args.report.is_relative_to(ROOT) else args.report}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
