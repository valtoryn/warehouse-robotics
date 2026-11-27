#!/usr/bin/env python3
"""Tests for the WMS-facing order API (FR-F7).

The HTTP tests exercise a real ``ThreadingHTTPServer`` bound to an ephemeral
loopback port; requests go over TCP with ``urllib``, so routing, JSON
encoding, status codes, headers and the service lock are all covered. Nothing
here touches ROS, Gazebo, a WMS or the network beyond 127.0.0.1.

Run: python -m unittest tests.test_fleet_api -v
"""
from __future__ import annotations

import json
import sys
import threading
import unittest
import urllib.error
import urllib.request
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_facility"))
sys.path.insert(0, str(ROOT / "tools"))

from valtoryn_facility.api import (  # noqa: E402
    API_PREFIX,
    OrderService,
    make_server,
)
from valtoryn_facility.coordinator import FleetCoordinator, OrderState  # noqa: E402
from valtoryn_facility.facility_model import load_facility  # noqa: E402

from run_fleet_sim import build_fleet  # noqa: E402

MOVE_CAPS = ["navigation.autonomous", "traffic.route_following"]


class ApiTestCase(unittest.TestCase):
    """Serves the reference facility and fleet on an ephemeral loopback port."""

    def setUp(self) -> None:
        self.facility = load_facility()
        self.robots = build_fleet()
        self.coordinator = FleetCoordinator(self.facility, self.robots)
        self.service = OrderService(
            self.facility, self.coordinator, clock=lambda: 0.0
        )
        self.server = make_server(self.service, host="127.0.0.1", port=0, log=None)
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True
        )
        self.thread.start()
        host, port = self.server.server_address[:2]
        self.base = f"http://{host}:{port}{API_PREFIX}"

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=5)

    # ---- helpers ------------------------------------------------------
    def request(
        self,
        method: str,
        path: str,
        body: dict | None = None,
        headers: dict[str, str] | None = None,
        raw: bytes | None = None,
    ) -> tuple[int, dict]:
        data = raw
        if data is None and body is not None:
            data = json.dumps(body).encode("utf-8")
        request = urllib.request.Request(
            self.base + path,
            data=data,
            method=method,
            headers={"Content-Type": "application/json", **(headers or {})},
        )
        try:
            with urllib.request.urlopen(request, timeout=15) as response:
                return response.status, json.loads(response.read() or b"{}")
        except urllib.error.HTTPError as exc:
            payload = exc.read()
            return exc.code, json.loads(payload) if payload.strip() else {}

    def submit(
        self,
        order_id: str = "move-3001",
        key: str = "wms:3001:v1",
        **overrides,
    ) -> tuple[int, dict]:
        body = {
            "order_id": order_id,
            "task_type": "move_pallet",
            "required_capabilities": MOVE_CAPS,
            "pickup_node": "pick_1",
            "dropoff_node": "drop_1",
        }
        body.update(overrides)
        return self.request("POST", "/orders", body, {"Idempotency-Key": key})

    # ---- liveness and facility ---------------------------------------
    def test_health_reports_facility_version(self) -> None:
        status, payload = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(
            payload["facility_map_version"], self.facility.facility_map_version
        )

    def test_facility_summary_matches_loaded_model(self) -> None:
        status, payload = self.request("GET", "/facility")
        self.assertEqual(status, 200)
        self.assertEqual(payload["counts"]["nodes"], len(self.facility.nodes))
        self.assertEqual(payload["counts"]["edges"], len(self.facility.edges))
        self.assertEqual(payload["frame"], self.facility.frame)
        self.assertIn("pick_1", payload["nodes"])

    # ---- order intake --------------------------------------------------
    def test_submit_order_is_created_pending(self) -> None:
        status, payload = self.submit()
        self.assertEqual(status, 201)
        self.assertTrue(payload["created"])
        self.assertFalse(payload["idempotent_replay"])
        order = payload["order"]
        self.assertEqual(order["order_id"], "move-3001")
        self.assertEqual(order["state"], OrderState.PENDING.value)
        self.assertEqual(order["idempotency_key"], "wms:3001:v1")
        self.assertEqual(order["pickup_node"], "pick_1")

    def test_server_generates_order_id_when_absent(self) -> None:
        status, payload = self.request(
            "POST",
            "/orders",
            {
                "task_type": "move_pallet",
                "required_capabilities": MOVE_CAPS,
                "pickup_node": "pick_1",
                "dropoff_node": "drop_1",
            },
            {"Idempotency-Key": "wms:auto:2"},
        )
        self.assertEqual(status, 201)
        self.assertTrue(payload["order"]["order_id"].startswith("ord-"))

    def test_idempotency_key_is_required(self) -> None:
        status, payload = self.request(
            "POST",
            "/orders",
            {
                "task_type": "move_pallet",
                "required_capabilities": MOVE_CAPS,
                "pickup_node": "pick_1",
                "dropoff_node": "drop_1",
            },
        )
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"]["code"], "MISSING_IDEMPOTENCY_KEY")

    def test_mismatched_header_and_body_key_is_refused(self) -> None:
        status, payload = self.submit(idempotency_key="wms:other:v1")
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"]["code"], "IDEMPOTENCY_KEY_MISMATCH")

    def test_replay_returns_the_existing_order(self) -> None:
        first_status, first = self.submit()
        second_status, second = self.submit()
        self.assertEqual(first_status, 201)
        self.assertEqual(second_status, 200)
        self.assertTrue(second["idempotent_replay"])
        self.assertFalse(second["created"])
        self.assertEqual(first["order"]["order_id"], second["order"]["order_id"])

        status, listed = self.request("GET", "/orders")
        self.assertEqual(status, 200)
        self.assertEqual(listed["count"], 1)

    def test_reusing_order_id_with_new_key_is_a_conflict(self) -> None:
        self.submit(order_id="move-3001", key="wms:3001:v1")
        status, payload = self.submit(order_id="move-3001", key="wms:3001:v2")
        self.assertEqual(status, 409)
        self.assertEqual(payload["error"]["code"], "ORDER_CONFLICT")
        self.assertIn("idempotency", payload["error"]["detail"])

    def test_unknown_node_is_refused_at_intake(self) -> None:
        status, payload = self.submit(pickup_node="dock_nowhere")
        self.assertEqual(status, 400)
        self.assertIn("dock_nowhere", payload["error"]["detail"])

    def test_missing_task_type_is_refused(self) -> None:
        status, payload = self.submit(task_type=None)
        self.assertEqual(status, 400)
        self.assertIn("task_type", payload["error"]["detail"])

    def test_malformed_json_is_refused(self) -> None:
        status, payload = self.request(
            "POST", "/orders", raw=b"{not json", headers={"Idempotency-Key": "k"}
        )
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"]["code"], "INVALID_JSON")

    def test_priority_must_be_an_integer(self) -> None:
        status, payload = self.submit(priority="high")
        self.assertEqual(status, 400)
        self.assertIn("priority", payload["error"]["detail"])

    # ---- dispatch ------------------------------------------------------
    def test_order_without_requirements_is_denied_by_default(self) -> None:
        self.submit(required_capabilities=[])
        status, payload = self.request("POST", "/dispatch", {})
        self.assertEqual(status, 200)
        order = payload["orders"][0]
        self.assertEqual(order["state"], OrderState.REJECTED.value)
        self.assertEqual(
            order["rejection_reason"], "UNDECLARED_TASK_REQUIREMENTS"
        )

    def test_unverified_capability_is_reported_as_capability_gap(self) -> None:
        self.submit(required_capabilities=["payload.pick_place"])
        status, payload = self.request("POST", "/dispatch", {})
        self.assertEqual(status, 200)
        order = payload["orders"][0]
        self.assertEqual(order["state"], OrderState.REJECTED.value)
        self.assertEqual(order["rejection_reason"], "NO_ELIGIBLE_ROBOT")
        self.assertIn("MISSING_CAPABILITY", order["rejection_detail"])
        self.assertIn("UNVERIFIED_CAPABILITY", order["rejection_detail"])

    def test_dispatch_reserves_routes_and_marks_robots_busy(self) -> None:
        self.submit(order_id="move-3001", key="wms:3001:v1")
        self.submit(
            order_id="move-3002",
            key="wms:3002:v1",
            pickup_node="pick_2",
            dropoff_node="drop_2",
        )
        status, payload = self.request("POST", "/dispatch", {"time": 1.0})
        self.assertEqual(status, 200)
        self.assertEqual(len(payload["started"]), 2)
        self.assertEqual(len(payload["orders"]), 2)
        for order in payload["orders"]:
            self.assertEqual(order["state"], OrderState.IN_PROGRESS.value)
            self.assertIsNotNone(order["robot_id"])

        status, robots = self.request("GET", "/robots")
        self.assertEqual(status, 200)
        self.assertEqual(robots["count"], 2)
        self.assertFalse(any(robot["available"] for robot in robots["robots"]))

        status, snapshot = self.request("GET", "/snapshot")
        self.assertEqual(status, 200)
        self.assertTrue(snapshot["reservations"]["nodes"])
        self.assertTrue(snapshot["reservations"]["edges"])
        self.assertEqual(
            snapshot["facility"]["facility_map_version"],
            self.facility.facility_map_version,
        )

    def test_second_order_on_a_held_route_waits(self) -> None:
        self.submit(order_id="move-3001", key="wms:3001:v1")
        self.submit(order_id="move-3002", key="wms:3002:v1")
        status, payload = self.request("POST", "/dispatch", {})
        self.assertEqual(status, 200)
        self.assertEqual(len(payload["started"]), 1)
        states = {order["order_id"]: order["state"] for order in payload["orders"]}
        self.assertEqual(states["move-3001"], OrderState.IN_PROGRESS.value)
        self.assertEqual(states["move-3002"], OrderState.PENDING.value)
        self.assertEqual(payload["pending"], 1)

    # ---- lifecycle over HTTP -------------------------------------------
    def test_ambiguous_outcome_parks_order_until_verified(self) -> None:
        self.submit()
        self.request("POST", "/dispatch", {})
        status, order = self.request(
            "POST", "/orders/move-3001/outcome", {"confirmed": None}
        )
        self.assertEqual(status, 200)
        self.assertEqual(order["state"], OrderState.NEEDS_VERIFICATION.value)

        status, payload = self.request(
            "POST", "/orders/move-3001/verification", {"confirmed": True}
        )
        self.assertEqual(status, 200, payload)
        self.assertEqual(payload["state"], OrderState.COMPLETED.value)
        self.assertIsNotNone(payload["completed_at"])

        status, robots = self.request("GET", "/robots")
        self.assertTrue(all(robot["available"] for robot in robots["robots"]))
        self.assertEqual(self.coordinator.node_owner, {})

    def test_verification_of_a_running_order_is_a_conflict(self) -> None:
        self.submit()
        self.request("POST", "/dispatch", {})
        status, payload = self.request(
            "POST", "/orders/move-3001/verification", {"confirmed": True}
        )
        self.assertEqual(status, 409)
        self.assertEqual(payload["error"]["code"], "INVALID_ORDER_STATE")

    def test_ambiguous_outcome_cannot_resolve_verification(self) -> None:
        self.submit()
        self.request("POST", "/dispatch", {})
        self.request("POST", "/orders/move-3001/outcome", {"confirmed": None})
        status, payload = self.request(
            "POST", "/orders/move-3001/verification", {"confirmed": None}
        )
        self.assertEqual(status, 400)
        self.assertEqual(payload["error"]["code"], "INVALID_VERIFICATION")

    def test_failed_outcome_marks_order_failed_and_frees_robot(self) -> None:
        self.submit()
        self.request("POST", "/dispatch", {})
        status, order = self.request(
            "POST", "/orders/move-3001/outcome", {"confirmed": False}
        )
        self.assertEqual(status, 200)
        self.assertEqual(order["state"], OrderState.FAILED.value)
        self.assertTrue(self.coordinator.robots[order["robot_id"]].available)

    def test_comms_loss_completes_only_after_reconnect(self) -> None:
        self.submit()
        status, dispatched = self.request("POST", "/dispatch", {})
        robot_id = dispatched["started"][0]["robot_id"]

        status, payload = self.request(
            "POST", f"/robots/{robot_id}/connectivity", {"online": False}
        )
        self.assertEqual(status, 200)
        self.assertTrue(payload["robot"]["offline"])
        self.assertTrue(payload["order"]["comms_lost"])

        status, payload = self.request("POST", f"/robots/{robot_id}/completed", {})
        self.assertEqual(status, 202)
        self.assertEqual(payload["state"], "AWAITING_RECONCILIATION")
        self.assertIsNone(payload["order"])

        status, payload = self.request(
            "POST", f"/robots/{robot_id}/connectivity", {"online": True}
        )
        self.assertEqual(status, 200)
        self.assertEqual(payload["order"]["state"], OrderState.COMPLETED.value)
        self.assertTrue(payload["order"]["comms_lost"])

    def test_robot_position_advances_the_reserved_route(self) -> None:
        self.submit()
        _, dispatched = self.request("POST", "/dispatch", {})
        order = dispatched["started"][0]
        self.assertNotIn("route_nodes", order)  # route internals stay off the payload

        route = self.coordinator.routes[order["robot_id"]]
        status, robot = self.request(
            "POST", f"/robots/{order['robot_id']}/position", {"node": route[1]}
        )
        self.assertEqual(status, 200)
        self.assertEqual(robot["at_node"], route[1])

    def test_robot_position_off_route_is_a_conflict(self) -> None:
        self.submit()
        _, dispatched = self.request("POST", "/dispatch", {})
        robot_id = dispatched["started"][0]["robot_id"]
        status, payload = self.request(
            "POST", f"/robots/{robot_id}/position", {"node": "drop_2"}
        )
        self.assertEqual(status, 409)
        self.assertEqual(payload["error"]["code"], "INVALID_PROGRESS")

    def test_robot_completed_without_order_is_a_conflict(self) -> None:
        status, payload = self.request("POST", "/robots/amr_01/completed", {})
        self.assertEqual(status, 409)
        self.assertEqual(payload["error"]["code"], "NO_ACTIVE_ORDER")

    # ---- filtering and errors ------------------------------------------
    def test_orders_can_be_filtered_by_state(self) -> None:
        self.submit(order_id="move-3001", key="wms:3001:v1")
        status, payload = self.request("GET", "/orders?state=PENDING")
        self.assertEqual(status, 200)
        self.assertEqual(payload["count"], 1)
        status, payload = self.request("GET", "/orders?state=COMPLETED")
        self.assertEqual(payload["count"], 0)
        status, payload = self.request("GET", "/orders?state=NONSENSE")
        self.assertEqual(status, 400)

    def test_unknown_order_and_robot_are_not_found(self) -> None:
        status, payload = self.request("GET", "/orders/does-not-exist")
        self.assertEqual(status, 404)
        self.assertEqual(payload["error"]["code"], "ORDER_NOT_FOUND")

        status, payload = self.request("GET", "/robots/amr_99")
        self.assertEqual(status, 404)
        self.assertEqual(payload["error"]["code"], "ROBOT_NOT_FOUND")

    def test_unknown_path_and_bad_method_are_reported_as_json(self) -> None:
        status, payload = self.request("GET", "/nope")
        self.assertEqual(status, 404)
        self.assertEqual(payload["error"]["code"], "NOT_FOUND")

        status, payload = self.request("DELETE", "/orders/move-3001")
        self.assertEqual(status, 405)
        self.assertEqual(payload["error"]["code"], "METHOD_NOT_ALLOWED")

    def test_robot_view_reports_capability_verification(self) -> None:
        status, payload = self.request("GET", "/robots/amr_01")
        self.assertEqual(status, 200)
        capabilities = {c["name"]: c for c in payload["capabilities"]}
        self.assertTrue(capabilities["navigation.autonomous"]["verified"])
        self.assertFalse(capabilities["payload.pick_place"]["verified"])
        self.assertIn("generated/", capabilities["navigation.autonomous"]["verification_ref"])

    def test_concurrent_idempotent_submissions_create_one_order(self) -> None:
        def send(_: int) -> tuple[int, dict]:
            return self.submit(order_id="move-4001", key="wms:4001:v1")

        with ThreadPoolExecutor(max_workers=8) as pool:
            results = list(pool.map(send, range(8)))
        codes = sorted(status for status, _ in results)
        self.assertEqual(codes.count(201), 1, codes)
        self.assertEqual(codes.count(200), 7, codes)
        order_ids = {payload["order"]["order_id"] for _, payload in results}
        self.assertEqual(order_ids, {"move-4001"})

        status, listed = self.request("GET", "/orders")
        self.assertEqual(listed["count"], 1)


if __name__ == "__main__":
    unittest.main()
