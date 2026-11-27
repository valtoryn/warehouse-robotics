#!/usr/bin/env python3
"""Unit tests for the Valtoryn facility model and fleet-coordination baseline.

Covers:
  - facility loading, schema and structural validation
  - lane routing with speed-limit zones and blocked lanes
  - exporters: Nav2 GeoJSON round-trip, RMF graph, LIF, VDA 5050 order/horizon
  - capability dispatch with default-deny semantics
  - coordinator: queueing, idempotency, verification, comms loss
  - deterministic two-robot coordination simulation

Run: python -m unittest discover -s tests -v
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_facility"))
sys.path.insert(0, str(ROOT / "tools"))

from valtoryn_facility.coordinator import (  # noqa: E402
    FleetCoordinator,
    OrderState,
)
from valtoryn_facility.dispatch import (  # noqa: E402
    Capability,
    OrderRequest,
    RobotState,
    select_robot,
)
from valtoryn_facility.exporters import (  # noqa: E402
    apply_horizon_update,
    from_nav2_geojson,
    to_lif,
    to_nav2_geojson,
    to_rmf_nav_graph,
    to_vda5050_order,
)
from valtoryn_facility.facility_model import (  # noqa: E402
    FacilityMap,
    FacilityValidationError,
    load_facility,
    load_schema,
    point_in_polygon,
    schema_errors,
)
from valtoryn_facility.sim import FleetSimulator  # noqa: E402

import check_facility  # noqa: E402

FACILITY_PATH = ROOT / "facilities" / "warehouse_ref.yaml"
MOVE_CAPS = ("navigation.autonomous", "traffic.route_following")


def facility_document() -> dict:
    import yaml

    return yaml.safe_load(FACILITY_PATH.read_text(encoding="utf-8"))


def robot(
    robot_id: str,
    at_node: str,
    *,
    capabilities: tuple[str, ...] = MOVE_CAPS,
    verified: bool = True,
    battery: float = 1.0,
    available: bool = True,
) -> RobotState:
    caps = {
        name: Capability(
            name=name,
            kind="navigation",
            verified=verified,
            verification_ref="sim:test" if verified else "",
        )
        for name in capabilities
    }
    return RobotState(
        robot_id=robot_id,
        capabilities=caps,
        at_node=at_node,
        battery=battery,
        available=available,
    )


def move_order(order_id: str, pickup: str, dropoff: str, **kwargs) -> OrderRequest:
    return OrderRequest(
        order_id=order_id,
        idempotency_key=f"key:{order_id}",
        task_type="move_pallet",
        required_capabilities=MOVE_CAPS,
        pickup_node=pickup,
        dropoff_node=dropoff,
        **kwargs,
    )


class FacilityModelTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def test_reference_facility_is_valid(self):
        errors, _warnings = self.facility.validate()
        self.assertEqual(errors, [])
        self.assertEqual(self.facility.name, "reference-warehouse")
        self.assertGreaterEqual(len(self.facility.nodes), 20)
        self.assertGreaterEqual(len(self.facility.assets), 5)

    def test_schema_rejects_unknown_field(self):
        doc = facility_document()
        doc["nodes"][0]["colour"] = "blue"
        errors = schema_errors(doc, load_schema())
        self.assertTrue(any("colour" in e for e in errors))

    def test_schema_rejects_malformed_polygon(self):
        doc = facility_document()
        doc["zones"][0]["polygon"] = [[0.0, 0.0], [1.0, 1.0]]
        errors = schema_errors(doc, load_schema())
        self.assertTrue(errors)

    def test_duplicate_node_id_detected(self):
        doc = facility_document()
        doc["nodes"].append(dict(doc["nodes"][0]))
        errors, _ = FacilityMap.from_dict(doc).validate()
        self.assertTrue(any("duplicate node id" in e for e in errors))

    def test_unknown_edge_endpoint_detected(self):
        doc = facility_document()
        doc["edges"].append(
            {"id": "e_broken", "start_node": "pick_1", "end_node": "nowhere"}
        )
        errors, _ = FacilityMap.from_dict(doc).validate()
        self.assertTrue(any("unknown node 'nowhere'" in e for e in errors))

    def test_asset_with_unknown_parent_detected(self):
        doc = facility_document()
        doc["assets"][0]["parent_node"] = "missing_node"
        errors, _ = FacilityMap.from_dict(doc).validate()
        self.assertTrue(any("unknown parent node" in e for e in errors))

    def test_speed_limit_zone_requires_speed(self):
        doc = facility_document()
        doc["zones"][0].pop("speed_limit")
        errors, _ = FacilityMap.from_dict(doc).validate()
        self.assertTrue(any("needs a positive speed_limit" in e for e in errors))

    def test_node_inside_keepout_is_an_error(self):
        doc = facility_document()
        doc["nodes"].append(
            {
                "id": "inside_pallet",
                "node_type": "waypoint",
                "pose": {"x": 0.5, "y": 0.45, "theta": 0.0},
            }
        )
        errors, _ = FacilityMap.from_dict(doc).validate()
        self.assertTrue(any("keep-out" in e for e in errors))

    def test_load_rejects_broken_document(self):
        import tempfile
        import yaml

        doc = facility_document()
        doc["edges"][0]["end_node"] = "nope"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.yaml"
            path.write_text(yaml.safe_dump(doc), encoding="utf-8")
            with self.assertRaises(FacilityValidationError):
                load_facility(path)

    def test_round_trip_document(self):
        again = FacilityMap.from_dict(self.facility.to_dict())
        self.assertEqual(set(again.nodes), set(self.facility.nodes))
        self.assertEqual(set(again.edges), set(self.facility.edges))
        self.assertEqual(again.facility_map_version, self.facility.facility_map_version)

    def test_point_in_polygon(self):
        square = [(0.0, 0.0), (2.0, 0.0), (2.0, 2.0), (0.0, 2.0)]
        self.assertTrue(point_in_polygon(1.0, 1.0, square))
        self.assertFalse(point_in_polygon(3.0, 1.0, square))
        self.assertFalse(point_in_polygon(1.0, 1.0, [(0, 0), (1, 1)]))


class RoutingTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def test_route_exists_and_uses_lane_graph(self):
        route = self.facility.shortest_path("pick_1", "drop_1")
        self.assertIsNotNone(route)
        assert route is not None
        self.assertEqual(route.node_ids[0], "pick_1")
        self.assertEqual(route.node_ids[-1], "drop_1")
        self.assertGreater(route.length, 10.0)
        self.assertGreater(route.duration, 0.0)
        self.assertEqual(len(route.edge_ids), len(route.node_ids) - 1)

    def test_central_aisle_is_slower_than_north_aisle(self):
        central = self.facility.shortest_path("west_c", "east_c")
        north = self.facility.shortest_path("west_n", "east_n")
        assert central is not None and north is not None
        central_speed = central.length / central.duration
        north_speed = north.length / north.duration
        self.assertLess(central_speed, north_speed)
        # Lanes whose midpoint lies inside the speed-limit zone run at its cap;
        # the route average also includes the unrestricted hall approaches.
        self.assertAlmostEqual(
            self.facility.effective_speed(self.facility.edges["e_aislewc_cmid"]),
            0.5,
            places=3,
        )
        self.assertAlmostEqual(
            self.facility.effective_speed(self.facility.edges["e_aislewn_nmid"]),
            0.8,
            places=3,
        )

    def test_active_blocked_zone_reroutes_traffic(self):
        eastbound = self.facility.shortest_path("west_n", "east_n")
        assert eastbound is not None
        self.assertIn("aisle_n_mid", eastbound.node_ids)

        zone = self.facility.zones["zone_north_maintenance"]
        self.facility.zones["zone_north_maintenance"] = type(zone)(
            id=zone.id,
            zone_type=zone.zone_type,
            polygon=zone.polygon,
            speed_limit=zone.speed_limit,
            active=True,
        )
        rerouted = self.facility.shortest_path("west_n", "east_n")
        self.assertIsNotNone(rerouted)
        assert rerouted is not None
        self.assertNotIn("aisle_n_mid", rerouted.node_ids)
        self.assertNotEqual(rerouted.node_ids, eastbound.node_ids)

    def test_unknown_node_raises(self):
        with self.assertRaises(KeyError):
            self.facility.shortest_path("pick_1", "not_a_node")

    def test_route_to_unreachable_node_returns_none(self):
        isolated = FacilityMap.from_dict(
            {
                "schema_version": "1.0.0",
                "name": "split",
                "facility_map_version": 1,
                "frame": "map",
                "nodes": [
                    {"id": "a", "node_type": "junction", "pose": {"x": 0, "y": 0}},
                    {"id": "b", "node_type": "junction", "pose": {"x": 1, "y": 0}},
                    {"id": "island", "node_type": "junction", "pose": {"x": 9, "y": 9}},
                ],
                "edges": [{"id": "e", "start_node": "a", "end_node": "b"}],
                "zones": [],
                "assets": [],
            }
        )
        self.assertIsNone(isolated.shortest_path("a", "island"))


class ExporterTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def test_nav2_geojson_round_trip(self):
        exported = to_nav2_geojson(self.facility)
        self.assertEqual(exported["type"], "FeatureCollection")
        imported = from_nav2_geojson(exported)
        self.assertEqual(set(imported.nodes), set(self.facility.nodes))
        self.assertEqual(set(imported.edges), set(self.facility.edges))
        self.assertEqual(set(imported.zones), set(self.facility.zones))
        self.assertEqual(set(imported.assets), set(self.facility.assets))
        self.assertEqual(
            imported.facility_map_version, self.facility.facility_map_version
        )
        self.assertEqual(imported.zones["zone_central_slow"].speed_limit, 0.5)
        self.assertFalse(imported.zones["zone_north_maintenance"].active)
        node = imported.nodes["pick_1"]
        self.assertAlmostEqual(node.pose.x, -7.0)

    def test_rmf_graph_export(self):
        graph = to_rmf_nav_graph(self.facility)
        level = graph["levels"][self.facility.frame]
        self.assertEqual(len(level["vertices"]), len(self.facility.nodes))
        self.assertEqual(len(level["lanes"]), len(self.facility.edges))
        names = {v["name"] for v in level["vertices"]}
        self.assertIn("drop_1", names)
        lane = next(l for l in level["lanes"] if l["edge_id"] == "e_aislewc_cmid")
        self.assertAlmostEqual(lane["max_velocity"], 0.5)

    def test_lif_export_indices_resolve(self):
        lif = to_lif(self.facility)
        names = {v["index"]: v["name"] for v in lif["vertices"]}
        self.assertEqual(len(names), len(self.facility.nodes))
        for lane in lif["lanes"]:
            self.assertIn(lane["start_vertex"], names)
            self.assertIn(lane["end_vertex"], names)

    def test_vda5050_order_released_base_and_horizon(self):
        route = self.facility.shortest_path("pick_1", "drop_1")
        assert route is not None
        order = to_vda5050_order(
            self.facility, "order-1", route.node_ids, released_count=3
        )
        self.assertEqual(order["orderId"], "order-1")
        self.assertEqual(order["orderUpdateId"], 0)
        self.assertEqual([n["sequenceId"] for n in order["nodes"]], list(range(len(route.node_ids))))
        self.assertEqual([n["released"] for n in order["nodes"][:3]], [True, True, True])
        self.assertFalse(any(n["released"] for n in order["nodes"][3:]))
        self.assertEqual(len(order["edges"]), len(route.node_ids) - 1)
        self.assertEqual(order["nodes"][0]["nodePosition"]["mapId"], "map")

        updated = apply_horizon_update(order, len(route.node_ids))
        self.assertEqual(updated["orderUpdateId"], 1)
        self.assertTrue(all(n["released"] for n in updated["nodes"]))
        self.assertTrue(all(e["released"] for e in updated["edges"]))
        self.assertFalse(any(n["released"] for n in order["nodes"][3:]))

    def test_vda5050_horizon_cannot_withdraw_nodes(self):
        route = self.facility.shortest_path("pick_1", "drop_1")
        assert route is not None
        order = to_vda5050_order(
            self.facility, "order-1", route.node_ids, released_count=4
        )
        with self.assertRaises(ValueError):
            apply_horizon_update(order, 2)

    def test_vda5050_single_node_route_rejected(self):
        with self.assertRaises(ValueError):
            to_vda5050_order(self.facility, "order-1", ["pick_1"])


class DispatchTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def test_assigns_closest_capable_robot(self):
        robots = {
            "amr_01": robot("amr_01", "pick_1"),
            "amr_02": robot("amr_02", "drop_2"),
        }
        assignment = select_robot(move_order("o1", "pick_1", "drop_1"), robots, self.facility)
        self.assertFalse(isinstance(assignment, type(None)))
        self.assertEqual(assignment.robot_id, "amr_01")
        self.assertEqual(assignment.node_ids[0], "pick_1")
        self.assertEqual(assignment.node_ids[-1], "drop_1")

    def test_task_without_requirements_is_denied(self):
        order = OrderRequest(
            order_id="o1",
            idempotency_key="k1",
            task_type="vague",
            required_capabilities=(),
            pickup_node="pick_1",
            dropoff_node="drop_1",
        )
        result = select_robot(order, {"amr_01": robot("amr_01", "pick_1")}, self.facility)
        self.assertEqual(result.reason, "UNDECLARED_TASK_REQUIREMENTS")

    def test_missing_capability_is_denied_with_detail(self):
        order = OrderRequest(
            order_id="o1",
            idempotency_key="k1",
            task_type="replenish",
            required_capabilities=("payload.pick_place",),
            pickup_node="pick_1",
            dropoff_node="drop_1",
        )
        result = select_robot(order, {"amr_01": robot("amr_01", "pick_1")}, self.facility)
        self.assertEqual(result.reason, "NO_ELIGIBLE_ROBOT")
        self.assertIn("MISSING_CAPABILITY", result.detail)

    def test_unverified_capability_is_denied_by_default(self):
        order = OrderRequest(
            order_id="o1",
            idempotency_key="k1",
            task_type="replenish",
            required_capabilities=("payload.pick_place",),
            pickup_node="pick_1",
            dropoff_node="drop_1",
        )
        unverified = robot(
            "amr_01", "pick_1", capabilities=("payload.pick_place",), verified=False
        )
        result = select_robot(order, {"amr_01": unverified}, self.facility)
        self.assertEqual(result.reason, "NO_ELIGIBLE_ROBOT")
        self.assertIn("UNVERIFIED_CAPABILITY", result.detail)

        permissive = select_robot(
            OrderRequest(
                order_id="o2",
                idempotency_key="k2",
                task_type="replenish",
                required_capabilities=("payload.pick_place",),
                pickup_node="pick_1",
                dropoff_node="drop_1",
                require_verified=False,
            ),
            {"amr_01": unverified},
            self.facility,
        )
        self.assertEqual(permissive.robot_id, "amr_01")

    def test_low_battery_and_unavailable_robots_are_skipped(self):
        robots = {
            "amr_01": robot("amr_01", "pick_1", battery=0.1),
            "amr_02": robot("amr_02", "pick_1", available=False),
            "amr_03": robot("amr_03", "pick_1"),
        }
        result = select_robot(move_order("o1", "pick_1", "drop_2"), robots, self.facility)
        self.assertEqual(result.robot_id, "amr_03")

    def test_invalid_nodes_rejected(self):
        order = move_order("o1", "pick_1", "missing")
        result = select_robot(order, {"amr_01": robot("amr_01", "pick_1")}, self.facility)
        self.assertEqual(result.reason, "INVALID_ORDER")


class CoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def test_idempotent_submission(self):
        coordinator = FleetCoordinator(
            self.facility, {"amr_01": robot("amr_01", "pick_1")}
        )
        first = coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        second = coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        self.assertIs(first, second)
        self.assertEqual(len(coordinator.orders), 1)

    def test_conflicting_order_id_raises(self):
        coordinator = FleetCoordinator(
            self.facility, {"amr_01": robot("amr_01", "pick_1")}
        )
        coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        other = OrderRequest(
            order_id="o1",
            idempotency_key="different-key",
            task_type="move_pallet",
            required_capabilities=MOVE_CAPS,
            pickup_node="pick_1",
            dropoff_node="drop_1",
        )
        with self.assertRaises(ValueError):
            coordinator.submit(other)

    def test_whole_route_reservation_queues_second_robot(self):
        robots = {
            "amr_01": robot("amr_01", "west_c"),
            "amr_02": robot("amr_02", "west_c"),
        }
        coordinator = FleetCoordinator(self.facility, robots)
        first = coordinator.submit(move_order("o1", "west_c", "drop_1"))
        second = coordinator.submit(move_order("o2", "west_c", "drop_2"))
        started = coordinator.assign_pending()
        self.assertEqual(len(started), 1)
        self.assertEqual(first.state, OrderState.IN_PROGRESS)
        self.assertEqual(second.state, OrderState.PENDING)
        self.assertIn("queued", second.history[-1])

        report = FleetSimulator(coordinator).run(max_seconds=600)
        self.assertTrue(report["converged"])
        self.assertEqual(report["violations"], [])
        self.assertEqual(report["orders"][0]["state"], OrderState.COMPLETED.value)
        self.assertEqual(report["orders"][1]["state"], OrderState.COMPLETED.value)
        self.assertEqual(coordinator.node_owner, {})
        self.assertEqual(coordinator.edge_owner, {})

    def test_ambiguous_outcome_needs_verification(self):
        coordinator = FleetCoordinator(
            self.facility, {"amr_01": robot("amr_01", "pick_1")}
        )
        order = coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        coordinator.assign_pending()
        coordinator.report_outcome("o1", None, t=1.0)
        self.assertEqual(order.state, OrderState.NEEDS_VERIFICATION)
        self.assertIsNone(order.completed_at)
        self.assertTrue(coordinator.robots["amr_01"].available)
        self.assertEqual(coordinator.node_owner, {})

        coordinator.report_outcome("o1", True, t=2.0)
        self.assertEqual(order.state, OrderState.COMPLETED)
        self.assertIsNotNone(order.completed_at)

    def test_order_survives_comms_loss(self):
        coordinator = FleetCoordinator(
            self.facility, {"amr_01": robot("amr_01", "pick_1")}
        )
        order = coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        coordinator.assign_pending()
        simulator = FleetSimulator(coordinator)
        simulator.step()
        self.assertEqual(order.state, OrderState.IN_PROGRESS)

        coordinator.go_offline("amr_01", t=simulator.time)
        self.assertTrue(order.comms_lost)
        # Robot keeps executing its released base while disconnected.
        simulator.run(max_seconds=600)
        self.assertEqual(order.state, OrderState.IN_PROGRESS)
        self.assertIsNone(order.completed_at)

        coordinator.go_online("amr_01", t=simulator.time)
        self.assertEqual(order.state, OrderState.COMPLETED)
        self.assertTrue(order.comms_lost)
        self.assertIn("reconciled after reconnect", order.history[-1])

    def test_default_deny_recorded_for_payload_order(self):
        robots = {
            "amr_01": robot("amr_01", "pick_1"),
            "amr_02": robot("amr_02", "pick_2"),
        }
        coordinator = FleetCoordinator(self.facility, robots)
        order = coordinator.submit(
            OrderRequest(
                order_id="p1",
                idempotency_key="p1",
                task_type="replenish",
                required_capabilities=("payload.pick_place",),
                pickup_node="pick_1",
                dropoff_node="drop_1",
            )
        )
        coordinator.assign_pending()
        self.assertEqual(order.state, OrderState.REJECTED)
        self.assertEqual(order.rejection_reason, "NO_ELIGIBLE_ROBOT")
        self.assertIn("MISSING_CAPABILITY", order.rejection_detail)


class FleetSimulationTests(unittest.TestCase):
    def setUp(self):
        self.facility = load_facility(FACILITY_PATH)

    def _fleet(self):
        return {
            "amr_01": robot("amr_01", "pick_1"),
            "amr_02": robot("amr_02", "pick_2"),
        }

    def test_two_robots_complete_independent_missions(self):
        coordinator = FleetCoordinator(self.facility, self._fleet())
        coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        coordinator.submit(move_order("o2", "pick_2", "drop_2"))

        report = FleetSimulator(coordinator).run(max_seconds=600)

        self.assertTrue(report["converged"])
        self.assertEqual(report["violations"], [])
        states = {o["order_id"]: o["state"] for o in report["orders"]}
        self.assertEqual(states, {"o1": "COMPLETED", "o2": "COMPLETED"})
        self.assertEqual(
            {o["order_id"]: o["robot_id"] for o in report["orders"]},
            {"o1": "amr_01", "o2": "amr_02"},
        )
        # Both robots actually moved and ended at their drop stations.
        self.assertGreater(report["robots"]["amr_01"]["distance"], 10.0)
        self.assertGreater(report["robots"]["amr_02"]["distance"], 10.0)
        self.assertEqual(report["robots"]["amr_01"]["at_node"], "drop_1")
        self.assertEqual(report["robots"]["amr_02"]["at_node"], "drop_2")

    def test_simulation_is_deterministic(self):
        def run_once():
            coordinator = FleetCoordinator(self.facility, self._fleet())
            coordinator.submit(move_order("o1", "pick_1", "drop_1"))
            coordinator.submit(move_order("o2", "pick_2", "drop_2"))
            report = FleetSimulator(coordinator).run(max_seconds=600)
            return report["sim_time"], [
                (o["order_id"], o["duration"]) for o in report["orders"]
            ]

        self.assertEqual(run_once(), run_once())

    def test_reservations_are_released_after_completion(self):
        coordinator = FleetCoordinator(self.facility, self._fleet())
        coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        FleetSimulator(coordinator).run(max_seconds=600)
        self.assertEqual(coordinator.node_owner, {})
        self.assertEqual(coordinator.edge_owner, {})
        self.assertTrue(all(r.available for r in coordinator.robots.values()))

    def test_no_two_robots_share_a_lane(self):
        """Disjoint routes: the violation detector must stay silent."""
        coordinator = FleetCoordinator(self.facility, self._fleet())
        coordinator.submit(move_order("o1", "pick_1", "drop_1"))
        coordinator.submit(move_order("o2", "pick_2", "drop_2"))
        simulator = FleetSimulator(coordinator)
        simulator.run(max_seconds=600)
        self.assertEqual(simulator.violations, [])


class FacilityCheckToolTests(unittest.TestCase):
    def test_reference_document_passes(self):
        import yaml

        schema = yaml.safe_load(
            (ROOT / "schemas" / "facility-map.schema.json").read_text(encoding="utf-8")
        )
        errors, _warnings = check_facility.check_document(FACILITY_PATH, schema)
        self.assertEqual(errors, [])

    def test_broken_document_is_reported(self):
        import tempfile
        import yaml

        schema = yaml.safe_load(
            (ROOT / "schemas" / "facility-map.schema.json").read_text(encoding="utf-8")
        )
        doc = facility_document()
        doc["edges"][0]["start_node"] = "ghost"
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "broken.yaml"
            path.write_text(yaml.safe_dump(doc), encoding="utf-8")
            errors, _warnings = check_facility.check_document(path, schema)
            self.assertTrue(any("ghost" in e for e in errors))


if __name__ == "__main__":
    unittest.main()
