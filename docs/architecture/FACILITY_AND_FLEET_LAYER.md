# Facility model and fleet-coordination layer

> **Status:** Implemented software layer (unit-tested offline, deterministic simulation).
> Not validated in Gazebo multi-robot simulation, not validated on physical hardware, and not
> safety-rated. No Open-RMF or VDA 5050 conformance is claimed.
> **Basis:** FR-F1..FR-F7 in [PRODUCT_REQUIREMENTS.md](PRODUCT_REQUIREMENTS.md), section 9 of
> [MODULARITY_SPECIFICATION.md](MODULARITY_SPECIFICATION.md), TD-9/TD-11 in
> [TECHNOLOGY_DECISIONS.md](TECHNOLOGY_DECISIONS.md), ADR-007/ADR-008 in
> [ARCHITECTURE_DECISION_RECORDS.md](ARCHITECTURE_DECISION_RECORDS.md).

## 1. What this layer is

Phase 4 of the roadmap adds fleet intelligence (L6) above the single-robot autonomy stack. This
milestone implements the parts of that layer that can be built and verified without a second
physical or simulated robot:

| Component | Location | Status |
|---|---|---|
| Internal facility model (nodes, edges, zones, assets, map version) | `src/valtoryn_facility/valtoryn_facility/facility_model.py` | Implemented |
| Facility document schema | [schemas/facility-map.schema.json](../../schemas/facility-map.schema.json) | Implemented |
| Reference facility for the warehouse world | [facilities/warehouse_ref.yaml](../../facilities/warehouse_ref.yaml) | Implemented |
| Exporters: Nav2 GeoJSON, RMF nav graph, LIF, VDA 5050 order/horizon | `src/valtoryn_facility/valtoryn_facility/exporters.py` | Implemented |
| Capability dispatch with default-deny | `src/valtoryn_facility/valtoryn_facility/dispatch.py` | Implemented |
| Reservation/queue coordinator with order lifecycle | `src/valtoryn_facility/valtoryn_facility/coordinator.py` | Implemented |
| Deterministic two-robot coordination simulation | `src/valtoryn_facility/valtoryn_facility/sim.py` | Implemented (software model) |
| Facility validator | [tools/check_facility.py](../../tools/check_facility.py) | Implemented |
| Scenario runner | [tools/run_fleet_sim.py](../../tools/run_fleet_sim.py) | Implemented |

Still open (not implemented, not claimed): multi-robot Gazebo Harmonic bring-up, a ROS 2 node
publishing `FacilityMap` and consuming orders, the WMS-facing API, Open-RMF backend adapter, and any
VDA 5050 MQTT transport. Those need either a second simulated robot or a decision that gate G-E has
not yet produced.

## 2. Facility model

One internal, versioned facility model is the source of truth for route topology, zones and assets
(FR-F1, FR-A4). It is never stored in an external format; exporters derive views from it.

- **Nodes** carry an id, an open/extensible `node_type` (`station`, `junction`, `charging`,
  `waypoint`) and a 2D pose in the facility frame.
- **Edges** are directed traversal links with an optional `max_velocity`, a `bidirectional` flag and
  an open/extensible `edge_type`. Lane length is derived from node poses.
- **Zones** carry a closed polygon and one of the documented types (`keep_out`, `speed_limit`,
  `blocked`, `released`, `preferred_lane`, `restricted`). Types are open in the interface; this
  baseline enforces `speed_limit`, `keep_out` and `blocked`.
- **Assets** (docks, charging stations, pick/drop stations, door/lift hooks) carry their own
  `asset_version`, independently of the map version, and attach to a parent node.
- **Versioning:** the document carries `facility_map_version` (monotonic) and the schema its own
  SemVer; both are independent of software releases.

[facilities/warehouse_ref.yaml](../../facilities/warehouse_ref.yaml) describes the existing
`worlds/warehouse.sdf` twin: three longitudinal aisles at y = +2.9, 0.0 and -2.9 between four rack
rows, west and east halls, the four pick/drop stations from the world file, a charging bay, a
narrow-aisle speed limit over the central aisle and a keep-out polygon over the static pallet stack.
Node poses are engineering layout estimates [EST]; the world file remains the physics source of
truth, and the facility file the routing source of truth.

Routing is minimum-time Dijkstra over the lane graph, so a narrow-aisle speed cap costs time rather
than being expressed in metres. Active `blocked` zones remove lanes; active `keep_out` zones make a
node illegal, and the validator rejects a facility that places a node inside one.

Validation is two-stage: JSON Schema for shape and types, then structural rules in
`FacilityMap.validate()` — duplicate ids, unknown edge endpoints, self-loops, asset parents, zone
polygon size, speed-limit zones without a limit, and warnings for isolated or unreachable nodes.

## 3. Exporters

External formats are views, not storage:

- **Nav2 GeoJSON** — a `FeatureCollection` of node points, edge linestrings, zone polygons and asset
  points, with the map version in the document properties. This exporter is used for round-trip
  tests ([tests/test_facility.py](../../tests/test_facility.py)).
- **RMF navigation graph** — one level named after the facility frame, with vertices and lanes.
- **LIF** — a lane-indexed view with integer vertex/lane indices.
- **VDA 5050 order document** — nodes and edges with `sequenceId`, a `released` flag implementing the
  released-base + horizon model, and `apply_horizon_update()` for monotonic base extension. Wording
  follows the published order model; conformance testing is still blocked by gate G-E, so this is a
  document shape, not a certificate.

Known gap: the RMF and LIF exports have not been imported by the upstream tools. Their shapes follow
the intent recorded in FR-F1; verifying them against a running Open-RMF or traffic-editor instance is
open work.

## 4. Capability dispatch (default deny)

A task names the capabilities it requires. Dispatch only assigns it to a robot that:

1. declares every required capability,
2. has each of them verified when `require_verified` is set (a declared-only capability is refused),
3. is available with battery at or above the minimum, and
4. has a route to the pickup and from the pickup to the dropoff.

A task that declares no requirements is denied (`UNDECLARED_TASK_REQUIREMENTS`) rather than treated
as universally assignable. Capability failures are reported before transient state, so a task no
robot could ever perform reads as a capability gap rather than as fleet congestion. Among eligible
robots the nearest (shortest time to pickup) wins, with robot id as a deterministic tie-break.

Verification is a recorded reference on the capability, not a safety claim. Every reference this
project can honestly produce today points at simulation evidence.

## 5. Coordination and order lifecycle

The baseline is deliberately small (ADR-008) and deadlock-free by construction:

- A robot reserves **every node and lane of its whole route atomically**. If any part is held by
  another robot, the order stays queued and nothing is reserved — so no robot ever holds
  reservations while waiting for more, and no cyclic wait can form.
- Reservations are released as the robot advances, and all of them on completion, failure or
  verification parking. The cost of the all-or-nothing rule is reduced concurrency on shared aisles;
  finer-grained release/reservation and negotiation are future work.
- Order states: `PENDING` → `IN_PROGRESS` → `COMPLETED`, with `REJECTED` for default-deny or invalid
  orders, `FAILED` for a confirmed failed execution, and `NEEDS_VERIFICATION` for an ambiguous
  pick/drop outcome. An ambiguous outcome parks the order and frees the robot; it is never guessed
  into `COMPLETED`.
- Idempotency keys make re-submission of the same request return the existing order. Reusing an
  existing `order_id` with a different key is an error, not a silent duplicate.
- Comms loss: because the entire route is released to the robot at assignment, a robot that loses
  connectivity finishes its released base and reports on reconnect (FR-F3 store-and-forward). The
  order completes only during that reconciliation and records that comms were lost.

## 6. What the scenario runner verifies, and what it does not

`tools/run_fleet_sim.py` runs two reference AMRs over the facility graph: one west-to-east through
the north aisle, one west-to-east through the south aisle, plus a payload order the fleet must
refuse. Robots advance at lane speed limits in fixed steps; the coordinator records reservations and
lane ownership. Exit status is non-zero if a move order does not complete, the payload order is not
denied for the missing capability, or a lane is traversed without its reservation.

That is a **software** simulation of coordination logic. It does not run Gazebo, Nav2, `ros2_control`
or DDS, and it does not model collisions, kinematics, sensor noise or Wi-Fi behaviour. Two AMRs
completing coordinated missions in the physics twin is still open Phase 4 work, and nothing here is
evidence about the physical warehouse.

## 7. Validation performed

- `python tools/check_facility.py` — 1 document, 0 errors, 0 warnings.
- `python tools/run_fleet_sim.py` — both move orders `COMPLETED`, payload order denied with the
  missing capability cited, 0 reservation violations, report written to `generated/fleet_report.json`.
- `python -m unittest tests.test_facility -v` — 40 tests, all passing: schema and structural
  validation, routing with speed limits and blocked lanes, exporter round-trip and VDA 5050 horizon
  rules, default-deny dispatch, queueing, idempotency, verification parking, comms-loss
  reconciliation, determinism, and reservation release.

## 8. Unresolved

- Coordination algorithm beyond all-or-nothing reservation (negotiation, partial reservation,
  multi-robot Gazebo validation).
- RMF graph and LIF export verification against upstream tooling.
- VDA 5050 v3 conformance and MQTT transport (gate G-E).
- WMS-facing task/order API surface (FR-F7) and its idempotency contract with an actual WMS.
- Throughput measurement of the baseline under fleet load; no performance claim is made.
