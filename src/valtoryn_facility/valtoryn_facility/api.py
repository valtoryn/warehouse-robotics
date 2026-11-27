#!/usr/bin/env python3
"""WMS-facing task/order API skeleton over the fleet coordinator (FR-F7).

The facility layer so far had no external task surface: orders could only be
submitted by importing the coordinator. This module adds the smallest honest
one:

* ``OrderService`` — transport-agnostic order intake, query, dispatch,
  outcome/verification reporting and robot execution callbacks over a
  ``FleetCoordinator``. All rules (default-deny dispatch, whole-route
  reservation, idempotency, ``NEEDS_VERIFICATION`` parking, store-and-forward
  after comms loss) stay in the coordinator; this layer maps requests and
  errors onto a JSON contract.
* ``FleetApiServer`` / ``make_server`` — a JSON/HTTP binding of that service on
  the Python standard library. No third-party web framework was added.

Contract (all under ``/api/v1``):

============================  =================================================
``GET  /health``              Liveness and facility map version.
``GET  /facility``            Facility summary (version, frame, counts).
``POST /orders``              Submit an order; requires ``Idempotency-Key``.
``GET  /orders``              List orders, optional ``?state=`` filter.
``GET  /orders/{id}``         One order.
``POST /orders/{id}/outcome`` Report a pick/drop outcome (true|false|null).
``POST /orders/{id}/verification``
                              Resolve an ambiguous outcome (true|false).
``POST /dispatch``            Dispatch every order that can start now.
``GET  /robots``              Robot roster with capability status.
``GET  /robots/{id}``         One robot.
``POST /robots/{id}/position``
                              Robot gateway callback: reached a route node.
``POST /robots/{id}/completed``
                              Robot gateway callback: route finished.
``POST /robots/{id}/connectivity``
                              Robot gateway callback: link lost/restored.
``GET  /snapshot``            Full coordinator snapshot.
============================  =================================================

Status: implemented software skeleton (unit-tested offline over real HTTP on
loopback). It is **not** a production service and makes no interoperability
claim: there is no authentication, no TLS, no persistence, no rate limiting,
no clock synchronisation and no WMS adapter. The contract has not been
validated with an actual WMS, and no Open-RMF or VDA 5050 transport is
involved.
"""
from __future__ import annotations

import json
import re
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable
from urllib.parse import parse_qs, urlparse

from .coordinator import FleetCoordinator, Order, OrderState
from .dispatch import OrderRequest, RobotState
from .facility_model import FacilityMap

API_PREFIX = "/api/v1"
API_VERSION = "v1"
MAX_BODY_BYTES = 1 << 20
ROBOT_NOT_FOUND = "ROBOT_NOT_FOUND"
ORDER_NOT_FOUND = "ORDER_NOT_FOUND"


class ApiError(Exception):
    """A request the API refuses, with the HTTP status and stable code."""

    def __init__(self, status: int, code: str, detail: str) -> None:
        self.status = status
        self.code = code
        self.detail = detail
        super().__init__(f"{status} {code}: {detail}")

    def payload(self) -> dict[str, Any]:
        return {"error": {"code": self.code, "detail": self.detail}}


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


class OrderService:
    """Order intake and execution service over one coordinator.

    Every public method is thread-safe: the HTTP server is threaded and the
    coordinator is not. State lives in the process (the coordinator's
    dictionaries); nothing is persisted.
    """

    def __init__(
        self,
        facility: FacilityMap,
        coordinator: FleetCoordinator,
        clock: Callable[[], float] | None = None,
        order_prefix: str = "ord",
    ) -> None:
        self.facility = facility
        self.coordinator = coordinator
        self._clock = clock or time.monotonic
        self._order_prefix = order_prefix
        self._lock = threading.RLock()
        self._order_seq = 0
        self._keys: dict[str, str] = {}

    # ---- helpers ------------------------------------------------------
    def now(self) -> float:
        return self._clock()

    def _time(self, body: dict[str, Any]) -> float:
        if "time" not in body:
            return self.now()
        value = body["time"]
        if not _is_number(value):
            raise ApiError(400, "INVALID_REQUEST", "'time' must be a number")
        return float(value)

    def _order(self, order_id: str) -> Order:
        for order in self.coordinator.orders:
            if order.order_id == order_id:
                return order
        raise ApiError(404, ORDER_NOT_FOUND, f"unknown order {order_id!r}")

    def _robot(self, robot_id: str) -> RobotState:
        robot = self.coordinator.robots.get(robot_id)
        if robot is None:
            raise ApiError(404, ROBOT_NOT_FOUND, f"unknown robot {robot_id!r}")
        return robot

    def _next_order_id(self) -> str:
        existing = {order.order_id for order in self.coordinator.orders}
        while True:
            self._order_seq += 1
            candidate = f"{self._order_prefix}-{self._order_seq:04d}"
            if candidate not in existing:
                return candidate

    @staticmethod
    def _string(body: dict[str, Any], name: str, required: bool = True) -> str:
        value = body.get(name)
        if value is None:
            if required:
                raise ApiError(400, "INVALID_REQUEST", f"'{name}' is required")
            return ""
        if not isinstance(value, str) or not value.strip():
            raise ApiError(400, "INVALID_REQUEST", f"'{name}' must be a non-empty string")
        return value.strip()

    # ---- read-only ----------------------------------------------------
    def health(self) -> dict[str, Any]:
        return {
            "status": "ok",
            "api_version": API_VERSION,
            "facility_map_version": self.facility.facility_map_version,
            "orders": len(self.coordinator.orders),
        }

    def facility_summary(self) -> dict[str, Any]:
        return {
            "name": self.facility.name,
            "frame": self.facility.frame,
            "facility_map_version": self.facility.facility_map_version,
            "schema_version": self.facility.schema_version,
            "default_speed": self.facility.default_speed,
            "counts": {
                "nodes": len(self.facility.nodes),
                "edges": len(self.facility.edges),
                "zones": len(self.facility.zones),
                "assets": len(self.facility.assets),
            },
            "nodes": sorted(self.facility.nodes),
        }

    def _robot_view(self, robot: RobotState) -> dict[str, Any]:
        return {
            "robot_id": robot.robot_id,
            "at_node": robot.at_node,
            "available": robot.available,
            "battery": round(robot.battery, 4),
            "offline": robot.robot_id in self.coordinator.offline,
            "capabilities": [
                {
                    "name": capability.name,
                    "kind": capability.kind,
                    "verified": capability.verified,
                    "verification_ref": capability.verification_ref,
                }
                for capability in sorted(
                    robot.capabilities.values(), key=lambda c: c.name
                )
            ],
        }

    def robots(self) -> list[dict[str, Any]]:
        with self._lock:
            return [
                self._robot_view(self.coordinator.robots[robot_id])
                for robot_id in sorted(self.coordinator.robots)
            ]

    def robot(self, robot_id: str) -> dict[str, Any]:
        with self._lock:
            return self._robot_view(self._robot(robot_id))

    def orders(self, state: str | None = None) -> list[dict[str, Any]]:
        with self._lock:
            if state is not None:
                if state not in {value.value for value in OrderState}:
                    raise ApiError(
                        400,
                        "INVALID_REQUEST",
                        f"unknown order state {state!r}; expected one of "
                        f"{sorted(value.value for value in OrderState)}",
                    )
                return [
                    order.snapshot()
                    for order in self.coordinator.orders
                    if order.state.value == state
                ]
            return [order.snapshot() for order in self.coordinator.orders]

    def order(self, order_id: str) -> dict[str, Any]:
        with self._lock:
            return self._order(order_id).snapshot()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "facility": {
                    "name": self.facility.name,
                    "facility_map_version": self.facility.facility_map_version,
                },
                **self.coordinator.snapshot(),
            }

    # ---- order intake --------------------------------------------------
    def submit_order(
        self, body: dict[str, Any], idempotency_key: str | None = None
    ) -> tuple[dict[str, Any], bool]:
        """Create (or replay) an order. Returns (snapshot, created)."""
        with self._lock:
            body_key = body.get("idempotency_key")
            if body_key is not None and not isinstance(body_key, str):
                raise ApiError(
                    400, "INVALID_REQUEST", "'idempotency_key' must be a string"
                )
            body_key = (body_key or "").strip()
            header_key = (idempotency_key or "").strip()
            if header_key and body_key and header_key != body_key:
                raise ApiError(
                    400,
                    "IDEMPOTENCY_KEY_MISMATCH",
                    "Idempotency-Key header and 'idempotency_key' field differ",
                )
            key = header_key or body_key
            if not key:
                raise ApiError(
                    400,
                    "MISSING_IDEMPOTENCY_KEY",
                    "POST /orders requires an Idempotency-Key header "
                    "(or an 'idempotency_key' field)",
                )

            task_type = self._string(body, "task_type")
            pickup_node = self._string(body, "pickup_node")
            dropoff_node = self._string(body, "dropoff_node")
            for label, node_id in (("pickup_node", pickup_node), ("dropoff_node", dropoff_node)):
                if node_id not in self.facility.nodes:
                    raise ApiError(
                        400, "INVALID_REQUEST", f"unknown {label} {node_id!r}"
                    )

            raw_capabilities = body.get("required_capabilities", [])
            if not isinstance(raw_capabilities, list) or any(
                not isinstance(item, str) or not item.strip()
                for item in raw_capabilities
            ):
                raise ApiError(
                    400,
                    "INVALID_REQUEST",
                    "'required_capabilities' must be a list of non-empty strings",
                )
            require_verified = body.get("require_verified", True)
            if not isinstance(require_verified, bool):
                raise ApiError(
                    400, "INVALID_REQUEST", "'require_verified' must be a boolean"
                )
            priority = body.get("priority", 0)
            if not isinstance(priority, int) or isinstance(priority, bool):
                raise ApiError(400, "INVALID_REQUEST", "'priority' must be an integer")

            order_id = self._string(body, "order_id", required=False)
            created = key not in self._keys
            request = OrderRequest(
                order_id=order_id or self._next_order_id(),
                idempotency_key=key,
                task_type=task_type,
                required_capabilities=tuple(
                    item.strip() for item in raw_capabilities
                ),
                pickup_node=pickup_node,
                dropoff_node=dropoff_node,
                require_verified=require_verified,
                priority=priority,
            )
            try:
                order = self.coordinator.submit(request, self._time(body))
            except ValueError as exc:
                raise ApiError(409, "ORDER_CONFLICT", str(exc)) from exc
            if created:
                self._keys[key] = order.order_id
            return order.snapshot(), created

    def dispatch(self, body: dict[str, Any]) -> dict[str, Any]:
        """Dispatch every pending order that can start now."""
        with self._lock:
            t = self._time(body)
            started = self.coordinator.assign_pending(t)
            return {
                "started": [order.snapshot() for order in started],
                "pending": len(self.coordinator.pending_orders()),
                "orders": [order.snapshot() for order in self.coordinator.orders],
            }

    # ---- outcome reporting ---------------------------------------------
    def report_outcome(self, order_id: str, body: dict[str, Any]) -> dict[str, Any]:
        """Report a pick/drop outcome: ``true``, ``false`` or ``null`` (ambiguous)."""
        with self._lock:
            self._order(order_id)
            if "confirmed" not in body:
                raise ApiError(
                    400,
                    "MISSING_OUTCOME",
                    "body needs 'confirmed': true, false, or null",
                )
            confirmed = body["confirmed"]
            if confirmed is not None and not isinstance(confirmed, bool):
                raise ApiError(
                    400, "INVALID_REQUEST", "'confirmed' must be true, false, or null"
                )
            try:
                order = self.coordinator.report_outcome(
                    order_id, confirmed, self._time(body)
                )
            except ValueError as exc:
                raise ApiError(409, "INVALID_ORDER_STATE", str(exc)) from exc
            return order.snapshot()

    def resolve_verification(
        self, order_id: str, body: dict[str, Any]
    ) -> dict[str, Any]:
        """Resolve an order parked in ``NEEDS_VERIFICATION``."""
        with self._lock:
            order = self._order(order_id)
            confirmed = body.get("confirmed")
            if not isinstance(confirmed, bool):
                raise ApiError(
                    400,
                    "INVALID_VERIFICATION",
                    "'confirmed' must be true or false; an ambiguous outcome "
                    "cannot itself resolve verification",
                )
            if order.state is not OrderState.NEEDS_VERIFICATION:
                raise ApiError(
                    409,
                    "INVALID_ORDER_STATE",
                    f"order {order_id!r} is {order.state.value}, not "
                    "NEEDS_VERIFICATION",
                )
            try:
                resolved = self.coordinator.report_outcome(
                    order_id, confirmed, self._time(body)
                )
            except ValueError as exc:
                raise ApiError(409, "INVALID_ORDER_STATE", str(exc)) from exc
            return resolved.snapshot()

    # ---- robot execution callbacks -------------------------------------
    def robot_position(self, robot_id: str, body: dict[str, Any]) -> dict[str, Any]:
        with self._lock:
            robot = self._robot(robot_id)
            node_id = self._string(body, "node")
            if node_id not in self.facility.nodes:
                raise ApiError(400, "INVALID_REQUEST", f"unknown node {node_id!r}")
            if robot_id not in self.coordinator.routes:
                raise ApiError(
                    409,
                    "NO_ACTIVE_ROUTE",
                    f"robot {robot_id!r} holds no reserved route",
                )
            try:
                self.coordinator.advance_to(robot_id, node_id, self._time(body))
            except (KeyError, ValueError) as exc:
                raise ApiError(409, "INVALID_PROGRESS", str(exc)) from exc
            return self._robot_view(robot)

    def robot_completed(self, robot_id: str, body: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        with self._lock:
            robot = self._robot(robot_id)
            if self.coordinator.active_order(robot_id) is None:
                raise ApiError(
                    409,
                    "NO_ACTIVE_ORDER",
                    f"robot {robot_id!r} has no order in progress",
                )
            order = self.coordinator.complete_route(robot_id, self._time(body))
            if order is None:
                # The released base completed while the robot is offline; the
                # order stays open until the robot reconnects (FR-F3).
                return 202, {
                    "state": "AWAITING_RECONCILIATION",
                    "order": None,
                    "robot": self._robot_view(robot),
                }
            return 200, {"state": order.state.value, "order": order.snapshot()}

    def robot_connectivity(
        self, robot_id: str, body: dict[str, Any]
    ) -> dict[str, Any]:
        with self._lock:
            robot = self._robot(robot_id)
            online = body.get("online")
            if not isinstance(online, bool):
                raise ApiError(400, "INVALID_REQUEST", "'online' must be a boolean")
            t = self._time(body)
            if online:
                order = self.coordinator.go_online(robot_id, t)
            else:
                self.coordinator.go_offline(robot_id, t)
                order = self.coordinator.active_order(robot_id)
            return {
                "robot": self._robot_view(robot),
                "order": None if order is None else order.snapshot(),
            }


# ---- HTTP binding ------------------------------------------------------
_PREFIX = re.escape(API_PREFIX)


class FleetApiHandler(BaseHTTPRequestHandler):
    """JSON handler for :class:`OrderService`."""

    server_version = "ValtorynFleetAPI/0.1"
    protocol_version = "HTTP/1.1"

    ROUTES: tuple[tuple[str, str, str], ...] = (
        ("GET", rf"{_PREFIX}/health$", "get_health"),
        ("GET", rf"{_PREFIX}/facility$", "get_facility"),
        ("GET", rf"{_PREFIX}/orders$", "get_orders"),
        ("POST", rf"{_PREFIX}/orders$", "post_order"),
        ("GET", rf"{_PREFIX}/orders/(?P<order_id>[^/]+)$", "get_order"),
        (
            "POST",
            rf"{_PREFIX}/orders/(?P<order_id>[^/]+)/outcome$",
            "post_outcome",
        ),
        (
            "POST",
            rf"{_PREFIX}/orders/(?P<order_id>[^/]+)/verification$",
            "post_verification",
        ),
        ("POST", rf"{_PREFIX}/dispatch$", "post_dispatch"),
        ("GET", rf"{_PREFIX}/robots$", "get_robots"),
        ("GET", rf"{_PREFIX}/robots/(?P<robot_id>[^/]+)$", "get_robot"),
        (
            "POST",
            rf"{_PREFIX}/robots/(?P<robot_id>[^/]+)/position$",
            "post_position",
        ),
        (
            "POST",
            rf"{_PREFIX}/robots/(?P<robot_id>[^/]+)/completed$",
            "post_completed",
        ),
        (
            "POST",
            rf"{_PREFIX}/robots/(?P<robot_id>[^/]+)/connectivity$",
            "post_connectivity",
        ),
        ("GET", rf"{_PREFIX}/snapshot$", "get_snapshot"),
    )

    @property
    def service(self) -> OrderService:
        return self.server.service  # type: ignore[attr-defined]

    # ---- plumbing -----------------------------------------------------
    def log_message(self, format: str, *args: Any) -> None:
        log = getattr(self.server, "log", None)
        if log is not None:
            log(format % args)

    def _send(
        self,
        status: int,
        payload: dict[str, Any] | list[Any],
        headers: dict[str, str] | None = None,
    ) -> None:
        body = json.dumps(payload, indent=2, default=str).encode("utf-8") + b"\n"
        try:
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            for name, value in (headers or {}).items():
                self.send_header(name, value)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _read_json(self) -> dict[str, Any]:
        raw_length = self.headers.get("Content-Length")
        if not raw_length:
            return {}
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise ApiError(
                400, "INVALID_REQUEST", "invalid Content-Length header"
            ) from exc
        if length < 0 or length > MAX_BODY_BYTES:
            raise ApiError(
                413,
                "PAYLOAD_TOO_LARGE",
                f"request body exceeds {MAX_BODY_BYTES} bytes",
            )
        raw = self.rfile.read(length)
        if not raw.strip():
            return {}
        try:
            document = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise ApiError(400, "INVALID_JSON", str(exc)) from exc
        if not isinstance(document, dict):
            raise ApiError(
                400, "INVALID_REQUEST", "request body must be a JSON object"
            )
        return document

    def _route(self, method: str) -> None:
        path = urlparse(self.path).path
        if len(path) > 1 and path.endswith("/"):
            path = path.rstrip("/")
        query = parse_qs(urlparse(self.path).query)
        matched_path = False
        allowed: list[str] = []
        for route_method, pattern, name in self.ROUTES:
            match = re.match(pattern, path)
            if match is None:
                continue
            matched_path = True
            if route_method != method:
                allowed.append(route_method)
                continue
            try:
                body = self._read_json() if method in ("POST", "PUT", "PATCH") else {}
                handler = getattr(self, name)
                result = handler(match, body, query)
            except ApiError as exc:
                self._send(exc.status, exc.payload())
                return
            except Exception as exc:  # pragma: no cover - defensive
                self._send(
                    500,
                    {
                        "error": {
                            "code": "INTERNAL_ERROR",
                            "detail": f"{type(exc).__name__}: {exc}",
                        }
                    },
                )
                return
            if isinstance(result, tuple) and len(result) == 2 and isinstance(result[0], int):
                status, payload = result
            else:
                status, payload = 200, result
            self._send(int(status), payload)
            return

        if matched_path:
            self._send(
                405,
                {
                    "error": {
                        "code": "METHOD_NOT_ALLOWED",
                        "detail": f"{method} is not allowed on {path}",
                    }
                },
                headers={"Allow": ", ".join(sorted(set(allowed) | {"HEAD"}))},
            )
            return
        self._send(
            404,
            {
                "error": {
                    "code": "NOT_FOUND",
                    "detail": f"no route for {path}; see GET {API_PREFIX}/health",
                }
            },
        )

    def _idempotency_key(self) -> str | None:
        return self.headers.get("Idempotency-Key")

    # ---- route handlers ------------------------------------------------
    def get_health(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.health()

    def get_facility(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.facility_summary()

    def get_orders(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        state = query.get("state", [None])[0]
        orders = self.service.orders(state)
        return {"orders": orders, "count": len(orders)}

    def get_order(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.order(match.group("order_id"))

    def post_order(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> tuple[int, dict[str, Any]]:
        order, created = self.service.submit_order(body, self._idempotency_key())
        return (201 if created else 200), {
            "order": order,
            "created": created,
            "idempotent_replay": not created,
        }

    def post_outcome(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.report_outcome(match.group("order_id"), body)

    def post_verification(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.resolve_verification(match.group("order_id"), body)

    def post_dispatch(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.dispatch(body)

    def get_robots(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        robots = self.service.robots()
        return {"robots": robots, "count": len(robots)}

    def get_robot(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.robot(match.group("robot_id"))

    def post_position(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.robot_position(match.group("robot_id"), body)

    def post_completed(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> tuple[int, dict[str, Any]]:
        return self.service.robot_completed(match.group("robot_id"), body)

    def post_connectivity(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.robot_connectivity(match.group("robot_id"), body)

    def get_snapshot(
        self, match: re.Match[str], body: dict[str, Any], query: dict[str, list[str]]
    ) -> dict[str, Any]:
        return self.service.snapshot()

    # ---- verbs ---------------------------------------------------------
    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self._route("GET")

    def do_POST(self) -> None:  # noqa: N802
        self._route("POST")

    def do_HEAD(self) -> None:  # noqa: N802
        self._route("GET")

    def do_PUT(self) -> None:  # noqa: N802
        self._route("PUT")

    def do_DELETE(self) -> None:  # noqa: N802
        self._route("DELETE")


class FleetApiServer(ThreadingHTTPServer):
    """Threaded JSON server carrying the order service."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        service: OrderService,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self.service = service
        self.log = log
        super().__init__(server_address, FleetApiHandler)


def make_server(
    service: OrderService,
    host: str = "127.0.0.1",
    port: int = 8080,
    log: Callable[[str], None] | None = None,
) -> FleetApiServer:
    """Bind a server without serving; call ``serve_forever()`` (or a thread)."""
    return FleetApiServer((host, port), service, log)


def stderr_log(message: str) -> None:
    print(message, file=sys.stderr, flush=True)
