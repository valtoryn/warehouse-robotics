#!/usr/bin/env python3
"""Serve the WMS-facing order API skeleton over the reference facility.

Wraps `valtoryn_facility.api.OrderService` in its stdlib HTTP server, using the
same reference fleet and facility the coordination simulation uses. This is a
development/demonstration surface for the order contract (FR-F7): in-memory
state, no authentication, no TLS, loopback by default. It is not a production
fleet-control service.

Order progress over HTTP follows the robot-gateway callbacks:

    POST /api/v1/orders                 submit (Idempotency-Key required)
    POST /api/v1/dispatch               dispatch whatever can start now
    POST /api/v1/robots/{id}/position   reached a node on the reserved route
    POST /api/v1/robots/{id}/completed  route finished
    POST /api/v1/orders/{id}/outcome    pick/drop confirmed, failed or ambiguous
    POST /api/v1/orders/{id}/verification   resolve an ambiguous outcome

Usage:
  python tools/serve_fleet_api.py
  python tools/serve_fleet_api.py --port 9000 --facility facilities/warehouse_ref.yaml
  python tools/serve_fleet_api.py --quiet
  python tools/serve_fleet_api.py --self-check --port 0  # serve, GET /health, exit

Then, for example:
  curl -s -X POST http://127.0.0.1:8080/api/v1/orders \
    -H 'Idempotency-Key: wms:3001:v1' -H 'Content-Type: application/json' \
    -d '{"order_id":"move-3001","task_type":"move_pallet",
         "required_capabilities":["navigation.autonomous","traffic.route_following"],
         "pickup_node":"pick_1","dropoff_node":"drop_1"}'
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "src" / "valtoryn_facility"))
sys.path.insert(0, str(ROOT / "tools"))

from valtoryn_facility.api import (  # noqa: E402
    API_PREFIX,
    FleetApiServer,
    OrderService,
    make_server,
    stderr_log,
)
from valtoryn_facility.coordinator import FleetCoordinator  # noqa: E402
from valtoryn_facility.facility_model import load_facility  # noqa: E402

from run_fleet_sim import build_fleet  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="bind address (default loopback)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--facility", type=Path, default=None, help="facility document to serve")
    parser.add_argument("--quiet", action="store_true", help="do not log each request")
    parser.add_argument(
        "--self-check",
        action="store_true",
        help="serve briefly, request /health over loopback, then exit",
    )
    args = parser.parse_args(argv)

    facility = load_facility(args.facility)
    robots = build_fleet()
    coordinator = FleetCoordinator(facility, robots)
    service = OrderService(facility, coordinator)

    try:
        server = make_server(
            service, host=args.host, port=args.port, log=None if args.quiet else stderr_log
        )
    except OSError as exc:
        print(f"ERROR: cannot bind {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 1

    if args.self_check:
        return _self_check(server)

    host, port = server.server_address[:2]
    print(f"facility : {facility.name} (map version {facility.facility_map_version})")
    print(f"robots   : {', '.join(sorted(robots))}")
    print(f"listening: http://{host}:{port}{API_PREFIX}")
    print("endpoints: /health /facility /orders /orders/{id} /orders/{id}/outcome")
    print("           /orders/{id}/verification /dispatch /robots /robots/{id}")
    print("           /robots/{id}/position /robots/{id}/completed")
    print("           /robots/{id}/connectivity /snapshot")
    print("state is in-memory only; this is a development surface, not a service contract")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nshutting down")
    finally:
        server.server_close()
    return 0


def _self_check(server: FleetApiServer) -> int:
    """Serve on loopback, request /health, and report the real response code."""
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        host, port = server.server_address[:2]
        url = f"http://{host}:{port}{API_PREFIX}/health"
        with urllib.request.urlopen(url, timeout=10) as response:
            payload = json.loads(response.read())
            status = response.status
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    print(f"self-check: GET {API_PREFIX}/health -> {status} ({payload.get('status')})")
    return 0 if status == 200 and payload.get("status") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
