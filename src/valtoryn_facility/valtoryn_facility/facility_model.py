#!/usr/bin/env python3
"""Internal, versioned facility model for the Valtoryn fleet layer (L6).

Implements docs/architecture/MODULARITY_SPECIFICATION.md section 9 and
docs/architecture/FACILITY_AND_FLEET_LAYER.md: topology (nodes, directed
edges), zones and constraints, assets with independent asset-class versions,
the facility-map version exchanged with fleet control, and the graph queries
the fleet-coordination baseline needs.

Pure Python (PyYAML + jsonschema); loading, validation, routing and export
require no ROS runtime, so the facility layer is testable on any host. The ROS
message equivalents are src/valtoryn_interfaces/msg/Facility*.msg; this module
is the internal representation those messages carry, never an external format.

Status: implemented software artifact (unit-tested offline). Not validated on
physical hardware, and no safety claim is made.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

import jsonschema
import yaml

SCHEMA_RELPATH = Path("schemas") / "facility-map.schema.json"
DEFAULT_FACILITY_RELPATH = Path("facilities") / "warehouse_ref.yaml"
DEFAULT_SPEED = 1.0

# Zone semantics recognised by this baseline. Zone types are open/extensible in
# the interface; these are the ones the coordinator actually enforces today.
SPEED_ZONE_TYPES = {"speed_limit"}
NO_ENTRY_ZONE_TYPES = {"keep_out", "blocked"}
TRAVERSAL_BLOCKING_ZONE_TYPES = {"blocked"}


class FacilityValidationError(ValueError):
    """Raised when a facility document fails schema or structural validation."""

    def __init__(self, source: str | Path, errors: Iterable[str]) -> None:
        self.source = str(source)
        self.errors = list(errors)
        detail = "\n  ".join(self.errors)
        super().__init__(f"{self.source}: {len(self.errors)} facility error(s):\n  {detail}")


def repo_root() -> Path:
    """Locate the repository root by looking for the facility schema."""
    for parent in Path(__file__).resolve().parents:
        if (parent / SCHEMA_RELPATH).exists():
            return parent
    raise FileNotFoundError(
        f"could not locate {SCHEMA_RELPATH} above {Path(__file__).resolve()}"
    )


def load_schema(path: Path | None = None) -> dict[str, Any]:
    schema_path = path or (repo_root() / SCHEMA_RELPATH)
    return yaml.safe_load(schema_path.read_text(encoding="utf-8"))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float = 0.0


@dataclass(frozen=True)
class FacilityNode:
    id: str
    node_type: str
    pose: Pose2D


@dataclass(frozen=True)
class FacilityEdge:
    id: str
    start_node: str
    end_node: str
    max_velocity: float | None = None
    bidirectional: bool = True
    edge_type: str = "lane"


@dataclass(frozen=True)
class FacilityZone:
    id: str
    zone_type: str
    polygon: tuple[tuple[float, float], ...]
    speed_limit: float | None = None
    active: bool = True


@dataclass(frozen=True)
class FacilityAsset:
    id: str
    asset_type: str
    asset_version: int
    pose: Pose2D
    parent_node: str
    available: bool = True


@dataclass(frozen=True)
class Route:
    """Ordered lane route between two facility nodes."""

    node_ids: tuple[str, ...]
    edge_ids: tuple[str, ...]
    length: float
    duration: float

    def __post_init__(self) -> None:
        if len(self.edge_ids) != max(len(self.node_ids) - 1, 0):
            raise ValueError(
                f"route has {len(self.node_ids)} nodes but {len(self.edge_ids)} edges"
            )


def point_in_polygon(x: float, y: float, polygon: Iterable[tuple[float, float]]) -> bool:
    """Even-odd ray cast; boundary behaviour is not defined and is not relied on."""
    pts = list(polygon)
    if len(pts) < 3:
        return False
    inside = False
    j = len(pts) - 1
    for i, (xi, yi) in enumerate(pts):
        xj, yj = pts[j]
        if (yi > y) != (yj > y):
            x_cross = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x < x_cross:
                inside = not inside
        j = i
    return inside


@dataclass
class FacilityMap:
    schema_version: str
    name: str
    facility_map_version: int
    frame: str
    default_speed: float
    nodes: dict[str, FacilityNode]
    edges: dict[str, FacilityEdge]
    zones: dict[str, FacilityZone]
    assets: dict[str, FacilityAsset]
    raw: dict[str, Any] = field(default_factory=dict, repr=False, compare=False)

    # ---- construction -------------------------------------------------
    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FacilityMap":
        nodes = {
            n["id"]: FacilityNode(
                id=n["id"],
                node_type=n["node_type"],
                pose=_pose(n.get("pose", {})),
            )
            for n in data.get("nodes", [])
        }
        edges = {
            e["id"]: FacilityEdge(
                id=e["id"],
                start_node=e["start_node"],
                end_node=e["end_node"],
                max_velocity=e.get("max_velocity"),
                bidirectional=bool(e.get("bidirectional", True)),
                edge_type=e.get("edge_type", "lane"),
            )
            for e in data.get("edges", [])
        }
        zones = {
            z["id"]: FacilityZone(
                id=z["id"],
                zone_type=z["zone_type"],
                polygon=tuple(
                    (float(p[0]), float(p[1])) for p in z.get("polygon", [])
                ),
                speed_limit=z.get("speed_limit"),
                active=bool(z.get("active", True)),
            )
            for z in data.get("zones", [])
        }
        assets = {
            a["id"]: FacilityAsset(
                id=a["id"],
                asset_type=a["asset_type"],
                asset_version=int(a.get("asset_version", 1)),
                pose=_pose(a.get("pose", {})),
                parent_node=a["parent_node"],
                available=bool(a.get("available", True)),
            )
            for a in data.get("assets", [])
        }
        return cls(
            schema_version=str(data.get("schema_version", "1.0.0")),
            name=str(data.get("name", "")),
            facility_map_version=int(data.get("facility_map_version", 1)),
            frame=str(data.get("frame", "map")),
            default_speed=float(data.get("default_speed", DEFAULT_SPEED)),
            nodes=nodes,
            edges=edges,
            zones=zones,
            assets=assets,
            raw=dict(data),
        )

    def to_dict(self) -> dict[str, Any]:
        """Round-trip back to the facility document shape."""
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "facility_map_version": self.facility_map_version,
            "frame": self.frame,
            "default_speed": self.default_speed,
            "nodes": [
                {
                    "id": n.id,
                    "node_type": n.node_type,
                    "pose": {"x": n.pose.x, "y": n.pose.y, "theta": n.pose.theta},
                }
                for n in self.nodes.values()
            ],
            "edges": [
                {
                    "id": e.id,
                    "start_node": e.start_node,
                    "end_node": e.end_node,
                    "max_velocity": e.max_velocity,
                    "bidirectional": e.bidirectional,
                    "edge_type": e.edge_type,
                }
                for e in self.edges.values()
            ],
            "zones": [
                {
                    "id": z.id,
                    "zone_type": z.zone_type,
                    "polygon": [[p[0], p[1]] for p in z.polygon],
                    "speed_limit": z.speed_limit,
                    "active": z.active,
                }
                for z in self.zones.values()
            ],
            "assets": [
                {
                    "id": a.id,
                    "asset_type": a.asset_type,
                    "asset_version": a.asset_version,
                    "pose": {"x": a.pose.x, "y": a.pose.y, "theta": a.pose.theta},
                    "parent_node": a.parent_node,
                    "available": a.available,
                }
                for a in self.assets.values()
            ],
        }

    # ---- topology -----------------------------------------------------
    def links(self, node_id: str) -> list[tuple[str, FacilityEdge]]:
        """All traversable-in-principle directed links out of a node."""
        out: list[tuple[str, FacilityEdge]] = []
        for edge in self.edges.values():
            if edge.start_node == node_id:
                out.append((edge.end_node, edge))
            if edge.bidirectional and edge.end_node == node_id:
                out.append((edge.start_node, edge))
        return out

    def edge_between(self, start: str, end: str) -> FacilityEdge:
        for edge in self.edges.values():
            if edge.start_node == start and edge.end_node == end:
                return edge
            if edge.bidirectional and edge.start_node == end and edge.end_node == start:
                return edge
        raise KeyError(f"no facility edge between {start!r} and {end!r}")

    def edge_length(self, edge: FacilityEdge) -> float:
        a = self.nodes[edge.start_node].pose
        b = self.nodes[edge.end_node].pose
        return math.hypot(b.x - a.x, b.y - a.y)

    def edge_midpoint(self, edge: FacilityEdge) -> tuple[float, float]:
        a = self.nodes[edge.start_node].pose
        b = self.nodes[edge.end_node].pose
        return ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)

    # ---- zones --------------------------------------------------------
    def zones_at(self, x: float, y: float, active_only: bool = True) -> list[FacilityZone]:
        return [
            z
            for z in self.zones.values()
            if (z.active or not active_only) and point_in_polygon(x, y, z.polygon)
        ]

    def speed_limit_at(self, x: float, y: float) -> float | None:
        limits = [
            z.speed_limit
            for z in self.zones_at(x, y)
            if z.zone_type in SPEED_ZONE_TYPES and z.speed_limit
        ]
        return min(limits) if limits else None

    def is_traversable(self, edge: FacilityEdge) -> bool:
        x, y = self.edge_midpoint(edge)
        for zone in self.zones_at(x, y):
            if zone.zone_type in TRAVERSAL_BLOCKING_ZONE_TYPES:
                return False
        return True

    def effective_speed(self, edge: FacilityEdge) -> float:
        """Edge speed capped by any active speed-limit zone over its midpoint."""
        speed = edge.max_velocity if edge.max_velocity is not None else self.default_speed
        x, y = self.edge_midpoint(edge)
        zone_limit = self.speed_limit_at(x, y)
        if zone_limit is not None:
            speed = min(speed, zone_limit)
        return speed

    def blocked_nodes(self) -> set[str]:
        """Nodes that sit inside an active keep-out or blocked zone."""
        out: set[str] = set()
        for node in self.nodes.values():
            for zone in self.zones_at(node.pose.x, node.pose.y):
                if zone.zone_type in NO_ENTRY_ZONE_TYPES:
                    out.add(node.id)
                    break
        return out

    def routing_blocked_nodes(self) -> set[str]:
        """Nodes unavailable for routing: declared no-entry zones plus their lanes."""
        blocked = self.blocked_nodes()
        for edge in self.edges.values():
            if not self.is_traversable(edge):
                blocked.add(edge.start_node)
                blocked.add(edge.end_node)
        return blocked

    def shortest_path(
        self,
        start: str,
        goal: str,
        blocked_nodes: Iterable[str] = (),
        blocked_edges: Iterable[str] = (),
    ) -> Route | None:
        """Minimum-time route over the lane graph (Dijkstra), or None if unreachable.

        Cost is lane travel time, so a narrow-aisle speed limit is respected
        rather than measured in metres. Reserved (blocked) nodes/edges are
        excluded, which is what the fleet coordinator's queueing relies on.
        """
        for node_id in (start, goal):
            if node_id not in self.nodes:
                raise KeyError(f"unknown facility node {node_id!r}")

        blocked = set(blocked_nodes) | self.routing_blocked_nodes()
        blocked_edge_ids = set(blocked_edges)
        if start in blocked or goal in blocked:
            return None

        dist: dict[str, float] = {start: 0.0}
        prev: dict[str, tuple[str, str]] = {}
        heap: list[tuple[float, str]] = [(0.0, start)]
        while heap:
            cost, node = heapq.heappop(heap)
            if cost > dist.get(node, math.inf) + 1e-12:
                continue
            if node == goal:
                break
            for neighbour, edge in self.links(node):
                if neighbour in blocked or edge.id in blocked_edge_ids:
                    continue
                if not self.is_traversable(edge):
                    continue
                speed = self.effective_speed(edge)
                if speed <= 0:
                    continue
                new_cost = cost + self.edge_length(edge) / speed
                if new_cost < dist.get(neighbour, math.inf) - 1e-12:
                    dist[neighbour] = new_cost
                    prev[neighbour] = (node, edge.id)
                    heapq.heappush(heap, (new_cost, neighbour))

        if goal not in dist:
            return None

        node_ids = [goal]
        edge_ids: list[str] = []
        cursor = goal
        while cursor != start:
            parent, edge_id = prev[cursor]
            edge_ids.append(edge_id)
            node_ids.append(parent)
            cursor = parent
        node_ids.reverse()
        edge_ids.reverse()

        length = sum(self.edge_length(self.edges[e]) for e in edge_ids)
        return Route(
            node_ids=tuple(node_ids),
            edge_ids=tuple(edge_ids),
            length=length,
            duration=dist[goal],
        )

    # ---- assets -------------------------------------------------------
    def assets_of_type(self, asset_type: str) -> list[FacilityAsset]:
        return [a for a in self.assets.values() if a.asset_type == asset_type]

    def asset(self, asset_id: str) -> FacilityAsset:
        return self.assets[asset_id]

    # ---- validation ---------------------------------------------------
    def validate(self) -> tuple[list[str], list[str]]:
        """Structural validation beyond the JSON schema.

        Returns (errors, warnings). Errors mean the model cannot be routed on;
        warnings are quality issues that do not block use.
        """
        errors: list[str] = []
        warnings: list[str] = []
        raw = self.raw or self.to_dict()

        if not self.name:
            errors.append("facility name is empty")
        if self.facility_map_version < 1:
            errors.append("facility_map_version must be >= 1")
        if not self.frame:
            errors.append("frame is empty")

        for kind in ("nodes", "edges", "zones", "assets"):
            ids = [item.get("id") for item in raw.get(kind, [])]
            duplicates = sorted({i for i in ids if i and ids.count(i) > 1})
            for dup in duplicates:
                errors.append(f"duplicate {kind[:-1]} id {dup!r}")

        if not self.nodes:
            errors.append("facility has no nodes")

        for edge in self.edges.values():
            if edge.start_node not in self.nodes:
                errors.append(
                    f"edge {edge.id!r} starts at unknown node {edge.start_node!r}"
                )
            if edge.end_node not in self.nodes:
                errors.append(
                    f"edge {edge.id!r} ends at unknown node {edge.end_node!r}"
                )
            if edge.start_node == edge.end_node:
                errors.append(f"edge {edge.id!r} is a self-loop")
            if not edge.edge_type:
                errors.append(f"edge {edge.id!r} has an empty edge_type")
            known_endpoints = (
                edge.start_node in self.nodes and edge.end_node in self.nodes
            )
            if known_endpoints and self.edge_length(edge) <= 1e-6:
                warnings.append(
                    f"edge {edge.id!r} has zero length (coincident node poses)"
                )

        pairs: dict[tuple[str, str], list[str]] = {}
        for edge in self.edges.values():
            pairs.setdefault((edge.start_node, edge.end_node), []).append(edge.id)
        for pair, ids in pairs.items():
            if len(ids) > 1:
                warnings.append(
                    f"nodes {pair[0]!r}->{pair[1]!r} are joined by {len(ids)} edges: {sorted(ids)}"
                )

        for zone in self.zones.values():
            if len(zone.polygon) < 3:
                errors.append(f"zone {zone.id!r} polygon has fewer than 3 points")
            if zone.zone_type in SPEED_ZONE_TYPES:
                if zone.speed_limit is None or zone.speed_limit <= 0:
                    errors.append(
                        f"speed_limit zone {zone.id!r} needs a positive speed_limit"
                    )
            elif zone.speed_limit is not None:
                warnings.append(
                    f"zone {zone.id!r} ({zone.zone_type}) declares speed_limit but is not a speed_limit zone"
                )

        for asset in self.assets.values():
            if asset.parent_node not in self.nodes:
                errors.append(
                    f"asset {asset.id!r} references unknown parent node {asset.parent_node!r}"
                )
            if asset.asset_version < 1:
                errors.append(f"asset {asset.id!r} has asset_version < 1")

        for node_id in sorted(self.blocked_nodes()):
            errors.append(
                f"node {node_id!r} lies inside an active keep-out/blocked zone"
            )

        if len(self.nodes) > 1:
            isolated = sorted(n.id for n in self.nodes.values() if not self.links(n.id))
            for node_id in isolated:
                warnings.append(f"node {node_id!r} has no edges")
            unreachable = sorted(self._unreachable_nodes())
            for node_id in unreachable:
                warnings.append(
                    f"node {node_id!r} is not reachable from the rest of the graph"
                )

        return errors, warnings

    def _unreachable_nodes(self) -> set[str]:
        if not self.nodes:
            return set()
        seed = next(iter(self.nodes))
        seen = {seed}
        stack = [seed]
        while stack:
            node = stack.pop()
            for neighbour, _ in self.links(node):
                if neighbour not in seen:
                    seen.add(neighbour)
                    stack.append(neighbour)
        return set(self.nodes) - seen


def _pose(data: dict[str, Any]) -> Pose2D:
    return Pose2D(
        x=float(data.get("x", 0.0)),
        y=float(data.get("y", 0.0)),
        theta=float(data.get("theta", 0.0)),
    )


def schema_errors(document: dict[str, Any], schema: dict[str, Any] | None = None) -> list[str]:
    validator = jsonschema.Draft202012Validator(schema or load_schema())
    errors: list[str] = []
    for error in sorted(validator.iter_errors(document), key=lambda e: list(e.path)):
        location = "/".join(str(p) for p in error.path) or "<document>"
        errors.append(f"schema: {location}: {error.message}")
    return errors


def load_facility(
    path: str | Path | None = None, schema: dict[str, Any] | None = None
) -> FacilityMap:
    """Load, schema-validate and structurally validate a facility document."""
    facility_path = Path(path) if path is not None else repo_root() / DEFAULT_FACILITY_RELPATH
    document = yaml.safe_load(facility_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise FacilityValidationError(facility_path, ["document is not a mapping"])

    errors = schema_errors(document, schema)
    facility = FacilityMap.from_dict(document)
    structural_errors, _warnings = facility.validate()
    errors.extend(structural_errors)
    if errors:
        raise FacilityValidationError(facility_path, errors)
    return facility
