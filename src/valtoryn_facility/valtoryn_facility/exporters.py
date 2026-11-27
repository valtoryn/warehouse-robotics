#!/usr/bin/env python3
"""Exporters from the internal facility model to external route formats.

FR-F1 requires one internal facility model with exporters to LIF, an RMF
navigation graph and Nav2 route-server GeoJSON; FR-F2/ADR-007 keep VDA 5050 as
an external order format only. This module implements those exports (and a
VDA 5050 v3 order/horizon builder) without ever storing the model in an
external format.

Evidence status: the exported shapes follow the documented intent and the
published VDA 5050 order model, but no exporter has been checked against an
upstream schema or a running Open-RMF / Nav2 route server. Gate G-E (VDA 5050
v3 coverage) remains open, so none of this is a conformance claim.
"""
from __future__ import annotations

import copy
from typing import Any, Sequence

from .facility_model import FacilityMap, Route


def to_nav2_geojson(facility: FacilityMap) -> dict[str, Any]:
    """Nav2 route-server shaped GeoJSON FeatureCollection (round-trips)."""
    features: list[dict[str, Any]] = []
    for node in facility.nodes.values():
        features.append(
            {
                "type": "Feature",
                "id": node.id,
                "properties": {
                    "kind": "facility_node",
                    "node_type": node.node_type,
                    "theta": node.pose.theta,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [node.pose.x, node.pose.y],
                },
            }
        )
    for edge in facility.edges.values():
        start = facility.nodes[edge.start_node].pose
        end = facility.nodes[edge.end_node].pose
        features.append(
            {
                "type": "Feature",
                "id": edge.id,
                "properties": {
                    "kind": "facility_edge",
                    "start_node": edge.start_node,
                    "end_node": edge.end_node,
                    "bidirectional": edge.bidirectional,
                    "max_velocity": edge.max_velocity,
                    "edge_type": edge.edge_type,
                },
                "geometry": {
                    "type": "LineString",
                    "coordinates": [
                        [start.x, start.y],
                        [end.x, end.y],
                    ],
                },
            }
        )
    for zone in facility.zones.values():
        ring = [[x, y] for x, y in zone.polygon]
        ring.append(list(ring[0]))
        features.append(
            {
                "type": "Feature",
                "id": zone.id,
                "properties": {
                    "kind": "facility_zone",
                    "zone_type": zone.zone_type,
                    "speed_limit": zone.speed_limit,
                    "active": zone.active,
                },
                "geometry": {"type": "Polygon", "coordinates": [ring]},
            }
        )
    for asset in facility.assets.values():
        features.append(
            {
                "type": "Feature",
                "id": asset.id,
                "properties": {
                    "kind": "facility_asset",
                    "asset_type": asset.asset_type,
                    "asset_version": asset.asset_version,
                    "parent_node": asset.parent_node,
                    "available": asset.available,
                    "theta": asset.pose.theta,
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [asset.pose.x, asset.pose.y],
                },
            }
        )
    return {
        "type": "FeatureCollection",
        "name": facility.name,
        "properties": {
            "schema_version": facility.schema_version,
            "facility_map_version": facility.facility_map_version,
            "frame": facility.frame,
            "default_speed": facility.default_speed,
        },
        "features": features,
    }


def from_nav2_geojson(document: dict[str, Any]) -> FacilityMap:
    """Rebuild a facility document from this module's GeoJSON export."""
    meta = document.get("properties", {})
    data: dict[str, Any] = {
        "schema_version": meta.get("schema_version", "1.0.0"),
        "name": document.get("name", "imported-facility"),
        "facility_map_version": int(meta.get("facility_map_version", 1)),
        "frame": meta.get("frame", "map"),
        "default_speed": float(meta.get("default_speed", 1.0)),
        "nodes": [],
        "edges": [],
        "zones": [],
        "assets": [],
    }
    for feature in document.get("features", []):
        props = feature.get("properties", {})
        kind = props.get("kind")
        geometry = feature.get("geometry", {})
        if kind == "facility_node":
            x, y = geometry["coordinates"][:2]
            data["nodes"].append(
                {
                    "id": feature["id"],
                    "node_type": props["node_type"],
                    "pose": {"x": x, "y": y, "theta": props.get("theta", 0.0)},
                }
            )
        elif kind == "facility_edge":
            data["edges"].append(
                {
                    "id": feature["id"],
                    "start_node": props["start_node"],
                    "end_node": props["end_node"],
                    "bidirectional": props.get("bidirectional", True),
                    "max_velocity": props.get("max_velocity"),
                    "edge_type": props.get("edge_type", "lane"),
                }
            )
        elif kind == "facility_zone":
            ring = geometry["coordinates"][0]
            if len(ring) > 1 and ring[0] == ring[-1]:
                ring = ring[:-1]
            data["zones"].append(
                {
                    "id": feature["id"],
                    "zone_type": props["zone_type"],
                    "polygon": [[float(px), float(py)] for px, py in ring],
                    "speed_limit": props.get("speed_limit"),
                    "active": props.get("active", True),
                }
            )
        elif kind == "facility_asset":
            x, y = geometry["coordinates"][:2]
            data["assets"].append(
                {
                    "id": feature["id"],
                    "asset_type": props["asset_type"],
                    "asset_version": props.get("asset_version", 1),
                    "pose": {"x": x, "y": y, "theta": props.get("theta", 0.0)},
                    "parent_node": props["parent_node"],
                    "available": props.get("available", True),
                }
            )
    return FacilityMap.from_dict(data)


def to_rmf_nav_graph(facility: FacilityMap) -> dict[str, Any]:
    """Open-RMF navigation-graph shaped export (one level, the facility frame)."""
    vertices = [
        {
            "name": node.id,
            "x": node.pose.x,
            "y": node.pose.y,
            "yaw": node.pose.theta,
            "type": node.node_type,
        }
        for node in facility.nodes.values()
    ]
    lanes = [
        {
            "edge_id": edge.id,
            "start": edge.start_node,
            "end": edge.end_node,
            "bidirectional": edge.bidirectional,
            "max_velocity": facility.effective_speed(edge),
            "type": edge.edge_type,
        }
        for edge in facility.edges.values()
    ]
    return {
        "name": facility.name,
        "facility_map_version": facility.facility_map_version,
        "levels": {facility.frame: {"vertices": vertices, "lanes": lanes}},
    }


def to_lif(facility: FacilityMap) -> dict[str, Any]:
    """Lane-indexed export: vertices and lanes referenced by integer index."""
    order = list(facility.nodes)
    index = {node_id: position for position, node_id in enumerate(order)}
    return {
        "name": facility.name,
        "facility_map_version": facility.facility_map_version,
        "frame": facility.frame,
        "vertices": [
            {
                "index": index[node.id],
                "name": node.id,
                "x": node.pose.x,
                "y": node.pose.y,
                "yaw": node.pose.theta,
                "node_type": node.node_type,
            }
            for node in facility.nodes.values()
        ],
        "lanes": [
            {
                "index": lane_index,
                "edge_id": edge.id,
                "start_vertex": index[edge.start_node],
                "end_vertex": index[edge.end_node],
                "bidirectional": edge.bidirectional,
                "max_velocity": facility.effective_speed(edge),
            }
            for lane_index, edge in enumerate(facility.edges.values())
        ],
    }


def to_vda5050_order(
    facility: FacilityMap,
    order_id: str,
    node_ids: Sequence[str],
    order_update_id: int = 0,
    released_count: int | None = None,
) -> dict[str, Any]:
    """Build a VDA 5050 v3 order document for a facility route.

    ``released_count`` implements the released-base + horizon model: nodes with
    sequenceId below it are released to the robot, the rest are horizon. The
    shape follows the published order model; conformance is not verified (G-E).
    """
    if len(node_ids) < 2:
        raise ValueError("a VDA 5050 order needs at least two nodes")
    for node_id in node_ids:
        if node_id not in facility.nodes:
            raise KeyError(f"unknown facility node {node_id!r}")
    released = len(node_ids) if released_count is None else int(released_count)
    if not 2 <= released <= len(node_ids):
        raise ValueError(
            f"released_count must release at least the first two nodes and at most all {len(node_ids)}"
        )

    nodes = []
    for sequence, node_id in enumerate(node_ids):
        pose = facility.nodes[node_id].pose
        nodes.append(
            {
                "nodeId": node_id,
                "sequenceId": sequence,
                "released": sequence < released,
                "nodePosition": {
                    "x": round(pose.x, 4),
                    "y": round(pose.y, 4),
                    "theta": round(pose.theta, 4),
                    "mapId": facility.frame,
                },
                "actions": [],
            }
        )

    edges = []
    for sequence, (start, end) in enumerate(zip(node_ids, node_ids[1:])):
        edge = facility.edge_between(start, end)
        edges.append(
            {
                "edgeId": edge.id,
                "sequenceId": sequence,
                "released": sequence + 1 < released,
                "startNodeId": start,
                "endNodeId": end,
                "maxSpeed": round(facility.effective_speed(edge), 4),
            }
        )

    return {
        "orderId": order_id,
        "orderUpdateId": int(order_update_id),
        "nodes": nodes,
        "edges": edges,
    }


def apply_horizon_update(
    order: dict[str, Any], released_count: int
) -> dict[str, Any]:
    """Release more of a horizon (VDA 5050 order update) and bump the update id."""
    updated = copy.deepcopy(order)
    total = len(updated.get("nodes", []))
    if not 2 <= released_count <= total:
        raise ValueError(f"released_count must be within [2, {total}]")
    currently_released = sum(1 for node in updated["nodes"] if node.get("released"))
    if released_count < currently_released:
        raise ValueError(
            "VDA 5050 released nodes cannot be withdrawn; only additional nodes may be released"
        )
    for node in updated["nodes"]:
        node["released"] = node["sequenceId"] < released_count
    for edge in updated.get("edges", []):
        edge["released"] = edge["sequenceId"] + 1 < released_count
    updated["orderUpdateId"] = int(updated.get("orderUpdateId", 0)) + 1
    return updated


def route_to_vda5050_order(
    facility: FacilityMap,
    order_id: str,
    route: Route,
    released_count: int | None = None,
) -> dict[str, Any]:
    return to_vda5050_order(
        facility,
        order_id,
        route.node_ids,
        released_count=released_count,
    )
