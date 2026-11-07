# Valtoryn facility-model types.
# Internal, versioned facility model owned by fleet intelligence (L6) per
# docs/architecture/MODULARITY_SPECIFICATION.md section 9. Superset of VDA 5050
# node/edge, LIF, RMF graph and Nav2 GeoJSON semantics. The internal model is
# never serialized in an external format internally.
# Status: proposed interfaces — not implemented.

string id                    # unique within the facility map version
string node_type             # open/extensible: "station", "junction", "charging", ...
geometry_msgs/Pose2D pose    # x, y, theta in the facility map frame
string parent_map_frame      # facility-level frame name (e.g. "map")
