# Complete facility map snapshot. Each asset class carries its own version;
# facility_map_version is exchanged with fleet control (VDA 5050 horizon etc.).
# Status: proposed interface — not implemented.

std_msgs/Header header
string facility_map_version  # monotonic map revision
FacilityNode[] nodes
FacilityEdge[] edges
FacilityZone[] zones
FacilityAsset[] assets
