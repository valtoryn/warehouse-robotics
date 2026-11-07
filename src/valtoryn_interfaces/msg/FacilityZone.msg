# Geofenced zone with an access/speed policy.
# Absorbs Nav2 costmap-filter semantics and VDA 5050 zone types.
# Status: proposed interface — not implemented.

string id
string zone_type             # "keep_out", "speed_limit", "preferred_lane",
                             # "blocked", "released", "restricted"
geometry_msgs/Point32[] polygon   # closed polygon in the facility map frame
float32 speed_limit          # m/s; only meaningful for zone_type == "speed_limit"
bool active                  # blocked/released toggles without re-publishing geometry
