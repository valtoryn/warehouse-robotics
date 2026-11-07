# Warehouse asset: dock, charging station, door/lift hook, pick/drop station.
# Each asset class is versioned independently (MODULARITY_SPEC section 9).
# Status: proposed interface — not implemented.

string id
string asset_type            # "dock", "charging_station", "door", "lift",
                             # "pick_station", "drop_station", ...
uint32 asset_version         # per-class version, independent of map version
geometry_msgs/Pose2D pose    # docking/interaction pose in the facility map frame
string parent_node           # FacilityNode.id this asset is attached to
bool available               # operational state mirrored from fleet/facility layer
