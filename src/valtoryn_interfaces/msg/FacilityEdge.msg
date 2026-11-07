# Directed traversal edge between two facility nodes.
# Status: proposed interface — not implemented.

string id                    # unique within the facility map version
string start_node            # FacilityNode.id
string end_node              # FacilityNode.id
double max_velocity          # m/s; 0 = inherit facility default
bool bidirectional           # false = directed edge (one-way lane)
string edge_type             # open/extensible: "lane", "doorway", "lift", ...
