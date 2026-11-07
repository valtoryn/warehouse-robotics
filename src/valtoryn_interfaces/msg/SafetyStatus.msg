# Read-only view of the certified safety domain for the ROS side.
# The safety domain is independent of ROS 2 (see ADR safety boundary in
# docs/architecture/ARCHITECTURE_DECISION_RECORDS.md). ROS nodes consume this
# state; they must never command the safety-rated stop path through it.
# Status: proposed interface — not implemented.

builtin_interfaces/Time stamp

bool safety_domain_alive      # heartbeat from the safety controller
bool emergency_stop_active    # hard stop asserted
bool protective_field_clear   # all protective fields clear (scanner inputs)
bool degraded_mode            # safety domain operating with reduced function
string safety_state_id        # vendor/state-machine state identifier
float32 battery_soc           # state-of-charge percent, if provided by safety domain
