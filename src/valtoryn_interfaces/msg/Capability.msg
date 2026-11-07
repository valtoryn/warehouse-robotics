# Valtoryn capability declaration.
# A capability is something a module or robot can do. The declared/verified
# split mirrors module-manifest.schema.json: 'declared' means the module claims
# it, 'verified' means it was confirmed by a recorded verification method.
# Status: proposed interface — not implemented.

string name              # open/extensible enumeration, e.g. "differential_drive_kinematics"
string kind              # e.g. "locomotion", "perception", "payload", "safety_monitoring"
bool verified            # false = declared-only claim
string verification_ref  # pointer to verification record; empty if unverified
