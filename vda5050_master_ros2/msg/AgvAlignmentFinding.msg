# AgvAlignmentFinding — one finding from
# vda5050_master::validation::check_factsheet_alignment.
#
# **V0 finding codes** (master-side; not on the VDA5050 wire):
#
#   "SpeedExceedsCapability" (ERROR)
#       The map declares an edge max_speed greater than the AGV's
#       factsheet physicalParameters.speedMax. The AGV cannot traverse
#       the edge at the requested speed.
#
#   "MapIdMismatch" (WARNING)
#       The factsheet (vendor-specific field) declares a mapId that
#       does not equal the master's loaded map_id. Vendors are
#       inconsistent about populating this so it is informational.
#
#   "MapVersionMismatch" (WARNING)
#       The factsheet declares a mapVersion that does not equal the
#       master's loaded map_version. Same caveat as MapIdMismatch.
#
# Forward-compat: future codes may be added — clients should treat
# unknown codes as opaque strings and rely on severity for routing.

uint8 SEVERITY_WARNING=0
uint8 SEVERITY_ERROR=1
uint8 severity

# Stable code identifier — see list above.
string code

# Human-readable description, including the offending values where
# relevant (e.g. "edge E1 max_speed=2.0 > factsheet speedMax=1.5").
string description
