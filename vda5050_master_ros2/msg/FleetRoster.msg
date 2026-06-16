# FleetRoster — the complete, authoritative set of AGVs that the
# master should currently be tracking. Published by an external fleet
# manager (e.g. RMF Device Manager) on a latched topic; the master
# subscribes and reconciles its onboarded list against this payload.
#
# **Semantics — authoritative full state, not delta**:
#   Every message carries the entire desired roster. On each message
#   the master computes the diff against its currently-onboarded set:
#     - entries present here, absent locally   → onboard_agv(...)
#     - entries absent here, present locally   → offboard_agv(...)
#     - entries in both                        → leave untouched (no-op)
#   Publishers MUST send the complete roster every time. Delta-style
#   updates (append-only / remove-only messages) are NOT supported.
#
# **Crash recovery**:
#   Use RELIABLE + TRANSIENT_LOCAL QoS on both publisher and subscriber
#   so a freshly-restarted master receives the latest roster as soon
#   as it subscribes — without waiting for the next roster change.
#
# **Idempotency**:
#   onboard_agv / offboard_agv are idempotent on the master side, so
#   re-publishing the same roster has no observable effect. Publishers
#   may republish on heartbeat without harm.

# Wall-clock time when the publisher built this roster snapshot.
# Used by the master for log correlation and staleness diagnostics.
builtin_interfaces/Time stamp

# Monotonic revision number of this roster — increments on every
# roster change. Named `roster_id` (not `sequence_id`) to avoid
# confusion with VDA5050 Node/Edge `sequence_id`, which addresses
# nodes inside an Order's graph. The master may log gaps (e.g.
# roster_id jumps from 5 to 8) to detect missed updates, but is not
# required to act on them: the latched message already carries the
# latest authoritative state. Master echoes this value back in
# FleetRosterState.roster_id_processed after applying the roster.
uint64 roster_id

# Human-readable identifier of the publisher (e.g. "rmf_device_manager").
# Diagnostic only — the master does not use this field for routing.
string publisher_id

# The complete current roster. Empty array means "no AGVs should be
# onboarded" — the master will offboard everything it currently tracks.
AGVOnboardSpec[] agvs
