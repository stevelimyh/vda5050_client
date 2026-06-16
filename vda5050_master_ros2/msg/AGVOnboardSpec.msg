# AGVOnboardSpec — one AGV's onboard specification.
#
# Generic per-AGV spec, used in three places:
#   - inside FleetRoster.msg          (DM → master fleet snapshot)
#   - inside OnboardAGV.srv requests  (single-AGV onboard)
#   - inside OnboardAGVBatch.srv requests (batch onboard)
#
# Same field set in all three contexts: {manufacturer, serial_number}
# are required identity; max_queue_size / drop_oldest mirror the C++
# defaults on VDA5050Master::onboard_agv().
#
# Leave the optional fields at their sentinel / default values to
# accept the master-side defaults.

# Required — AGV identity.
string manufacturer
string serial_number

# Optional — per-AGV outbound message queue capacity.
# 0 means "use master default" (currently 10).
uint32 max_queue_size 0

# Optional — queue-full policy for the AGV's outbound publisher.
# true  (default) → drop oldest queued message on overflow.
# false           → reject the new message instead.
bool drop_oldest true
