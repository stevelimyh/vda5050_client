# MasterConnection — master's own operational state, latched so DM and
# other ROS 2 consumers can detect master readiness, health degradation,
# and crashes without polling.
#
# Topic:     /<topic_namespace>/master_connection
# QoS:       RELIABLE + TRANSIENT_LOCAL (latched)
# Direction: master -> DM and other consumers
#
# Publish triggers:
#   - On startup → STARTING, then READY once the MQTT broker connects
#   - On MQTT broker disconnect → DEGRADED
#   - On MQTT broker reconnect → READY
#   - On master shutdown → SHUTTING_DOWN (best-effort)
#   - Periodic heartbeat (30 s) republishes the current state
#
# DM-side liveness detection: watch for absence of heartbeat. No new
# message within ~2x kHeartbeatInterval -> assume master is dead and
# hold off on publishing FleetRoster until it comes back.

# Wall-clock time of this state snapshot.
builtin_interfaces/Time stamp

# Master operational state.
uint8 STARTING=0       # Process up, subsystems initializing
uint8 READY=1          # Broker connected, ready to accept rosters
uint8 DEGRADED=2       # Broker disconnected; queued ops will buffer
uint8 SHUTTING_DOWN=3  # Graceful shutdown in progress
uint8 state

# Configurable per deployment for multi-master disambiguation. Falls
# back to a hostname-derived default if not set in the master ctor.
string master_id

# Software version string, e.g. "vda5050_master_ros2-2.0.0".
string master_version

# MQTT broker reachability. Mirrors what GetMasterBrokerStatus reports.
bool broker_connected

# Current fleet size (number of onboarded AGVs).
uint32 onboarded_agv_count
