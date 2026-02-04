# Architecture (v0)

## Components
- fleetops_msgs: message contracts ( Heartbeat, Fault, RobotHealth)
- fleetops_heartbeat: publishes heartbeats (simulated robot)
- fleetops_health: monitors heartbeat, publishes faults + health

## Data Flow
heartbeat_publisher -> /fleet/heartbeat -> health_supervisor -> /fleet/faults + /fleet/health
