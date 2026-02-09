# FleetOps Gateway

ROS 2 health‑monitoring stack with custom messages, heartbeat publisher, and supervisor node that detects missed heartbeats and emits faults.

## Features
- Custom ROS 2 interfaces: Heartbeat, Fault, RobotHealth
- Heartbeat publisher (configurable rate + robot_id)
- Health supervisor with timeout detection
- Launch file for one‑command bring‑up
- Lint + unit tests
- Foxglove visualization

## Architecture
- `/fleet/heartbeat` → HeartbeatPublisher
- HealthSupervisor subscribes, tracks last‑seen, publishes:
  - `/fleet/health`
  - `/fleet/faults`

## Quickstart
```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch fleetops_health fleetops.launch.py
```

## Visualization (Foxglove)
```bash
ros2 run foxglove_bridge foxglove_bridge
```
Connect Foxglove to `ws://localhost:8765`.

## Tests
```bash
colcon test --event-handlers console_direct+
colcon test-result --all
```

## Topics
- `/fleet/heartbeat`
- `/fleet/health`
- `/fleet/faults`

## License
MIT
