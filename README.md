A deployable ROS 2 project that demonstrates:
- fleet health monitoring (heartbeat -> faults)
mission management (Action-based) [planned]
-integration gateway (REST/gRPC) [planned]
-CI, testing, and production-style SDLC docs

## platform
Windows + WSL2 (Ubuntu 24.04) + ROS 2 Jazzy

## Quick Start
```bash
cd ros_ws
colcon build
source install/setup.bash
