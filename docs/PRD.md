# FleeOps Gateway - PRD(Production Requirements Document) (v0)

## Problem
Robotics deployments fail in the integration + operations layer: health moinitoring, mission dispatch, and reliablity.

## Goal (v0 -> v1)
v0: Heartbeat + health supervision (fault on heartbeat loss)
v1: Mission execution (ROS 2 Action) + external API gateway

## Users
- Operator: wants robot health + faults
- Solution Engineer: wants stable API contracts and runbooks
- Developer: wants reproducbile builds, tests, CI

## Success Metrics
- Detect heartbeat loss within 2 seconds
- Publish fault within 100ms after detection
- One-command run for demo
- CI passes on every PR
