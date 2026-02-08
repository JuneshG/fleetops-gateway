# (Architecture Decision Record)ADR0001 - Platform choice

## Decision
Use Windows + WSL2 (Ubuntu 24.04)  + ROS 2 Jazzy.

## Rationale
- Best ROS ecosystem compatibilty on Linux
- Keeps Windows productivity (VS Code UI)
- Reproducible environment for others

## Consequences
- Must distinguish Windows commands (PS>) vs Linux commands ($)
- Keep builds in Linux filesystem (~/...) for speed
