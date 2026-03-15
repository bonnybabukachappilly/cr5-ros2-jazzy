![ROS2 CI](https://github.com/bonnybabukachappilly/cr5-ros2-jazzy/actions/workflows/ros2_ci.yml/badge.svg)

# CR5 ROS2 Jazzy — From-Scratch Driver & Digital Twin

A complete, from-scratch ROS2 integration for the **Dobot CR5 collaborative 
robot arm** — built to deepen hands-on knowledge across the full ROS2 stack, 
from low-level hardware communication to motion planning, simulation, and 
real-time digital twin synchronisation.

## Why This Project

This project brings together experience from industrial automation, cobot 
programming, and simulation into a single, end-to-end ROS2 system — built 
entirely from scratch rather than cloning vendor packages.

Every layer is written manually: the TCP/IP driver, custom message 
definitions, MoveIt2 hardware interface, Gazebo simulation, and Digital Twin 
bridge. This approach reflects how core robotics engineering teams actually 
work — understanding the full stack, not just the application layer on top.

The project also serves as a structured learning environment for:
- Deepening ROS2 architecture knowledge through real hardware
- Advancing C++ skills within the ROS2 and MoveIt2 ecosystem
- Applying professional software practices — CI/CD, Git workflow, 
  test-driven development — to a robotics project from day one

## Project Goal

Build a **Gazebo Harmonic Digital Twin** of the Dobot CR5 — where the real 
arm moves and the simulation mirrors it in real time at 125Hz — with MoveIt2 
motion planning running on both sim and real hardware via the same launch file.

## System Architecture
```
TCP/IP Driver (cr5_driver)
    └── ROS2 Interface (cr5_bringup)
          └── MoveIt2 Motion Planning (cr5_moveit_config)
                └── Gazebo Harmonic Simulation (cr5_gazebo)
                      └── Digital Twin Bridge (cr5_twin_bridge)
                            └── RealSense Perception (cr5_perception)
```

## Packages

| Package | Purpose |
|---|---|
| `cr5_msgs` | Custom ROS2 message, service and action definitions |
| `cr5_driver` | TCP/IP driver communicating directly with CR5 ports 29999 and 30004 |
| `cr5_bringup` | Launch files, URDF integration, system startup |
| `cr5_moveit_config` | MoveIt2 configuration — planning groups, kinematics, controllers |
| `cr5_gazebo` | Gazebo Harmonic simulation environment and ros2_control setup |
| `cr5_twin_bridge` | Digital Twin sync node — mirrors real arm state into Gazebo at 125Hz |
| `cr5_perception` | Intel RealSense depth camera integration and object detection |

## Tech Stack

- **ROS2 Jazzy** on Ubuntu 24.04
- **MoveIt2** — motion planning and trajectory execution
- **Gazebo Harmonic** — simulation and Digital Twin
- **ros2_control** — hardware abstraction layer
- **Python** — all core nodes
- **C++** — progressive rewrites of key components as C++ skills advance
- **Intel RealSense** — depth perception (Phase 5)

## Phase Progress

- [x] Phase 0 — Environment setup + GitHub/CI infrastructure
- [ ] Phase 1 — TCP/IP driver from scratch
- [ ] Phase 2 — ROS2 core concepts on real hardware
- [ ] Phase 3 — MoveIt2 motion planning
- [ ] Phase 4 — Gazebo Harmonic Digital Twin
- [ ] Phase 5 — Intel RealSense integration (bonus)

## Software Practices

This project follows professional engineering practices from day one:

- **Git workflow** — feature branches, PRs, conventional commits, phase tags
- **CI/CD** — GitHub Actions running lint, build, unit tests and dependency 
  checks on every PR
- **GitHub Projects** — full Kanban board with issues, milestones and 
  acceptance criteria tracked per phase
- **Test-driven** — unit tests for TCP parser logic, integration tests for 
  node startup and topic existence

## Hardware

- Dobot CR5 — 6-axis collaborative robot arm (5kg payload, ±0.02mm repeatability)
- Intel RealSense Depth Camera
- Custom gripper

## Quick Start
```bash
# Clone the repository
git clone git@github.com:bonnybabukachappilly/cr5-ros2-jazzy.git ~/cr5_ws
cd ~/cr5_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
. /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Launch
ros2 launch cr5_bringup cr5_full.launch.py robot_ip:=192.168.5.1
```

## Hard Problems Solved

Honest engineering notes on real problems encountered and how they were 
fixed. Updated as the project grows.

---

*Built on Ubuntu 24.04 | ROS2 Jazzy | Gazebo Harmonic*
