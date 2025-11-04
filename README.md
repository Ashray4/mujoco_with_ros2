# Mujoco_with_ros2

## Pre-alpha: 
Active development; interfaces and file paths are changing. Expect rough edges.

A lightweight bridge for real-time simulation and interaction between ROS 2 and MuJoCo. The goal is to bring MuJoCo models into a ROS 2 workflow, drive them with ros2_control controllers, and (soon) interact with the scene live and plan motions via MoveIt.

## Key Features

✅ ros2_control integration — Load controllers and command joints over standard ROS 2 interfaces.

🚧 Interactive UI to add/remove objects at runtime — Planned: spawn/delete bodies during an active MuJoCo simulation.

🚧 MoveIt integration for planning — Planned: use MoveIt to plan trajectories and execute them through ros2_control.

## Project Status

This is a pre-alpha prototype. Basic simulation + ros2_control are in place. UI tools for runtime scene editing and MoveIt wiring are in development. Breaking changes are likely.
