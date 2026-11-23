# Robotika Theory

## ROS Line Follower

| Implementation | ROS environment | Sensor |
| --- | --- | --- |
| [Camera line follower](camera-noetic-humble/) | ROS 1 Noetic and ROS 2 Humble | Downward-facing camera |
| [IR line follower](ir-humble/line_follower/) | ROS 2 Humble | Simulated infrared sensors |

The implementations each use the ROS package name line_follower. Build them in
separate workspaces so the launch files resolve the intended package.

Use the camera project for the Noetic–Humble verification comparison. Use the IR
project for the line-following and lap experiments. Each implementation has its
own run instructions.
