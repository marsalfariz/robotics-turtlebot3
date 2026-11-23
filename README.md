# ROS Line Follower

A camera-based differential-drive line follower with a shared controller, track
generator, and verifier. Both launchers check straight-line tracking, turns,
and a stop after the line disappears.

## ROS 1 Noetic

Use Ubuntu 20.04 with Gazebo Classic. From this directory, run:

```bash
bash noetic/setup.sh && bash noetic/run.sh verify auto
```

To leave the Gazebo window open for a demo, run `bash noetic/run.sh demo gui`.
Use `verify headless` for a verification run in a virtual display.

## ROS 2 Humble

Use Ubuntu 22.04 with Gazebo Fortress. From this directory, run:

```bash
bash ./setup.sh && LIBGL_ALWAYS_SOFTWARE=1 bash ./run.sh verify auto
```

For a demo, run `LIBGL_ALWAYS_SOFTWARE=1 bash ./run.sh demo gui`. The software
rendering option addresses an OGRE2 crash observed in WSLg. Use `verify headless`
for a verification run with no visible window.

## Source layout

- `noetic/` contains the ROS 1 node, launch configuration, and Gazebo Classic
  world adapter.
- `ros_ws/src/line_follower/` contains the ROS 2 package and the shared track,
  vision, and verification logic.
- The camera publishes `/camera/image`. The controller publishes `/cmd_vel` and
  the simulator supplies `/odom`.

Each verification run writes telemetry, a result JSON, a launch log, and the
generated world to a timestamped directory in `evidence/`.
