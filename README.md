# TurtleBot3 Nav2 + SLAM (ROS 2 Humble + Gazebo Harmonic)

A dockerized ROS 2 Humble workspace that runs Gazebo Harmonic, TurtleBot3 simulation, SLAM (slam_toolbox), Navigation2, and RViz2 together. It is designed for WSL2 + WSLg but includes native Ubuntu instructions.

## Prerequisites

WSL2 + Docker Desktop + WSLg:
- Windows 11 with WSL2 enabled
- Docker Desktop with WSL integration enabled
- WSLg (GUI apps work inside WSL)
- Run the scripts from a WSL Ubuntu terminal (not PowerShell)

Native Ubuntu:
- Docker Engine + docker compose plugin
- X11 or Wayland display available (for RViz/Gazebo GUI)

## Quick start (WSL2)

1) Build the image and workspace:
```
chmod +x scripts/*.sh
./scripts/build.sh
```

2) Bring up the full stack (Gazebo + SLAM + Nav2 + RViz):
```
./scripts/up.sh
```

Optional: enable autonomous exploration (frontier-based):
```
EXPLORE=true ./scripts/up.sh
```

3) Teleop the robot (new terminal):
```
./scripts/bash.sh
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

4) Save the map:
```
./scripts/bash.sh
ros2 run nav2_map_server map_saver_cli -f /root/project/maps/my_map
```

5) Shut everything down:
```
./scripts/down.sh
```

## WSLg GPU acceleration (optional)

If you have WSL2 GPU support (`/dev/dxg` present), you can validate OpenGL inside the container:
```
./scripts/bash.sh
glxinfo -B
```

If RViz or Gazebo fails to create a GL context, test software rendering once:
```
./scripts/bash.sh
LIBGL_ALWAYS_SOFTWARE=1 rviz2
```

If software rendering works but hardware doesn't, try setting:
```
export QT_QPA_PLATFORM='wayland;xcb'
```

For D3D12 acceleration under WSLg, you may also need to mount `/usr/lib/wsl` and set:
```
export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
export LIBGL_DRIVERS_PATH=/usr/lib/x86_64-linux-gnu/dri:/usr/lib/wsl/lib
export LD_LIBRARY_PATH=/usr/lib/wsl/lib
```

If you want to build with the optional Mesa upgrade, set:
```
ENABLE_WSLG_GPU=true ./scripts/build.sh
```

By default, RViz is launched in software rendering mode. To allow GPU rendering in RViz:
```
RVIZ_SOFTWARE=false ./scripts/up.sh
```

## Display environment variables

`docker-compose.yml` uses these variables, and `scripts/up.sh` auto-sets WSLg paths when `/mnt/wslg` exists:
- `WSLG_MOUNT` (default `/tmp/wslg`, WSLg uses `/mnt/wslg`)
- `X11_SOCKET_DIR` (default `/tmp/.X11-unix`, WSLg uses `/mnt/wslg/.X11-unix`)
- `QT_QPA_PLATFORM` (default `wayland`)

## Native Ubuntu (no WSL)

You can run the same dockerized workflow. Ensure X11 is accessible to the container:
```
xhost +local:root
export DISPLAY=:0
```

Then run:
```
./scripts/build.sh
./scripts/up.sh
```

If you are using pure X11, you may want:
```
export QT_QPA_PLATFORM=xcb
```

If your project is not mounted at `/root/project`, set `PROJECT_ROOT` before launching:
```
PROJECT_ROOT=$PWD ./scripts/up.sh
```

## How it works

The `bringup_slam_nav2.launch.py` file starts all major components:
- Gazebo Harmonic via `ros_gz_sim` + TurtleBot3 spawn
- ROS <-> Gazebo bridges (clock, cmd_vel, odom, scan)
- `slam_toolbox` for live mapping (`/map`, `map->odom`)
- Nav2 core servers (planner, controller, behavior tree)
- RViz2 visualization
- Optional frontier exploration via `explore_lite` (m-explore-ros2)

Text diagram:
```
Gazebo (gz) --(ros_gz_bridge)--> /scan /odom /clock
        |                                   |
        |                                   v
        |                            slam_toolbox
        |                           (/map, map->odom)
        |                                   |
        |                                   v
        |                                Nav2
        |                    (costmaps + planner/controller)
        |                                   |
        +<----------- /cmd_vel <------------+

Optional exploration:
explore_lite -> Nav2 NavigateToPose goals (frontier targets)

RViz2 consumes /map /scan /tf and provides goal tools.
```

## Validation checklist

Run these after `./scripts/up.sh`:

1) Sim time:
```
./scripts/bash.sh
ros2 param get /slam_toolbox use_sim_time
ros2 topic echo /clock --once
```

2) Core topics:
```
./scripts/bash.sh
ros2 topic list | egrep "cmd_vel|odom|scan|map|tf|clock"
```

3) TF tree:
```
./scripts/bash.sh
ros2 run tf2_tools view_frames
```
Confirm `map->odom`, `odom->base_link`, `base_link->base_scan`.

4) SLAM updates:
- Drive with `teleop_twist_keyboard`
- In RViz, add a Map display for `/map` and confirm updates

5) Save map:
```
./scripts/bash.sh
ros2 run nav2_map_server map_saver_cli -f /root/project/maps/my_map
```

6) Nav2 goal:
- Use RViz "Nav2 Goal" tool
- Robot should plan and drive while SLAM is running

7) Exploration (optional):
- Start with `EXPLORE=true ./scripts/up.sh`
- Watch the robot autonomously select frontier goals and expand the map

## Troubleshooting

- Gazebo GUI does not open in WSL2:
  - Verify WSLg is enabled and `DISPLAY`/`WAYLAND_DISPLAY` are set.
  - Try headless mode: `./scripts/up.sh` with `GUI=false` or `HEADLESS=true`.
  - Confirm the Wayland socket exists inside the container: `ls -l /run/user/0/wayland-0`.
  - If GUI never opens, ensure `gz_args` does not force `-s` (server-only). This repo removes it in `ws/src/tb3_nav2_slam_bringup/launch/sim_gz.launch.py`.

- No `/scan` topic:
  - Run `gz topic -l` inside the container to confirm the lidar topic exists.
  - If the topic name is not `/scan`, update `ws/src/tb3_nav2_slam_bringup/config/bridge.yaml`.
  - Ensure your world loads the Gazebo Sensors system plugin; see `assets/worlds/empty_world.sdf`.

- No `map->odom` transform:
  - Confirm slam_toolbox is running and frames match `slam_toolbox_params.yaml`.
  - Ensure `/tf` is being published; the `odom_to_tf` node can be disabled if it conflicts.

- Nav2 does not move the robot:
  - Echo `/cmd_vel` and ensure `ros_gz_bridge` maps to the correct gz topic.
  - Check that the diff drive system in the model listens to `/cmd_vel`.
  - Ensure `Nav2` is active and `explore_lite` is not running without a valid costmap.
  - For exploration, confirm `/global_costmap/costmap` and `/global_costmap/costmap_updates` exist.
  - Use `gz topic -l` to verify Gazebo topic names match `bridge.yaml`.

- TF mismatch:
  - If you see duplicate TF warnings, disable `odom_to_tf` in launch:
    `ros2 launch tb3_nav2_slam_bringup bringup_slam_nav2.launch.py odom_tf:=False`
  - If exploration starts too early, increase its delay: `ros2 launch tb3_nav2_slam_bringup explore.launch.py start_delay:=15.0`

- Package conflicts:
  - Do not install `ros-humble-ros-gz` (Fortress) alongside `ros-humble-ros-gzharmonic`.
  - If installed, remove it and rebuild the image.

## Useful commands

- Open a shell in the container:
```
./scripts/bash.sh
```

- Launch manually (inside container):
```
source /opt/ros/humble/setup.bash
source /root/project/ws/install/setup.bash
ros2 launch tb3_nav2_slam_bringup bringup_slam_nav2.launch.py
```

- Switch worlds:
```
WORLD=/root/project/assets/worlds/empty_world.sdf ./scripts/up.sh
```

## Notes on Gazebo Harmonic + Humble

Humble defaults to Gazebo Fortress. This repo uses Harmonic, so the image installs:
- `gz-harmonic`
- `ros-humble-ros-gzharmonic`

Avoid installing `ros-humble-ros-gz` in the same image.
