#!/usr/bin/env bash
set -e

GUI=${GUI:-true}
HEADLESS=${HEADLESS:-false}
RVIZ=${RVIZ:-true}
RVIZ_SOFTWARE=${RVIZ_SOFTWARE:-true}
SLAM=${SLAM:-true}
EXPLORE=${EXPLORE:-false}
MODEL=${MODEL:-waffle}
WORLD=${WORLD:-/root/project/assets/worlds/tb3_world.sdf}

if [ -d /mnt/wslg ]; then
  export WSLG_MOUNT=/mnt/wslg
  export X11_SOCKET_DIR=/mnt/wslg/.X11-unix
fi

if [ -d /usr/lib/wsl ]; then
  export WSL_LIB_MOUNT=/usr/lib/wsl
else
  export WSL_LIB_MOUNT=${WSL_LIB_MOUNT:-/tmp/wsl-lib}
  mkdir -p "${WSL_LIB_MOUNT}"
fi

if [ -e /dev/dxg ]; then
  export DXG_DEVICE=/dev/dxg
else
  export DXG_DEVICE=/dev/null
fi

docker compose up -d

docker compose exec -d tb3_nav2_slam bash -lc "source /opt/ros/humble/setup.bash && source /root/project/ws/install/setup.bash && ros2 launch tb3_nav2_slam_bringup bringup_slam_nav2.launch.py gui:=${GUI} headless:=${HEADLESS} rviz:=${RVIZ} rviz_software:=${RVIZ_SOFTWARE} slam:=${SLAM} explore:=${EXPLORE} model:=${MODEL} world:=${WORLD}"
