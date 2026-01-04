#!/usr/bin/env bash
set -e

docker compose build

docker compose run --rm tb3_nav2_slam bash -lc "cd /root/project/ws && if [ -d src/turtlebot3_simulations ]; then vcs pull src; else vcs import src < .repos; fi && rosdep update && rosdep install --from-paths src -i -y --rosdistro humble && colcon build --symlink-install"
