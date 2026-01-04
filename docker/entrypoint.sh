#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /root/project/ws/install/setup.bash ]; then
  source /root/project/ws/install/setup.bash
elif [ -f /root/ws/install/setup.bash ]; then
  source /root/ws/install/setup.bash
fi

if [ -d /mnt/wslg/runtime-dir ]; then
  install -d -m700 /run/user/0
  ln -sf /mnt/wslg/runtime-dir/wayland-0 /run/user/0/wayland-0
  export XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/0}
  export WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}
  export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-wayland}
  export PULSE_SERVER=${PULSE_SERVER:-/mnt/wslg/PulseServer}
fi

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
export GZ_VERSION=${GZ_VERSION:-harmonic}

exec "$@"
