FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ENV ROS_DISTRO=${ROS_DISTRO}

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    ca-certificates \
    gnupg \
    lsb-release \
    locales \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    mesa-utils \
  && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# OSRF repo for Gazebo Harmonic
RUN mkdir -p /etc/apt/keyrings \
 && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /etc/apt/keyrings/gazebo.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/gazebo.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Gazebo Harmonic + ROS gz harmonic + Nav2 + SLAM toolbox
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-${ROS_DISTRO}-ros-gzharmonic \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-twist-mux \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-turtlebot3-description \
  && rm -rf /var/lib/apt/lists/*

# rosdep init/update (must be root)
RUN rosdep init || true
RUN rosdep update --rosdistro=${ROS_DISTRO}

# Workspace
ENV TB3_WS=/opt/tb3_ws
RUN mkdir -p ${TB3_WS}/src
WORKDIR ${TB3_WS}/src

# Upstream sources (cloned into the image)
RUN git clone --branch new_gazebo https://github.com/azeey/turtlebot3_simulations.git \
 && git clone https://github.com/MaxStrange/m-explore-ros2.git \
 && cd m-explore-ros2 \
 && git checkout 9f0cd5dcb785b9fdd25c9fcf6d5b7a2a03773a3c

# Bringup package from this repo
COPY ws/src/tb3_nav2_slam_bringup ${TB3_WS}/src/tb3_nav2_slam_bringup

WORKDIR ${TB3_WS}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
 && rosdep install --from-paths src --ignore-src -r -y --rosdistro ${ROS_DISTRO} \
 && colcon build --symlink-install"

# Non-root dev user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME}

# Source ROS + workspace in interactive shells
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /etc/bash.bashrc \
 && echo "source ${TB3_WS}/install/setup.bash" >> /etc/bash.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}

CMD ["bash"]
