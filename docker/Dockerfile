FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ARG ENABLE_WSLG_GPU=false

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    wget \
    git \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg | gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

RUN if [ "$ENABLE_WSLG_GPU" = "true" ]; then \
    add-apt-repository -y ppa:kisak/turtle && \
    apt-get update && apt-get install -y --no-install-recommends \
    libglvnd0 libgl1 libglx0 libxext6 libx11-6 libxkbcommon-x11-0 \
    libegl1 libglu1-mesa mesa-utils mesa-utils-extra \
    libxrandr2 libxss1 libxcursor1 libxcomposite1 libasound2 libxi6 libxtst6 \
    x11-apps x11-utils x11-xserver-utils xauth \
    libgl1-mesa-dri libegl-mesa0 libgbm1 libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*; \
  fi

RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-teleop-twist-keyboard \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-bringup \
    ros-humble-xacro \
    qtwayland5 \
    mesa-utils \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

WORKDIR /root/project

COPY docker/entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
