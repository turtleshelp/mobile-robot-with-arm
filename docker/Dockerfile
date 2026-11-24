# ROS 2 Humble + Gazebo Classic 11 on Ubuntu 22.04, for running this
# workspace on hosts with a newer Ubuntu (e.g. 24.04).
FROM osrf/ros:humble-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# Same packages as the README quick start, plus build tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-moveit ros-humble-navigation2 ros-humble-nav2-bringup \
      ros-humble-slam-toolbox ros-humble-rmw-cyclonedds-cpp \
      ros-humble-teleop-twist-keyboard \
      python3-colcon-common-extensions python3-rosdep \
      ffmpeg mesa-utils sudo \
    && rm -rf /var/lib/apt/lists/*

# Everything else the packages declare (dynamixel_sdk, cartographer_ros, ...).
COPY src /tmp/src
RUN apt-get update \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths /tmp/src --ignore-src -r -y --rosdistro humble \
    && rm -rf /tmp/src /var/lib/apt/lists/*

# A user matching the host user, so files written to the mounted
# workspace (build/, install/, log/) belong to you, not root.
RUN groupadd -g ${USER_GID} ${USERNAME} \
    && useradd -m -u ${USER_UID} -g ${USER_GID} -G video,sudo -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /home/${USERNAME}/.bashrc \
    && echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# GPU selection (NVIDIA or CPU rendering) is done by docker/run.sh.
# Gazebo's own environment (as /usr/share/gazebo/setup.sh sets it), but
# without fetching models from models.gazebosim.org at startup: that
# server hangs gzserver long enough for spawn_entity to time out. The
# models the world needs (ground_plane, sun) ship with Gazebo.
# gazebo_ros only sources setup.sh when GAZEBO_MODEL_DATABASE_URI is
# unset, so once it is set the paths below must be set here too.
ENV GAZEBO_MASTER_URI=http://localhost:11345 \
    GAZEBO_MODEL_DATABASE_URI= \
    GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11 \
    GAZEBO_PLUGIN_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins \
    GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models \
    OGRE_RESOURCE_PATH=/usr/lib/x86_64-linux-gnu/OGRE-1.9.0
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/gazebo-11/plugins
WORKDIR /ws
CMD ["bash"]
