FROM ros:humble-ros-base-jammy AS runtime

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        gz-harmonic \
        fastdds-tools \
        libxml2-utils \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-foxglove-bridge \
        ros-humble-foxglove-msgs \
        ros-humble-navigation2 \
        ros-humble-nav2-bringup \
        ros-humble-robot-state-publisher \
        ros-humble-ros-gzharmonic \
        ros-humble-rosbag2 \
        ros-humble-slam-toolbox \
        ros-humble-tf2-ros \
        ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src /opt/mentorpi_ws/src
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/mentorpi_ws \
    && colcon build --symlink-install

RUN useradd --create-home --uid 1000 --user-group --shell /bin/bash ros \
    && chown -R ros:ros /opt/mentorpi_ws /home/ros

COPY entrypoint.sh /usr/local/bin/mentorpi-entrypoint
COPY dds_env.sh /usr/local/bin/mentorpi-dds-env
COPY healthcheck.sh /usr/local/bin/mentorpi-healthcheck
RUN chmod +x \
    /usr/local/bin/mentorpi-entrypoint \
    /usr/local/bin/mentorpi-dds-env \
    /usr/local/bin/mentorpi-healthcheck

ENV DEBIAN_FRONTEND= \
    HOME=/home/ros

USER ros
WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/mentorpi-entrypoint"]
CMD ["bash"]
