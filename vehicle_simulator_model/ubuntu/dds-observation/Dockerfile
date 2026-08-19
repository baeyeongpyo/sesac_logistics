FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-foxglove-bridge \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-map-server \
        ros-humble-rosbag2 \
        ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 1000 --user-group --shell /bin/bash ros

COPY rosbag_recorder.sh /usr/local/bin/mentorpi-rosbag-recorder
COPY rosbag_recorder_bootstrap.sh /usr/local/bin/mentorpi-rosbag-recorder-bootstrap
COPY map_server.sh /usr/local/bin/mentorpi-map-server
COPY map_server_bootstrap.sh /usr/local/bin/mentorpi-map-server-bootstrap
RUN chmod +x /usr/local/bin/mentorpi-rosbag-recorder \
    /usr/local/bin/mentorpi-rosbag-recorder-bootstrap \
    /usr/local/bin/mentorpi-map-server \
    /usr/local/bin/mentorpi-map-server-bootstrap

ENV DEBIAN_FRONTEND= \
    HOME=/home/ros

USER ros
WORKDIR /work
CMD ["bash"]
