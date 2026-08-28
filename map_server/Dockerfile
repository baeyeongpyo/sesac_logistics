FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-colcon-common-extensions \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-nav2-map-server \
        ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 1000 --user-group --shell /bin/bash ros

WORKDIR /ws
COPY mentorpi_map_server src/mentorpi_map_server
RUN . /opt/ros/humble/setup.sh \
    && colcon build --packages-select mentorpi_map_server \
    && chown -R ros:ros /ws

COPY run_map_server.sh /usr/local/bin/mentorpi-map-server
RUN chmod +x /usr/local/bin/mentorpi-map-server

ENV DEBIAN_FRONTEND= \
    HOME=/home/ros

USER ros
CMD ["/usr/local/bin/mentorpi-map-server"]
