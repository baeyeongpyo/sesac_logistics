FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-colcon-common-extensions \
      python3-yaml \
      ros-humble-geometry-msgs \
      ros-humble-rclpy \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/warehouse_server_ws
COPY ros2_ws/src/warehouse_overlay_server src/warehouse_overlay_server

RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

COPY --chmod=755 entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "warehouse_overlay_server", "warehouse_zone_publisher"]
