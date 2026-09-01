FROM ros:humble-ros-base-jammy AS builder

SHELL ["/bin/bash", "-c"]
ARG FOXGLOVE_BRIDGE_COMMIT=41f96cc6053632a472d9a821989952771b1117f2

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      git \
      libasio-dev \
      libssl-dev \
      libwebsocketpp-dev \
      nlohmann-json3-dev \
      python3-colcon-common-extensions \
      ros-humble-ament-index-cpp \
      ros-humble-rclcpp-components \
      ros-humble-resource-retriever \
      ros-humble-rosbag2 \
      ros-humble-rosgraph-msgs \
      ros-humble-rosidl-default-runtime \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/monitoring_ws
RUN git clone https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove_bridge \
    && git -C src/foxglove_bridge checkout "${FOXGLOVE_BRIDGE_COMMIT}"

RUN source /opt/ros/humble/setup.bash \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

FROM ros:humble-ros-base-jammy AS runtime

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      libssl3 \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-resource-retriever \
      ros-humble-rosbag2 \
      ros-humble-rosidl-default-runtime \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      zlib1g \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/monitoring_ws/install /opt/monitoring_ws/install
COPY --chmod=755 entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "foxglove_bridge", "foxglove_bridge"]
