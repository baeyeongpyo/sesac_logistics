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
      python3-yaml \
      ros-humble-ament-index-cpp \
      ros-humble-diagnostic-msgs \
      ros-humble-launch-ros \
      ros-humble-nav-msgs \
      ros-humble-rclcpp-components \
      ros-humble-resource-retriever \
      ros-humble-rosgraph-msgs \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-tf2-msgs \
      zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fleet_bridge_ws
RUN git clone https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove_bridge \
    && git -C src/foxglove_bridge checkout "${FOXGLOVE_BRIDGE_COMMIT}"
COPY common/fleet_bridge_config src/fleet_bridge_config
COPY vehicle/ros2_ws/src/fleet_telemetry_filter src/fleet_telemetry_filter

RUN source /opt/ros/humble/setup.bash \
    && colcon build \
      --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF


FROM ros:humble-ros-base-jammy AS runtime

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      libssl3 \
      python3-yaml \
      ros-humble-diagnostic-msgs \
      ros-humble-launch-ros \
      ros-humble-nav-msgs \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-rosgraph-msgs \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-tf2-msgs \
      zlib1g \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/fleet_bridge_ws/install /opt/fleet_bridge_ws/install
COPY --chmod=755 vehicle/entrypoint.sh /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "fleet_telemetry_filter", "vehicle_foxglove.launch.py"]
