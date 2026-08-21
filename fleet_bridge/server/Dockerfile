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
      python3-pip \
      python3-yaml \
      ros-humble-ament-index-cpp \
      ros-humble-action-msgs \
      ros-humble-diagnostic-msgs \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs \
      ros-humble-rclcpp-components \
      ros-humble-resource-retriever \
      ros-humble-rosgraph-msgs \
      ros-humble-rosbag2 \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-tf2-msgs \
      zlib1g-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --target /opt/python \
      fastapi==0.115.12 \
      httpx==0.28.1 \
      uvicorn==0.34.0 \
      websockets==10.4

WORKDIR /opt/fleet_bridge_ws
RUN git clone https://github.com/foxglove/ros-foxglove-bridge.git src/foxglove_bridge \
    && git -C src/foxglove_bridge checkout "${FOXGLOVE_BRIDGE_COMMIT}"
COPY common/fleet_bridge_config src/fleet_bridge_config
COPY server/ros2_ws/src/foxglove_ros_worker src/foxglove_ros_worker

RUN source /opt/ros/humble/setup.bash \
    && colcon build \
      --merge-install \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF


FROM ros:humble-ros-base-jammy AS runtime

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      libssl3 \
      python3-yaml \
      ros-humble-action-msgs \
      ros-humble-diagnostic-msgs \
      ros-humble-geometry-msgs \
      ros-humble-nav-msgs \
      ros-humble-resource-retriever \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-rosgraph-msgs \
      ros-humble-rosbag2 \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-tf2-msgs \
      zlib1g \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/fleet_bridge_ws/install /opt/fleet_bridge_ws/install
COPY --from=builder /opt/python /opt/python
COPY --chmod=755 server/entrypoint.sh /entrypoint.sh

ENV PYTHONPATH=/opt/python

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "foxglove_ros_worker", "foxglove_ros_worker"]
