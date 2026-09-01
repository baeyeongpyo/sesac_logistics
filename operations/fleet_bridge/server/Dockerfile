FROM ros:humble-ros-base-jammy AS builder

SHELL ["/bin/bash", "-c"]
ARG MENTORPI_COMMIT=fb6d9969e935eb0e31966185158e33347951e761

RUN apt-get update && apt-get install -y --no-install-recommends \
      git \
      python3-colcon-common-extensions \
      python3-pip \
      python3-yaml \
      ros-humble-action-msgs \
      ros-humble-bond \
      ros-humble-diagnostic-msgs \
      ros-humble-dwb-msgs \
      ros-humble-geometry-msgs \
      ros-humble-lifecycle-msgs \
      ros-humble-map-msgs \
      ros-humble-nav2-msgs \
      ros-humble-nav-msgs \
      ros-humble-rcl-interfaces \
      ros-humble-resource-retriever \
      ros-humble-rosgraph-msgs \
      ros-humble-rosbag2 \
      ros-humble-rosidl-default-generators \
      ros-humble-rosidl-default-runtime \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-tf2-msgs \
      ros-humble-tf2-ros \
      ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir --target /opt/python \
      fastapi==0.115.12 \
      httpx==0.28.1 \
      uvicorn==0.34.0 \
      websockets==10.4

WORKDIR /opt/fleet_bridge_ws
RUN git init /tmp/mentorpi \
    && git -C /tmp/mentorpi remote add origin https://github.com/Hiwonder/MentorPi.git \
    && git -C /tmp/mentorpi fetch --depth 1 origin "${MENTORPI_COMMIT}" \
    && git -C /tmp/mentorpi checkout FETCH_HEAD -- driver/ros_robot_controller_msgs \
    && mkdir -p src \
    && mv /tmp/mentorpi/driver/ros_robot_controller_msgs src/ros_robot_controller_msgs \
    && rm -rf /tmp/mentorpi
COPY common/fleet_bridge_config src/fleet_bridge_config
COPY server/ros2_ws/src/fleet_bridge_worker src/fleet_bridge_worker

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
      ros-humble-bond \
      ros-humble-diagnostic-msgs \
      ros-humble-dwb-msgs \
      ros-humble-geometry-msgs \
      ros-humble-lifecycle-msgs \
      ros-humble-map-msgs \
      ros-humble-nav2-msgs \
      ros-humble-nav-msgs \
      ros-humble-rcl-interfaces \
      ros-humble-resource-retriever \
      ros-humble-rmw-fastrtps-cpp \
      ros-humble-rosgraph-msgs \
      ros-humble-rosbag2 \
      ros-humble-rosidl-default-runtime \
      ros-humble-rosx-introspection \
      ros-humble-sensor-msgs \
      ros-humble-std-msgs \
      ros-humble-tf2-msgs \
      ros-humble-tf2-ros \
      ros-humble-visualization-msgs \
      zlib1g \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /opt/fleet_bridge_ws/install /opt/fleet_bridge_ws/install
COPY --from=builder /opt/python /opt/python
COPY --chmod=755 server/entrypoint.sh /entrypoint.sh

ENV PYTHONPATH=/opt/python

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "fleet_bridge_worker", "fleet_bridge_worker"]
