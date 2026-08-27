FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-yaml \
        ros-humble-domain-bridge \
        ros-humble-geometry-msgs \
        ros-humble-nav-msgs \
        ros-humble-sensor-msgs \
        ros-humble-tf2-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY render_bridge_config.py publish_bridge_status.py /opt/mentorpi-domain-bridge/
COPY run-domain-bridge /usr/local/bin/run-domain-bridge
RUN chmod +x /usr/local/bin/run-domain-bridge

CMD ["/usr/local/bin/run-domain-bridge"]
