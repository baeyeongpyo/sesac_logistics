FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        fastdds-tools \
        gettext-base \
        ros-humble-domain-bridge \
        ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN useradd --create-home --uid 1000 --user-group --shell /bin/bash ros

COPY --chown=ros:ros bridge.yaml.template /etc/dds-domain-bridge/bridge.yaml.template
COPY entrypoint.sh /usr/local/bin/dds-domain-bridge-entrypoint
COPY dds_env.sh /usr/local/bin/dds-domain-bridge-env
COPY dds-observe /usr/local/bin/dds-observe
RUN chmod +x \
    /usr/local/bin/dds-domain-bridge-entrypoint \
    /usr/local/bin/dds-domain-bridge-env \
    /usr/local/bin/dds-observe

ENV DEBIAN_FRONTEND= \
    HOME=/home/ros

USER ros
WORKDIR /work

ENTRYPOINT ["/usr/local/bin/dds-domain-bridge-entrypoint"]
CMD ["bash"]
