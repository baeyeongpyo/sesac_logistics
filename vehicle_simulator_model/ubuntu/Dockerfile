FROM osrf/ros:humble-desktop-full-jammy

ARG VIRTUALGL_VERSION=3.1.4
ARG VIRTUALGL_SHA256=02edc6b599571c385389af1a006f07a70c298e1d97c580a9bfd4b39d835c51e6
ENV DEBIAN_FRONTEND=noninteractive

COPY vendor/virtualgl_${VIRTUALGL_VERSION}_amd64.deb /tmp/virtualgl.deb

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        dbus-x11 \
        libxml2-utils \
        mesa-utils \
        python3-colcon-common-extensions \
        python3-rosdep \
        ros-humble-ros-gz \
        ros-humble-xacro \
        x11-apps \
        xauth \
    && echo "${VIRTUALGL_SHA256}  /tmp/virtualgl.deb" | sha256sum -c - \
    && apt-get install -y --no-install-recommends /tmp/virtualgl.deb \
    && rm -f /tmp/virtualgl.deb \
    && rm -rf /var/lib/apt/lists/*

COPY ros2_ws/src /opt/mentorpi_ws/src
RUN . /opt/ros/humble/setup.sh \
    && cd /opt/mentorpi_ws \
    && colcon build --symlink-install

RUN useradd --create-home --uid 1000 --user-group --shell /bin/bash ros \
    && chown -R ros:ros /opt/mentorpi_ws /home/ros

COPY entrypoint.sh /usr/local/bin/mentorpi-entrypoint
RUN chmod +x /usr/local/bin/mentorpi-entrypoint

ENV DEBIAN_FRONTEND= \
    HOME=/home/ros \
    PATH=/opt/VirtualGL/bin:${PATH} \
    QT_X11_NO_MITSHM=1

USER ros
WORKDIR /ws

ENTRYPOINT ["/usr/local/bin/mentorpi-entrypoint"]
CMD ["bash"]
