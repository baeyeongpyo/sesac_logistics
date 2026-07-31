#!/usr/bin/env bash
set -euo pipefail

# Run inside the Raspberry Pi ROS 2 Jazzy container.
# This recreates the manually installed vehicle runtime pieces used for the
# AprilTag loading test. It intentionally does not commit container state.

ROS_SETUP=${ROS_SETUP:-/opt/ros/jazzy/setup.bash}
WS=${WS:-/home/ubuntu/ros2_ws}
ASCAMERA_WS=${ASCAMERA_WS:-/home/ubuntu/ascam_ros2_ws}
MENTORPI_REPO=${MENTORPI_REPO:-https://github.com/Hiwonder/MentorPi.git}
ORADAR_REPO=${ORADAR_REPO:-https://github.com/lehoangan2906/Oradar-ms200-setup-file.git}

source "$ROS_SETUP"

apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-gpiozero \
  python3-lgpio \
  python3-serial \
  ros-jazzy-cv-bridge \
  ros-jazzy-laser-filters \
  ros-jazzy-rqt \
  ros-jazzy-rqt-image-view

mkdir -p "$WS/src"
tmpdir=$(mktemp -d)
trap 'rm -rf "$tmpdir"' EXIT

git clone --depth 1 "$MENTORPI_REPO" "$tmpdir/MentorPi"
rm -rf \
  "$WS/src/controller" \
  "$WS/src/ros_robot_controller" \
  "$WS/src/ros_robot_controller_msgs" \
  "$WS/src/sdk" \
  "$WS/src/peripherals"
cp -a "$tmpdir/MentorPi/driver/controller" "$WS/src/controller"
cp -a "$tmpdir/MentorPi/driver/ros_robot_controller" "$WS/src/ros_robot_controller"
cp -a "$tmpdir/MentorPi/driver/ros_robot_controller_msgs" "$WS/src/ros_robot_controller_msgs"
cp -a "$tmpdir/MentorPi/driver/sdk" "$WS/src/sdk"
cp -a "$tmpdir/MentorPi/peripherals" "$WS/src/peripherals"

git clone --depth 1 "$ORADAR_REPO" "$tmpdir/oradar_lidar"
rm -rf "$WS/src/oradar_lidar"
cp -a "$tmpdir/oradar_lidar" "$WS/src/oradar_lidar"
cp "$WS/src/oradar_lidar/package_ros2.xml" "$WS/src/oradar_lidar/package.xml"
sed -i 's/set(COMPILE_METHOD CATKIN)/set(COMPILE_METHOD COLCON)/' \
  "$WS/src/oradar_lidar/CMakeLists.txt"

if [ -d "$ASCAMERA_WS/install" ]; then
  source "$ASCAMERA_WS/install/setup.bash"
else
  echo "WARN: $ASCAMERA_WS/install not found; ascamera must be mounted or built separately." >&2
fi

cd "$WS"
colcon build --symlink-install --packages-select \
  controller \
  ros_robot_controller \
  ros_robot_controller_msgs \
  sdk \
  peripherals \
  oradar_lidar

echo "Vehicle runtime dependencies installed in $WS."
echo "Build the local fork_control and mentorpi_apriltag_control packages from this repository separately."
