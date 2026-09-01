#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/fleet_bridge_ws/install/setup.bash

exec "$@"
