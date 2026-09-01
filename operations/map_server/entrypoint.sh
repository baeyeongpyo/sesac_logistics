#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
source /opt/map_server_ws/install/setup.bash

exec "$@"
