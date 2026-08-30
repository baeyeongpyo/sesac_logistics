#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

"$script_dir/command_ros_stop.sh"
"$script_dir/foxglove_stop.sh"
