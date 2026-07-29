#!/usr/bin/env bash
set -euo pipefail

export DISPLAY=:99
pids=()

cleanup() {
  local pid
  for pid in "${pids[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 0' INT TERM

Xvfb :99 -screen 0 "${VIEWER_GEOMETRY:-1920x1080x24}" -nolisten tcp &
pids+=("$!")

for _ in $(seq 1 50); do
  xdpyinfo -display :99 >/dev/null 2>&1 && break
  sleep 0.1
done
xdpyinfo -display :99 >/dev/null 2>&1

gz sim --force-version 8 -g &
pids+=("$!")

x11vnc \
  -display :99 \
  -rfbport 5900 \
  -localhost \
  -nopw \
  -forever \
  -shared \
  -viewonly &
pids+=("$!")

websockify --web=/usr/share/novnc 6080 localhost:5900 &
pids+=("$!")

wait -n "${pids[@]}"
