#!/usr/bin/env bash
set -euo pipefail

pid_file="${TMPDIR:-/tmp}/foxglove-bridge.pid"
log_file="${TMPDIR:-/tmp}/foxglove-bridge.log"

read_pid() {
  [[ -f "$pid_file" ]] || return 1

  local pid
  pid=$(<"$pid_file")
  [[ "$pid" =~ ^[0-9]+$ ]] || return 1
  printf '%s\n' "$pid"
}

is_running() {
  local pid
  pid=$(read_pid) || return 1
  kill -0 "$pid" 2>/dev/null
}

start() {
  if is_running; then
    printf 'already running: PID=%s\n' "$(read_pid)"
    return
  fi

  rm -f "$pid_file"

  nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml \
    >"$log_file" 2>&1 < /dev/null &

  local pid=$!
  printf '%s\n' "$pid" > "$pid_file"
  printf 'started: PID=%s\n' "$pid"
}

stop() {
  local pid
  pid=$(read_pid) || {
    printf 'not running\n'
    return
  }

  if ! kill -0 "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    printf 'removed stale PID file: PID=%s\n' "$pid"
    return
  fi

  if ! kill -TERM "$pid" 2>/dev/null; then
    rm -f "$pid_file"
    printf 'removed stale PID file: PID=%s\n' "$pid"
    return
  fi

  for _ in {1..20}; do
    kill -0 "$pid" 2>/dev/null || break
    sleep 0.1
  done

  if kill -0 "$pid" 2>/dev/null; then
    printf 'process did not exit: PID=%s\n' "$pid" >&2
    return 1
  fi

  rm -f "$pid_file"
  printf 'stopped: PID=%s\n' "$pid"
}

status() {
  if is_running; then
    printf 'running: PID=%s\n' "$(read_pid)"
    return
  fi
  printf 'not running\n'
  return 1
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  *)
    printf 'usage: %s {start|stop|status}\n' "$0" >&2
    exit 2
    ;;
esac
