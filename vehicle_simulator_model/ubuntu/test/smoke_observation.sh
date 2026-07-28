#!/usr/bin/env bash
set -euo pipefail

mode="${1:-}"
if [[ "$#" -ne 1 || "$mode" != lan && "$mode" != viewer ]]; then
  echo 'Usage: smoke_observation.sh <lan|viewer>' >&2
  exit 2
fi

bundle_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

smoke_lan() {
  : "${GZ_SERVER_IP:?set GZ_SERVER_IP}"
  : "${GZ_CLIENT_IP:?set GZ_CLIENT_IP}"

  check_client() {
    local name="$1"
    local topics
    topics="$(
      GZ_IP="$GZ_CLIENT_IP" \
      GZ_RELAY="$GZ_SERVER_IP" \
      GZ_PARTITION=mentorpi-sim \
      gz topic -l
    )"
    for topic in \
      /world/mentorpi_warehouse/stats \
      /robot_1/scan_raw \
      /robot_2/scan_raw; do
      grep -Fxq "$topic" <<<"$topics" || {
        printf '%s missing %s\n' "$name" "$topic" >&2
        return 1
      }
    done
  }

  check_client client-a &
  pid_a="$!"
  check_client client-b &
  pid_b="$!"
  wait "$pid_a"
  wait "$pid_b"

  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.lan.yaml" \
    exec -T gazebo-server /usr/local/bin/mentorpi-healthcheck server
  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.lan.yaml" \
    exec -T sim-adapter /usr/local/bin/mentorpi-healthcheck adapter
}

check_viewer_websockets() {
  VIEWER_WEBSOCKET_URL="$1" python3 - <<'PY'
import base64
import hashlib
import os
import socket
import threading
from urllib.parse import urlsplit

url = urlsplit(os.environ['VIEWER_WEBSOCKET_URL'])
host = url.hostname or '127.0.0.1'
port = url.port or 80
path = url.path or '/'
if url.query:
    path += f'?{url.query}'

barrier = threading.Barrier(2)
errors = []


def connect(client_number):
    sock = None
    try:
        key = base64.b64encode(
            hashlib.sha256(
                f'mentorpi-viewer-{client_number}'.encode()
            ).digest()[:16]
        ).decode()
        expected_accept = base64.b64encode(hashlib.sha1(
            (key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').encode()
        ).digest()).decode()
        request = (
            f'GET {path} HTTP/1.1\r\n'
            f'Host: {host}:{port}\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            f'Sec-WebSocket-Key: {key}\r\n'
            'Sec-WebSocket-Version: 13\r\n'
            'Sec-WebSocket-Protocol: binary\r\n'
            '\r\n'
        ).encode()
        sock = socket.create_connection((host, port), timeout=10)
        sock.sendall(request)
        response = b''
        while b'\r\n\r\n' not in response:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError('connection closed during handshake')
            response += chunk
            if len(response) > 16384:
                raise RuntimeError('oversized handshake response')
        header = response.split(b'\r\n\r\n', 1)[0].decode(
            'iso-8859-1'
        )
        lines = header.split('\r\n')
        headers = dict(
            line.split(':', 1)
            for line in lines[1:]
            if ':' in line
        )
        headers = {name.lower(): value.strip() for name, value in headers.items()}
        if ' 101 ' not in f' {lines[0]} ':
            raise RuntimeError(f'unexpected status: {lines[0]}')
        if headers.get('sec-websocket-accept') != expected_accept:
            raise RuntimeError('invalid Sec-WebSocket-Accept')
        barrier.wait(timeout=10)
        threading.Event().wait(1)
        print(f'viewer websocket client-{client_number}=connected')
    except Exception as error:
        errors.append(f'client-{client_number}: {error}')
        try:
            barrier.abort()
        except threading.BrokenBarrierError:
            pass
    finally:
        if sock is not None:
            sock.close()


threads = [
    threading.Thread(target=connect, args=(client_number,))
    for client_number in (1, 2)
]
for thread in threads:
    thread.start()
for thread in threads:
    thread.join()
if errors:
    raise SystemExit('; '.join(errors))
PY
}

smoke_viewer() {
  viewer_url="http://127.0.0.1:${VIEWER_PORT:-8080}/vnc.html?view_only=1&autoconnect=1"
  websocket_url="ws://127.0.0.1:${VIEWER_PORT:-8080}/websockify"
  curl -fsS "$viewer_url" | grep -Fq '<title>noVNC</title>'
  check_viewer_websockets "$websocket_url"

  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.viewer.yaml" \
    --profile viewer \
    exec -T gazebo-viewer \
    bash -lc '
      pgrep -f "gz sim --force-version 8 -g"
      pgrep -x Xvfb
      pgrep -af x11vnc | grep -F -- "-viewonly" | grep -F -- "-shared"
      pgrep -f websockify
    '
  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.viewer.yaml" \
    --profile viewer \
    exec -T gazebo-server /usr/local/bin/mentorpi-healthcheck server
  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.viewer.yaml" \
    --profile viewer \
    exec -T sim-adapter /usr/local/bin/mentorpi-healthcheck adapter
}

case "$mode" in
  lan) smoke_lan ;;
  viewer) smoke_viewer ;;
esac
