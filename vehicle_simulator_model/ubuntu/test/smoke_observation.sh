#!/usr/bin/env bash
set -euo pipefail

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
import struct
from urllib.parse import urlsplit

url = urlsplit(os.environ['VIEWER_WEBSOCKET_URL'])
host = url.hostname or '127.0.0.1'
port = url.port or 80
path = url.path or '/'
if url.query:
    path += f'?{url.query}'


class RFBWebSocketClient:
    def __init__(self, label):
        self.label = label
        self.socket = None
        self.socket_buffer = bytearray()
        self.rfb_buffer = bytearray()
        self.width = 0
        self.height = 0

    def connect(self):
        key = base64.b64encode(
            hashlib.sha256(
                f'mentorpi-viewer-{self.label}'.encode()
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
        self.socket = socket.create_connection((host, port), timeout=10)
        self.socket.sendall(request)
        response = b''
        while b'\r\n\r\n' not in response:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise RuntimeError('connection closed during handshake')
            response += chunk
            if len(response) > 16384:
                raise RuntimeError('oversized handshake response')
        header_bytes, remainder = response.split(b'\r\n\r\n', 1)
        self.socket_buffer.extend(remainder)
        header = header_bytes.decode(
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

        try:
            banner = self.receive_rfb(12)
        except EOFError as error:
            raise RuntimeError(
                f'{self.label} missing RFB banner: {error}'
            ) from error
        if banner != b'RFB 003.008\n':
            raise RuntimeError(
                f'{self.label} invalid RFB banner: {banner!r}'
            )
        self.send_binary(b'RFB 003.008\n')
        security_type_count = self.receive_rfb(1)[0]
        if security_type_count == 0:
            reason_length = struct.unpack('>I', self.receive_rfb(4))[0]
            reason = self.receive_rfb(reason_length).decode(
                'utf-8', errors='replace'
            )
            raise RuntimeError(f'{self.label} RFB security failure: {reason}')
        security_types = self.receive_rfb(security_type_count)
        if 1 not in security_types:
            raise RuntimeError(
                f'{self.label} RFB no-auth type unavailable: '
                f'{security_types!r}'
            )
        self.send_binary(b'\x01')
        security_result = struct.unpack('>I', self.receive_rfb(4))[0]
        if security_result != 0:
            raise RuntimeError(
                f'{self.label} RFB security result={security_result}'
            )
        self.send_binary(b'\x01')
        server_init = self.receive_rfb(24)
        self.width, self.height = struct.unpack('>HH', server_init[:4])
        name_length = struct.unpack('>I', server_init[20:24])[0]
        server_name = self.receive_rfb(name_length).decode(
            'utf-8', errors='replace'
        )
        print(
            f'viewer websocket {self.label}=rfb-ready '
            f'size={self.width}x{self.height} name={server_name!r}'
        )

    def receive_exact(self, length):
        data = bytearray()
        if self.socket_buffer:
            take = min(length, len(self.socket_buffer))
            data.extend(self.socket_buffer[:take])
            del self.socket_buffer[:take]
        while len(data) < length:
            chunk = self.socket.recv(length - len(data))
            if not chunk:
                raise EOFError(f'{self.label} WebSocket closed')
            data.extend(chunk)
        return bytes(data)

    def receive_frame(self):
        first, second = self.receive_exact(2)
        opcode = first & 0x0f
        length = second & 0x7f
        if length == 126:
            length = struct.unpack('>H', self.receive_exact(2))[0]
        elif length == 127:
            length = struct.unpack('>Q', self.receive_exact(8))[0]
        if second & 0x80:
            raise RuntimeError(f'{self.label} masked server frame')
        payload = self.receive_exact(length)
        if opcode == 8:
            raise EOFError(f'{self.label} WebSocket close frame')
        if opcode == 9:
            self.send_frame(payload, opcode=10)
            return self.receive_frame()
        if opcode not in (0, 2):
            raise RuntimeError(
                f'{self.label} unexpected WebSocket opcode={opcode}'
            )
        return payload

    def receive_rfb(self, length):
        while len(self.rfb_buffer) < length:
            self.rfb_buffer.extend(self.receive_frame())
        data = bytes(self.rfb_buffer[:length])
        del self.rfb_buffer[:length]
        return data

    def send_frame(self, payload, opcode=2):
        mask = os.urandom(4)
        length = len(payload)
        header = bytes([0x80 | opcode])
        if length < 126:
            header += bytes([0x80 | length])
        elif length < 65536:
            header += b'\xfe' + struct.pack('>H', length)
        else:
            header += b'\xff' + struct.pack('>Q', length)
        masked = bytes(
            value ^ mask[index % 4]
            for index, value in enumerate(payload)
        )
        self.socket.sendall(header + mask + masked)

    def send_binary(self, payload):
        self.send_frame(payload, opcode=2)

    def prove_survival(self, closed_label):
        request = struct.pack(
            '>BBHHHH',
            3, 0, 0, 0, self.width, self.height,
        )
        self.send_binary(request)
        update_header = self.receive_rfb(4)
        if update_header[0] != 0:
            raise RuntimeError(
                f'{self.label} survivor received invalid RFB payload '
                f'after {closed_label} closed: {update_header!r}'
            )
        rectangles = struct.unpack('>H', update_header[2:4])[0]
        print(
            f'viewer websocket survivor={self.label} '
            f'after={closed_label} rectangles={rectangles}'
        )

    def close(self):
        if self.socket is None:
            return
        try:
            self.send_frame(b'', opcode=8)
        except OSError:
            pass
        self.socket.close()
        self.socket = None


def prove_close_order(first_label, survivor_label):
    first = RFBWebSocketClient(first_label)
    survivor = RFBWebSocketClient(survivor_label)
    try:
        first.connect()
        survivor.connect()
        first.close()
        survivor.prove_survival(first_label)
    finally:
        first.close()
        survivor.close()


try:
    prove_close_order('client-a', 'client-b')
    prove_close_order('client-b', 'client-a')
except Exception as error:
    raise SystemExit(f'viewer websocket survivor check failed: {error}')
PY
}

check_viewer_processes() {
  docker compose \
    -f "$bundle_dir/compose.yaml" \
    -f "$bundle_dir/compose.viewer.yaml" \
    --profile viewer \
    exec -T gazebo-viewer \
    python3 - <<'PY'
import os
import sys
from pathlib import Path


def has_sequence(arguments, required):
    width = len(required)
    return any(
        arguments[index:index + width] == required
        for index in range(len(arguments) - width + 1)
    )


gz_command = 'gz sim --force-version 8 -g'.split()

specifications = (
    (
        'gz-gui',
        lambda comm, executable: (
            comm.startswith('ruby') or executable.startswith('ruby')
        ),
        ['/usr/bin/gz', *gz_command[1:]],
    ),
    (
        'Xvfb',
        lambda comm, executable: (
            comm == 'Xvfb' and executable in ('Xvfb', 'rosetta')
        ),
        ['Xvfb', ':99', '-screen', '0'],
    ),
    (
        'x11vnc-viewonly-shared',
        lambda comm, executable: (
            comm == 'x11vnc' and executable in ('x11vnc', 'rosetta')
        ),
        ['-forever', '-shared', '-viewonly'],
    ),
    (
        'websockify',
        lambda comm, executable: (
            comm.startswith('python3')
            and (executable.startswith('python3') or executable == 'rosetta')
        ),
        [
            '/usr/bin/websockify',
            '--web=/usr/share/novnc',
            '6080',
            'localhost:5900',
        ],
    ),
)

processes = []
for proc_dir in Path('/proc').glob('[0-9]*'):
    try:
        comm = (proc_dir / 'comm').read_text().strip()
        executable = Path(os.readlink(proc_dir / 'exe')).name
        arguments = [
            value.decode(errors='replace')
            for value in (proc_dir / 'cmdline').read_bytes().split(b'\0')
            if value
        ]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    processes.append((proc_dir.name, comm, executable, arguments))

missing = []
for label, identity_matches, required in specifications:
    match = next(
        (
            process
            for process in processes
            if identity_matches(process[1], process[2])
            and has_sequence(process[3], required)
        ),
        None,
    )
    if match is None:
        missing.append(label)
        continue
    pid, comm, executable, arguments = match
    print(
        f'viewer process={label} pid={pid} comm={comm} '
        f'exe={executable} argv={arguments!r}'
    )

if missing:
    print(
        f'missing viewer process: {", ".join(missing)}',
        file=sys.stderr,
    )
    raise SystemExit(1)
PY
}

smoke_viewer() {
  viewer_url="http://127.0.0.1:${VIEWER_PORT:-8080}/vnc.html?view_only=1&autoconnect=1"
  websocket_url="ws://127.0.0.1:${VIEWER_PORT:-8080}/websockify"
  curl -fsS "$viewer_url" | grep -Fq '<title>noVNC</title>'
  check_viewer_websockets "$websocket_url"
  check_viewer_processes
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

main() {
  local mode="${1:-}"
  if [[ "$#" -ne 1 || "$mode" != lan && "$mode" != viewer ]]; then
    echo 'Usage: smoke_observation.sh <lan|viewer>' >&2
    return 2
  fi

  case "$mode" in
    lan) smoke_lan ;;
    viewer) smoke_viewer ;;
  esac
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  main "$@"
fi
