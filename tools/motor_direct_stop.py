#!/usr/bin/env python3
"""User-run serial zero burst after all motion producers/drivers have stopped."""
import os
from pathlib import Path
import time


def zero_packet():
    # Use the board SDK's verified framing without constructing Board or a Node.
    import struct
    from ros_robot_controller.ros_robot_controller_sdk import PacketFunction, checksum_crc8
    data = bytes([1, 4]) + b''.join(struct.pack('<Bf', i, 0.0) for i in range(4))
    body = bytes([int(PacketFunction.PACKET_FUNC_MOTOR), len(data)]) + data
    return b'\xaa\x55' + body + bytes([checksum_crc8(body)])


def require_unused(device):
    target = os.stat(device).st_rdev
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            for fd in (proc / 'fd').iterdir():
                try:
                    if fd.stat().st_rdev == target:
                        raise RuntimeError(f'{device} still open by PID {proc.name}')
                except FileNotFoundError:
                    pass
        except (FileNotFoundError, ProcessLookupError):
            pass


def send_zero_burst(port, packet, pause=time.sleep):
    for _ in range(20):
        if port.write(packet) != len(packet):
            raise IOError('Incomplete motor zero packet')
        port.flush()
        pause(0.05)


def main():
    import serial
    device = '/dev/rrc'
    require_unused(device)
    packet = zero_packet()
    port = serial.Serial(None, 1000000, timeout=1, write_timeout=1, exclusive=True)
    port.rts = False
    port.dtr = False
    port.port = device
    try:
        port.open()
        send_zero_burst(port, packet)
    finally:
        port.close()
    print('Direct motor zeros transmitted and flushed; physical stop is not acknowledged by this protocol.')


if __name__ == '__main__':
    main()
