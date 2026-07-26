#!/usr/bin/env python3
"""Atomically publish a completed session without replacing an existing target."""

import ctypes
import errno
import os
from pathlib import Path
import platform
import sys


AT_FDCWD = -100
RENAME_NOREPLACE = 1
RENAMEAT2_BY_MACHINE = {
    'x86_64': 316,
    'amd64': 316,
    'aarch64': 276,
    'arm64': 276,
}


def fail(message: str, exit_code: int = 1) -> int:
    print(f'atomic_publish: {message}', file=sys.stderr)
    return exit_code


def rename_noreplace(stage: Path, final: Path) -> None:
    if sys.platform != 'linux':
        raise OSError(errno.ENOSYS, 'renameat2 is only available on Linux')
    syscall_number = RENAMEAT2_BY_MACHINE.get(platform.machine().lower())
    if syscall_number is None:
        raise OSError(errno.ENOSYS, f'unsupported Linux architecture: {platform.machine()}')

    libc = ctypes.CDLL(None, use_errno=True)
    result = libc.syscall(
        ctypes.c_long(syscall_number),
        ctypes.c_int(AT_FDCWD), ctypes.c_char_p(os.fsencode(stage)),
        ctypes.c_int(AT_FDCWD), ctypes.c_char_p(os.fsencode(final)),
        ctypes.c_uint(RENAME_NOREPLACE),
    )
    if result != 0:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))


def main(arguments: list[str]) -> int:
    if len(arguments) != 2:
        return fail('usage: atomic_publish.py STAGE_DIR FINAL_DIR', 2)
    stage = Path(arguments[0])
    final = Path(arguments[1])
    if not stage.is_dir():
        return fail(f'stage directory does not exist: {stage}', 2)
    if stage.parent.stat().st_dev != final.parent.stat().st_dev:
        return fail('stage and final parents must be on the same filesystem', 2)
    try:
        rename_noreplace(stage, final)
    except OSError as error:
        if error.errno == errno.EEXIST:
            return fail(f'final path already exists: {final}')
        if error.errno in (errno.ENOSYS, errno.EINVAL):
            return fail(f'renameat2(RENAME_NOREPLACE) is unavailable: {error.strerror}', 2)
        return fail(f'cannot publish {stage} to {final}: {error.strerror}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
