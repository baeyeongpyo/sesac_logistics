#!/usr/bin/env python3
"""Select only checksum-verified SLAM map sessions for localization."""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path
import sys
from typing import Optional


REQUIRED_FILES = ('map.yaml', 'map.pgm', 'manifest.json', 'checksums.sha256')


@dataclass(frozen=True)
class MapSession:
    session_id: str
    path: Path
    map_yaml: Path
    created_at: str


def _safe_relative_path(value: str) -> Optional[Path]:
    candidate = Path(value)
    if candidate.is_absolute() or '..' in candidate.parts or str(candidate) in ('', '.'):
        return None
    return candidate


def _checksums_match(session: Path) -> bool:
    checksum_file = session / 'checksums.sha256'
    try:
        lines = checksum_file.read_text(encoding='utf-8').splitlines()
    except OSError:
        return False
    expected = set(REQUIRED_FILES) - {'checksums.sha256'}
    actual = set()
    for line in lines:
        try:
            digest, relative = line.split('  ', 1)
        except ValueError:
            return False
        path = _safe_relative_path(relative)
        if path is None or len(digest) != 64 or any(char not in '0123456789abcdef' for char in digest):
            return False
        target = session / path
        if not target.is_file():
            return False
        if hashlib.sha256(target.read_bytes()).hexdigest() != digest:
            return False
        actual.add(path.as_posix())
    return expected.issubset(actual)


def validate_session(session: Path) -> Optional[MapSession]:
    if not session.is_dir() or any(not (session / name).is_file() for name in REQUIRED_FILES):
        return None
    if not _checksums_match(session):
        return None
    try:
        manifest = json.loads((session / 'manifest.json').read_text(encoding='utf-8'))
        session_id = manifest['session_id']
        created_at = manifest['created_at']
    except (KeyError, OSError, TypeError, ValueError):
        return None
    if not isinstance(session_id, str) or session_id != session.name or not isinstance(created_at, str):
        return None
    return MapSession(session_id, session, session / 'map.yaml', created_at)


def find_valid_session(root: Path, requested_id: Optional[str]) -> Optional[MapSession]:
    if requested_id:
        return validate_session(root / requested_id)
    candidates = (validate_session(path) for path in root.iterdir()) if root.is_dir() else ()
    valid = [session for session in candidates if session is not None]
    return max(valid, key=lambda session: (session.created_at, session.session_id), default=None)


def main(argv: list[str]) -> int:
    root = Path(argv[1]) if len(argv) > 1 else Path('/slam-data')
    requested = argv[2] if len(argv) > 2 and argv[2] else None
    session = find_valid_session(root, requested)
    if session is None:
        print('mapping\t\t')
    else:
        print(f'localization\t{session.session_id}\t{session.map_yaml}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
