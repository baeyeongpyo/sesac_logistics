"""Deterministic session artifact metadata and integrity files."""

import hashlib
import json
import os
from pathlib import Path
import stat
from typing import Mapping


REQUIRED_FIELDS = (
    'session_id', 'robot_id', 'image_version', 'git_commit',
    'world_version', 'model_version', 'slam_params_sha256',
    'tf_calibration_version', 'created_at',
)


def write_manifest(session_dir: Path, metadata: Mapping[str, str]) -> Path:
    """Write validated session metadata using a deterministic JSON format."""
    for field in REQUIRED_FIELDS:
        if not metadata.get(field):
            raise ValueError(f'missing or empty required metadata field: {field}')

    manifest_path = session_dir / 'manifest.json'
    temporary_path = session_dir / 'manifest.json.tmp'
    with temporary_path.open('w', encoding='utf-8') as stream:
        json.dump(metadata, stream, indent=2, sort_keys=True)
        stream.write('\n')
    temporary_path.replace(manifest_path)
    return manifest_path


def write_checksums(session_dir: Path) -> Path:
    """Hash regular session files in POSIX-relative-path order."""
    checksum_path = session_dir / 'checksums.sha256'
    files = []
    for root, _, names in os.walk(session_dir, followlinks=False):
        for name in names:
            path = Path(root) / name
            if path == checksum_path or path.name.endswith('.tmp'):
                continue
            if not stat.S_ISREG(path.lstat().st_mode):
                continue
            files.append((path.relative_to(session_dir).as_posix(), path))

    lines = []
    for relative_path, path in sorted(files):
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        lines.append(f'{digest}  {relative_path}')
    checksum_path.write_text('\n'.join(lines) + ('\n' if lines else ''), encoding='utf-8')
    return checksum_path
