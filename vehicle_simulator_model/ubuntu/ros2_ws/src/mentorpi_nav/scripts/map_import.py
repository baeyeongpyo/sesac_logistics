#!/usr/bin/env python3
"""Publish a user-supplied static map as an immutable Nav2 map session."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import sys
from datetime import datetime, timezone

import yaml


MAP_ID_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9._-]*$')
REQUIRED_SOURCE_FILES = ('map.pgm', 'map.yaml')


def fail(message: str) -> None:
    print(f'map-import: {message}', file=sys.stderr)
    raise SystemExit(2)


def validate_map_id(map_id: str) -> None:
    if not MAP_ID_PATTERN.fullmatch(map_id) or map_id in ('.', '..'):
        fail('map ID may contain only A-Z, a-z, 0-9, period, underscore, and hyphen, but not . or ..')


def validate_source(source: Path) -> None:
    if not source.is_dir():
        fail(f'map source does not exist: {source}')
    for name in REQUIRED_SOURCE_FILES:
        path = source / name
        if not path.is_file() or path.stat().st_size == 0:
            fail(f'map source is missing a non-empty {name}: {source}')
    try:
        map_config = yaml.safe_load((source / 'map.yaml').read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as error:
        fail(f'map.yaml is not valid YAML: {error}')
    image = map_config.get('image') if isinstance(map_config, dict) else None
    if not isinstance(image, str) or Path(image).parts != ('map.pgm',):
        fail('map.yaml image must reference map.pgm in the imported session')


def write_checksums(session: Path) -> None:
    entries = []
    for path in sorted(session.iterdir(), key=lambda candidate: candidate.name):
        if path.is_file() and path.name != 'checksums.sha256':
            entries.append(f'{hashlib.sha256(path.read_bytes()).hexdigest()}  {path.name}')
    (session / 'checksums.sha256').write_text('\n'.join(entries) + '\n', encoding='utf-8')


def import_map(source_root: Path, destination_root: Path, map_id: str) -> Path:
    validate_map_id(map_id)
    source = source_root / map_id
    validate_source(source)

    destination_root.mkdir(parents=True, exist_ok=True)
    final = destination_root / map_id
    staging = destination_root / '.inprogress' / map_id
    if final.exists():
        fail(f'published map ID already exists: {map_id}')
    if staging.exists():
        fail(f'map ID is already being imported: {map_id}')

    staging.parent.mkdir(parents=True, exist_ok=True)
    try:
        staging.mkdir()
        for name in REQUIRED_SOURCE_FILES:
            shutil.copyfile(source / name, staging / name)
        manifest = {
            'session_id': map_id,
            'robot_id': 'external-map',
            'image_version': 'not-applicable',
            'git_commit': 'not-applicable',
            'world_version': 'not-applicable',
            'model_version': 'not-applicable',
            'slam_params_sha256': 'not-applicable',
            'tf_calibration_version': 'not-applicable',
            'created_at': datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%SZ'),
            'map_source': 'external-import',
        }
        (staging / 'manifest.json').write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + '\n', encoding='utf-8'
        )
        write_checksums(staging)
        staging.replace(final)
    except Exception:
        shutil.rmtree(staging, ignore_errors=True)
        raise
    return final


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=Path('/map-import'))
    parser.add_argument('--destination-root', type=Path, default=Path('/slam-data'))
    parser.add_argument('--map-id', default=os.getenv('MAP_IMPORT_ID', ''))
    args = parser.parse_args(argv)

    if not args.map_id:
        fail('map ID must be provided with --map-id or MAP_IMPORT_ID')

    session = import_map(args.source_root, args.destination_root, args.map_id)
    print(f'map-import: published map_id={args.map_id} session={session}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
