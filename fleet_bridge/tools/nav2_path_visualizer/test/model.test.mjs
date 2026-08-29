import test from 'node:test';
import assert from 'node:assert/strict';

import {
  createOccupancyMap,
  gridToWorld,
  parseMapYaml,
  parsePgm,
  worldToGrid,
} from '../model.mjs';

test('PGM parser reads P2 pixels while ignoring comments', () => {
  const bytes = new TextEncoder().encode('P2\n# warehouse map\n3 2\n255\n0 127 255\n255 0 255\n');

  assert.deepEqual(parsePgm(bytes), {
    width: 3,
    height: 2,
    maxValue: 255,
    pixels: [0, 127, 255, 255, 0, 255],
  });
});

test('PGM parser reads P5 pixels after the binary header', () => {
  const header = new TextEncoder().encode('P5\n2 1\n255\n');
  const bytes = new Uint8Array([...header, 12, 240]);

  assert.deepEqual(parsePgm(bytes), {
    width: 2,
    height: 1,
    maxValue: 255,
    pixels: [12, 240],
  });
});

test('map metadata preserves configured occupancy thresholds', () => {
  const metadata = parseMapYaml([
    'resolution: 0.5',
    'origin: [-1, -2, 0]',
    'negate: 0',
    'occupied_thresh: 0.65',
    'free_thresh: 0.25',
  ].join('\n'));

  assert.deepEqual(metadata, {
    resolution: 0.5,
    origin: [-1, -2],
    negate: 0,
    occupiedThresh: 0.65,
    freeThresh: 0.25,
  });
});

test('world and grid coordinates meet at the cell center', () => {
  const map = createOccupancyMap(
    { width: 4, height: 4, maxValue: 255, pixels: new Array(16).fill(255) },
    { resolution: 0.5, origin: [-1, -2], negate: 0, occupiedThresh: 0.65, freeThresh: 0.25 },
  );

  const cell = worldToGrid(map, { x: -0.25, y: -0.75 });

  assert.deepEqual(cell, { column: 1, row: 1 });
  assert.deepEqual(gridToWorld(map, cell), { x: -0.25, y: -0.75 });
});
