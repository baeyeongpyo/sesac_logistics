import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

import '../model.mjs';

const {
  canvasToMapPoint,
  canvasVectorToYaw,
  computeNavigation,
  createMapTransform,
  createOccupancyMap,
  gridToWorld,
  parseMapYaml,
  parsePgm,
  worldToGrid,
  zoomMapView,
} = globalThis.Nav2PathModel;

function createRouteTestMap(blockedCells = []) {
  const width = 41;
  const height = 31;
  const pixels = new Array(width * height).fill(255);
  const blocked = new Set(blockedCells.map(({ column, row }) => `${column},${row}`));

  for (let row = 0; row < height; row += 1) {
    for (let column = 0; column < width; column += 1) {
      const isBorder = row === 0 || row === height - 1 || column === 0 || column === width - 1;
      const isWall = column === 20 && (row < 10 || row > 20);
      if (isBorder || isWall || blocked.has(`${column},${row}`)) {
        pixels[(row * width) + column] = 0;
      }
    }
  }

  return createOccupancyMap(
    { width, height, maxValue: 255, pixels },
    { resolution: 0.1, origin: [0, 0], negate: 0, occupiedThresh: 0.65, freeThresh: 0.25 },
  );
}

function poseAtCell(map, column, row, yaw = 0) {
  return { ...gridToWorld(map, { column, row }), yaw };
}

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

test('canvas hover position returns continuous map coordinates and occupancy', () => {
  const map = createOccupancyMap(
    { width: 4, height: 4, maxValue: 255, pixels: new Array(16).fill(255) },
    { resolution: 0.5, origin: [-1, -2], negate: 0, occupiedThresh: 0.65, freeThresh: 0.25 },
  );

  assert.equal(typeof canvasToMapPoint, 'function');
  assert.deepEqual(
    canvasToMapPoint(map, { left: 40, top: 20, scale: 20 }, { x: 70, y: 70 }),
    { x: -0.25, y: -1.25, column: 1, row: 2, occupancy: 'free' },
  );
  assert.equal(canvasToMapPoint(map, { left: 40, top: 20, scale: 20 }, { x: 15, y: 70 }), undefined);
});

test('pointer-centred zoom preserves the hovered map coordinate', () => {
  const map = createOccupancyMap(
    { width: 4, height: 4, maxValue: 255, pixels: new Array(16).fill(255) },
    { resolution: 0.5, origin: [0, 0], negate: 0, occupiedThresh: 0.65, freeThresh: 0.25 },
  );
  const anchor = { x: 76, y: 56 };
  const initialView = { zoom: 1, offsetX: 0, offsetY: 0 };

  assert.equal(typeof createMapTransform, 'function');
  assert.equal(typeof zoomMapView, 'function');
  const before = canvasToMapPoint(map, createMapTransform(map, 200, 160, initialView), anchor);
  const zoomedView = zoomMapView(initialView, 2, anchor, 200, 160);
  const after = canvasToMapPoint(map, createMapTransform(map, 200, 160, zoomedView), anchor);

  assert.deepEqual(zoomedView, { zoom: 2, offsetX: 24, offsetY: 24 });
  assert.deepEqual(after, before);
});

test('drag direction converts from canvas axes into Nav2 yaw', () => {
  assert.equal(typeof canvasVectorToYaw, 'function');
  assert.equal(canvasVectorToYaw({ x: 10, y: 0 }), 0);
  assert.equal(canvasVectorToYaw({ x: 0, y: -10 }), Math.PI / 2);
  assert.equal(canvasVectorToYaw({ x: 0, y: 10 }), -Math.PI / 2);
});

test('navigation result exposes global, transformed, and selected local plans', () => {
  const map = createRouteTestMap();
  const start = poseAtCell(map, 5, 15);
  const goal = poseAtCell(map, 35, 15);

  const result = computeNavigation({ map, start, goal });

  assert.equal(result.ok, true);
  assert.ok(result.globalPlan.length > 1);
  assert.ok(result.transformedGlobalPlan.length > 1);
  assert.ok(result.transformedGlobalPlan.length <= result.globalPlan.length);
  assert.ok(result.localPlan.length > 1);
  assert.equal(result.summary.candidateCount, 300);
  assert.ok(Number.isFinite(result.summary.selectedVx));
  assert.ok(Number.isFinite(result.summary.selectedVtheta));
  assert.ok(result.transformedGlobalPlan.every((pose) => Math.hypot(pose.x - start.x, pose.y - start.y) <= 1.5));
});

test('navigation returns a clear input error when the start cell is blocked', () => {
  const map = createRouteTestMap([{ column: 5, row: 15 }]);
  const result = computeNavigation({
    map,
    start: poseAtCell(map, 5, 15),
    goal: poseAtCell(map, 35, 15),
  });

  assert.deepEqual(result, {
    ok: false,
    code: 'START_IN_COLLISION',
    message: '시작 위치가 지도 밖이거나 충돌 영역에 있습니다.',
  });
});

test('project P5 map produces all plan layers without a ROS runtime', () => {
  const pgm = parsePgm(readFileSync(new URL('../../../../map_server/maps/map_0825.pgm', import.meta.url)));
  const metadata = parseMapYaml(readFileSync(new URL('../../../../map_server/maps/map_0825.yaml', import.meta.url), 'utf8'));
  const result = computeNavigation({
    map: createOccupancyMap(pgm, metadata),
    start: { x: -4, y: -3, yaw: 0 },
    goal: { x: 3.8, y: 2, yaw: 0 },
  });

  assert.equal(result.ok, true);
  assert.ok(result.globalPlan.length > result.transformedGlobalPlan.length);
  assert.ok(result.transformedGlobalPlan.length > 1);
  assert.ok(result.localPlan.length > 1);
  assert.equal(result.summary.candidateCount, 300);
});
