import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';

import {
  batteryPercent,
  clampZoom,
  clampMapPan,
  inventoryBreakdown,
  ManualControlSession,
  manualRepeatInterval,
  mapPointAtPanZoom,
  mapPointAtZoom,
  mapToPixel,
  manualCommand,
  parsePgm,
  pixelToMap,
  rotationHoldMs,
  svgRotationFromYaw,
  vehicleSizeInPixels,
  yawFromDrag,
} from './dashboard.js';

const map = {
  width: 196,
  height: 128,
  resolution: 0.05,
  origin: [-5.04, -4.03, 0],
};

test('map and pixel coordinates round-trip using the ROS occupancy convention', () => {
  const point = { x: -1.2, y: 0.75 };
  const pixel = mapToPixel(point.x, point.y, map);

  assert.deepEqual(pixel, { x: 76.8, y: 32.4 });
  assert.deepEqual(pixelToMap(pixel.x, pixel.y, map), point);
});

test('initial pose yaw follows the map-space direction of the drag', () => {
  assert.equal(yawFromDrag({ x: 50, y: 60 }, { x: 70, y: 60 }), 0);
  assert.equal(yawFromDrag({ x: 50, y: 60 }, { x: 50, y: 40 }), Math.PI / 2);
});

test('the operational PGM map decodes to its native dimensions', async () => {
  const source = new URL('../../monitoring/map_server/maps/map_0825.pgm', import.meta.url);
  const pgm = parsePgm(await readFile(source));

  assert.equal(pgm.width, 196);
  assert.equal(pgm.height, 128);
  assert.equal(pgm.pixels.length, 196 * 128);
});

test('battery raw values are clamped and converted from the 6500 to 8500 range', () => {
  assert.equal(batteryPercent(6500), 0);
  assert.equal(batteryPercent(7500), 50);
  assert.equal(batteryPercent(8500), 100);
  assert.equal(batteryPercent(9000), 100);
  assert.equal(batteryPercent(6200), 0);
});

test('inventory shows individual item quantities while retaining the zone total', () => {
  assert.equal(
    inventoryBreakdown([
      { payload_type: 'FRESH', quantity: 12 },
      { payload_type: 'NORMAL', quantity: 12 },
    ]),
    'FRESH 12 · NORMAL 12',
  );
  assert.equal(inventoryBreakdown([]), '재고 없음');
});

test('vehicle body rotation keeps ROS positive yaw pointing upward on the SVG map', () => {
  assert.equal(svgRotationFromYaw(0), 0);
  assert.equal(svgRotationFromYaw(Math.PI / 2), -90);
  assert.equal(svgRotationFromYaw(-Math.PI / 2), 90);
});

test('vehicle body uses MentorPi M1 physical dimensions at the map resolution', () => {
  assert.deepEqual(vehicleSizeInPixels(map), { length: 4.24, width: 3.42 });
});

test('manual controls use independently configured speeds and cap each turn at ten degrees', () => {
  const speeds = { drive: 0.7, strafe: 0.8, rotation: 0.5 };

  assert.deepEqual(manualCommand('forward', speeds), {
    linear_x: 0.7, linear_y: 0, angular_z: 0, hold_ms: 300,
  });
  assert.deepEqual(manualCommand('strafe-right', speeds), {
    linear_x: 0, linear_y: -0.8, angular_z: 0, hold_ms: 300,
  });
  assert.deepEqual(manualCommand('rotate-left', speeds), {
    linear_x: 0, linear_y: 0, angular_z: 0.5, hold_ms: 349,
  });
  assert.equal(rotationHoldMs(1), 174);
  assert.equal(rotationHoldMs(0.1), 1000);
});

test('manual control completion yields one stop target and never creates one for other controls', () => {
  const session = new ManualControlSession();

  assert.equal(session.complete(), null);
  session.begin('robot_1');
  assert.equal(session.complete(), 'robot_1');
  assert.equal(session.complete(), null);
});

test('held rotation renews its bounded command before it expires', () => {
  const command = manualCommand('rotate-left', {
    drive: 0.1,
    strafe: 0.1,
    rotation: 1,
  });

  assert.equal(command.hold_ms, 174);
  assert.ok(manualRepeatInterval(command) < command.hold_ms);
});

test('map zoom is constrained to readable minimum and maximum scales', () => {
  assert.equal(clampZoom(0.2), 0.5);
  assert.equal(clampZoom(1.25), 1.25);
  assert.equal(clampZoom(4), 3);
});

test('map panning is bounded by the enlarged map extent', () => {
  assert.deepEqual(
    clampMapPan({ x: 999, y: -999 }, map, 2),
    { x: 98, y: -64 },
  );
  assert.deepEqual(clampMapPan({ x: 10, y: -10 }, map, 1), { x: 0, y: 0 });
});

test('initial pose point remains aligned after zooming and panning', () => {
  assert.deepEqual(
    mapPointAtPanZoom({ x: 148, y: 64 }, map, 2, { x: 25, y: -10 }),
    { x: 110.5, y: 69 },
  );
});

test('initial pose point remains aligned with the map while zoomed around its center', () => {
  assert.deepEqual(
    mapPointAtZoom({ x: 148, y: 64 }, map, 2),
    { x: 123, y: 64 },
  );
});
