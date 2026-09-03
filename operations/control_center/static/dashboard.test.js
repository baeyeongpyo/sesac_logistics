import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import * as dashboard from './dashboard.js';

import {
  batteryPercent,
  clampZoom,
  clampMapPan,
  connectivityAgeLabel,
  connectivityLabel,
  displayedFleetState,
  fleetStateLabel,
  inventoryBreakdown,
  ManualControlSession,
  manualRepeatInterval,
  mapPointAtPanZoom,
  mapPointAtZoom,
  mapToPixel,
  manualCommand,
  NavigationExecutionSession,
  NavigationDraft,
  navigationExecutionMatchesDraft,
  parsePgm,
  pixelToMap,
  rotationHoldMs,
  scheduleSnapshotRefresh,
  svgRotationFromYaw,
  taskLabel,
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

test('navigation draft keeps initial and goal poses pending independently until execution', () => {
  const draft = new NavigationDraft();
  const initialPose = { x: -1.2, y: 0.75, yaw: 0.4 };
  const goalPose = { x: 2.4, y: -0.5, yaw: -0.8 };

  draft.setPose('initial', initialPose);
  draft.setPose('goal', goalPose);

  assert.deepEqual(draft.initialPose, initialPose);
  assert.deepEqual(draft.goalPose, goalPose);
  assert.equal(draft.hasPending('initial'), true);
  assert.equal(draft.hasPending('goal'), true);
});

test('navigation draft keeps follow waypoints in selection order and renumbers after removal', () => {
  const draft = new NavigationDraft();
  draft.addWaypoint({ x: 0.5, y: 0, yaw: 0 });
  draft.addWaypoint({ x: 1.0, y: 0.5, yaw: 0.4 });
  draft.addWaypoint({ x: 1.5, y: 1.0, yaw: 0.8 });

  draft.removeWaypoint(1);

  assert.deepEqual(draft.waypoints, [
    { x: 0.5, y: 0, yaw: 0 },
    { x: 1.5, y: 1.0, yaw: 0.8 },
  ]);
  assert.equal(draft.hasPending('waypoints'), true);
  draft.clearWaypoints();
  assert.equal(draft.hasPending('waypoints'), false);
});

test('navigation execution session accepts one request and only clears its owning vehicle draft', () => {
  const session = new NavigationExecutionSession();
  const execution = session.begin('robot_1', 'waypoints', 4);

  assert.deepEqual(execution, { robotId: 'robot_1', mode: 'waypoints', draftRevision: 4 });
  assert.equal(session.begin('robot_1', 'waypoints', 5), null);
  assert.equal(session.isPending(), true);
  assert.equal(navigationExecutionMatchesDraft(execution, 'robot_2', { waypoints: 4 }), false);
  assert.equal(navigationExecutionMatchesDraft(execution, 'robot_1', { waypoints: 5 }), false);
  assert.equal(
    navigationExecutionMatchesDraft(execution, 'robot_1', {
      initial: 9, goal: 3, waypoints: 4,
    }),
    true,
  );
  assert.equal(session.complete(execution), true);
  assert.equal(session.isPending(), false);
});

test('the operational PGM map decodes to its native dimensions', async () => {
  const source = new URL('../../monitoring/map_server/maps/map_0825.pgm', import.meta.url);
  const pgm = parsePgm(await readFile(source));

  assert.equal(pgm.width, 196);
  assert.equal(pgm.height, 128);
  assert.equal(pgm.pixels.length, 196 * 128);
});

test('rotated operational map view crops unknown margin while preserving map input coordinates', async () => {
  assert.equal(typeof dashboard.operationalMapView, 'function');
  assert.equal(typeof dashboard.mapPixelToRotatedDisplay, 'function');
  assert.equal(typeof dashboard.rotatedDisplayToMapPixel, 'function');
  assert.equal(typeof dashboard.mapYawToRotatedDisplay, 'function');
  assert.equal(typeof dashboard.rotatedDisplayYawToMapYaw, 'function');

  const source = new URL('../../monitoring/map_server/maps/map_0825.pgm', import.meta.url);
  const view = dashboard.operationalMapView(dashboard.parsePgm(await readFile(source)));

  assert.ok(view.width > view.height);
  assert.ok(view.sourceBounds.x > 0 && view.sourceBounds.y > 0);
  assert.ok(80 - view.sourceBounds.x >= 8);
  assert.ok(43 - view.sourceBounds.y >= 8);
  assert.ok(view.sourceBounds.x + view.sourceBounds.width - 121 >= 8);
  assert.ok(view.sourceBounds.y + view.sourceBounds.height - 109 >= 8);

  const fixedView = {
    sourceBounds: { x: 10, y: 20, width: 60, height: 100 },
    width: 100,
    height: 60,
  };
  const displayPoint = dashboard.mapPixelToRotatedDisplay({ x: 25, y: 35 }, fixedView);
  assert.deepEqual(displayPoint, { x: 85, y: 15 });
  assert.deepEqual(dashboard.rotatedDisplayToMapPixel(displayPoint, fixedView), { x: 25, y: 35 });
  assert.equal(dashboard.mapYawToRotatedDisplay(0), -Math.PI / 2);
  assert.equal(dashboard.rotatedDisplayYawToMapYaw(0), Math.PI / 2);
});

test('battery raw values are clamped and converted from the 6500 to 8500 range', () => {
  assert.equal(batteryPercent(6500), 0);
  assert.equal(batteryPercent(7500), 50);
  assert.equal(batteryPercent(8500), 100);
  assert.equal(batteryPercent(9000), 100);
  assert.equal(batteryPercent(6200), 0);
});

test('telemetry connection labels distinguish fresh, delayed, disconnected, and unconfirmed vehicles', () => {
  assert.equal(connectivityLabel('online'), '연결됨');
  assert.equal(connectivityLabel('stale'), '응답 지연');
  assert.equal(connectivityLabel('offline'), '연결 끊김');
  assert.equal(connectivityLabel('unconfirmed'), '수신 이력 없음');
});

test('disconnected vehicles hide their last telemetry age', () => {
  assert.equal(connectivityAgeLabel({ state: 'online', age_sec: 7 }), ' · 7초 전');
  assert.equal(connectivityAgeLabel({ state: 'stale', age_sec: 29 }), ' · 29초 전');
  assert.equal(connectivityAgeLabel({ state: 'offline', age_sec: 960 }), '');
  assert.equal(connectivityAgeLabel({ state: 'unconfirmed', age_sec: null }), '');
});

test('dashboard schedules snapshot refresh once per second', () => {
  const calls = [];
  const refresh = () => {};

  scheduleSnapshotRefresh(refresh, (callback, interval) => {
    calls.push({ callback, interval });
    return 'poll-timer';
  });

  assert.deepEqual(calls, [{ callback: refresh, interval: 1000 }]);
});

test('vehicle list uses human-readable operational state and task names instead of task IDs', () => {
  assert.equal(fleetStateLabel('DRIVE'), '주행 중');
  assert.equal(fleetStateLabel('FAIL'), '오류');
  assert.equal(fleetStateLabel('STOPPED'), '정지');
  assert.equal(taskLabel({ status: 'TO_PICK' }, 'DRIVE'), '픽업 구역으로 이동');
  assert.equal(taskLabel({ status: 'TO_PLACE' }, 'DRIVE'), '적치 구역으로 이동');
  assert.equal(taskLabel({ status: 'TO_PICK', operation_id: 'opaque-operation-id' }, 'FAIL'), '운행 복구 대기');
  assert.equal(taskLabel(null, 'STOPPED'), '정지 요청됨');
  assert.equal(taskLabel(null, 'WAIT'), '할당된 작업 없음');
});

test('dashboard prefers the UI-only navigation display state over the vehicle control state', () => {
  assert.equal(displayedFleetState({
    fleet_state: { state: 'DRIVE' },
    display_state: 'AUTO_DRIVE',
  }), 'AUTO_DRIVE');
  assert.equal(displayedFleetState({
    fleet_state: { state: 'DRIVE' },
  }), 'DRIVE');
  assert.equal(fleetStateLabel('AUTO_DRIVE'), '자동 주행 중');
  assert.equal(fleetStateLabel('MANUAL_DRIVE'), '수동 주행 중');
});

test('switching to a different navigation mode clears every existing point', () => {
  const draft = new NavigationDraft();
  draft.selectMode('initial');
  draft.setPose('initial', { x: -1.2, y: 0.75, yaw: 0.4 });

  draft.selectMode('goal');

  assert.equal(draft.initialPose, null);
  assert.equal(draft.goalPose, null);
  assert.deepEqual(draft.waypoints, []);
});

test('reselecting the current navigation mode keeps its existing points', () => {
  const draft = new NavigationDraft();
  const initialPose = { x: -1.2, y: 0.75, yaw: 0.4 };
  draft.selectMode('initial');
  draft.setPose('initial', initialPose);

  draft.selectMode('initial');

  assert.deepEqual(draft.initialPose, initialPose);
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
