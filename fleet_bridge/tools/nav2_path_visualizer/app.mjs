const {
  canvasToMapPoint,
  canvasVectorToYaw,
  computeNavigation,
  createExampleMap,
  createMapTransform,
  createOccupancyMap,
  parseMapYaml,
  parsePgm,
  toNav2Pose,
  zoomMapView,
} = globalThis.Nav2PathModel ?? {};

if (!computeNavigation || !canvasToMapPoint || !createMapTransform || !toNav2Pose || !zoomMapView) {
  throw new Error('Nav2PathModel 초기화에 실패했습니다.');
}

document.documentElement.dataset.nav2PathRuntime = 'loading';

const DRAG_HEADING_THRESHOLD = 8;
const ZOOM_STEP = 1.25;

const elements = {
  pgmFile: document.querySelector('#pgm-file'),
  yamlFile: document.querySelector('#yaml-file'),
  loadMap: document.querySelector('#load-map'),
  useExampleMap: document.querySelector('#use-example-map'),
  runNavigation: document.querySelector('#run-navigation'),
  pickStart: document.querySelector('#pick-start'),
  pickGoal: document.querySelector('#pick-goal'),
  pickHint: document.querySelector('#pick-hint'),
  mapSource: document.querySelector('#map-source'),
  status: document.querySelector('#status-message'),
  canvas: document.querySelector('#map-canvas'),
  hoverReadout: document.querySelector('#map-hover-readout'),
  zoomOut: document.querySelector('#zoom-out'),
  zoomFit: document.querySelector('#zoom-fit'),
  zoomIn: document.querySelector('#zoom-in'),
  zoomLevel: document.querySelector('#zoom-level'),
  emptyState: document.querySelector('#empty-state'),
  metricGlobal: document.querySelector('#metric-global'),
  metricTransformed: document.querySelector('#metric-transformed'),
  metricLocal: document.querySelector('#metric-local'),
  metricCandidates: document.querySelector('#metric-candidates'),
  metricVelocity: document.querySelector('#metric-velocity'),
  metricScore: document.querySelector('#metric-score'),
  startNav2Pose: document.querySelector('#start-nav2-pose'),
  startNav2X: document.querySelector('#start-nav2-x'),
  startNav2Y: document.querySelector('#start-nav2-y'),
  startYawDegrees: document.querySelector('#start-yaw-deg'),
  startYawRadians: document.querySelector('#start-yaw-rad'),
  startQuaternionZ: document.querySelector('#start-quaternion-z'),
  startQuaternionW: document.querySelector('#start-quaternion-w'),
  goalNav2Pose: document.querySelector('#goal-nav2-pose'),
  goalNav2X: document.querySelector('#goal-nav2-x'),
  goalNav2Y: document.querySelector('#goal-nav2-y'),
  goalYawDegrees: document.querySelector('#goal-yaw-deg'),
  goalYawRadians: document.querySelector('#goal-yaw-rad'),
  goalQuaternionZ: document.querySelector('#goal-quaternion-z'),
  goalQuaternionW: document.querySelector('#goal-quaternion-w'),
  goalNav2Command: document.querySelector('#goal-nav2-command'),
  copyGoalCommand: document.querySelector('#copy-goal-command'),
};

const state = {
  map: createExampleMap(),
  mapName: '내장 예제 지도',
  result: undefined,
  start: undefined,
  goal: undefined,
  hover: undefined,
  pickMode: undefined,
  pointerStart: undefined,
  pointerEnd: undefined,
  view: { zoom: 1, offsetX: 0, offsetY: 0 },
};

function numberInput(id) {
  return Number(document.querySelector(`#${id}`).value);
}

function readPose(prefix) {
  return {
    x: numberInput(`${prefix}-x`),
    y: numberInput(`${prefix}-y`),
    yaw: numberInput(`${prefix}-yaw`) * (Math.PI / 180),
  };
}

function hasFinitePose(pose) {
  return Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.yaw);
}

function formatPoseNumber(value, digits, unit = '') {
  return Number.isFinite(value) ? `${value.toFixed(digits)}${unit ? ` ${unit}` : ''}` : '—';
}

function createNavigateToPoseCommand(nav2Pose) {
  const { position, orientation } = nav2Pose;
  return `ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: 'map'}, pose: {position: {x: ${position.x.toFixed(4)}, y: ${position.y.toFixed(4)}, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ${orientation.z.toFixed(6)}, w: ${orientation.w.toFixed(6)}}}}}"`;
}

function renderNav2Pose(prefix) {
  const pose = readPose(prefix);
  const valid = hasFinitePose(pose);
  const nav2Pose = valid ? toNav2Pose(pose) : undefined;
  const idPrefix = prefix === 'start' ? 'start' : 'goal';
  const poseContainer = elements[`${idPrefix}Nav2Pose`];

  poseContainer.classList.toggle('is-invalid', !valid);
  elements[`${idPrefix}Nav2X`].textContent = valid ? formatPoseNumber(nav2Pose.position.x, 3, 'm') : '—';
  elements[`${idPrefix}Nav2Y`].textContent = valid ? formatPoseNumber(nav2Pose.position.y, 3, 'm') : '—';
  elements[`${idPrefix}YawDegrees`].textContent = valid ? formatPoseNumber(nav2Pose.yawDegrees, 1, '°') : '—';
  elements[`${idPrefix}YawRadians`].textContent = valid ? formatPoseNumber(nav2Pose.yawRadians, 4, 'rad') : '—';
  elements[`${idPrefix}QuaternionZ`].textContent = valid ? formatPoseNumber(nav2Pose.orientation.z, 6) : '—';
  elements[`${idPrefix}QuaternionW`].textContent = valid ? formatPoseNumber(nav2Pose.orientation.w, 6) : '—';

  if (prefix === 'goal') {
    elements.goalNav2Command.textContent = valid
      ? createNavigateToPoseCommand(nav2Pose)
      : '유효한 Goal x, y, yaw 값을 입력하면 실행 명령을 생성합니다.';
    elements.copyGoalCommand.disabled = !valid;
  }
}

function renderNav2Poses() {
  renderNav2Pose('start');
  renderNav2Pose('goal');
}

function setStatus(message, kind = '') {
  elements.status.textContent = message;
  elements.status.className = `status-message${kind ? ` is-${kind}` : ''}`;
}

function setMapSource(message) {
  elements.mapSource.textContent = message;
}

function renderMetrics(result) {
  if (!result?.ok) {
    for (const element of [
      elements.metricGlobal,
      elements.metricTransformed,
      elements.metricLocal,
      elements.metricCandidates,
      elements.metricVelocity,
      elements.metricScore,
    ]) element.textContent = '—';
    return;
  }
  elements.metricGlobal.textContent = `${result.globalPlan.length} pts`;
  elements.metricTransformed.textContent = `${result.transformedGlobalPlan.length} pts`;
  elements.metricLocal.textContent = `${result.localPlan.length} pts`;
  elements.metricCandidates.textContent = `${result.summary.candidateCount} / ${result.summary.rejectedCandidateCount} 제외`;
  elements.metricVelocity.textContent = `${result.summary.selectedVx?.toFixed(2) ?? '—'} m/s · ${result.summary.selectedVtheta?.toFixed(2) ?? '—'} rad/s`;
  elements.metricScore.textContent = result.summary.score?.toFixed(2) ?? '경로 없음';
}

function drawMap(ctx, map, transform) {
  const { scale, left, top } = transform;
  ctx.fillStyle = '#101c22';
  ctx.fillRect(0, 0, ctx.canvas.width, ctx.canvas.height);
  for (let row = 0; row < map.height; row += 1) {
    for (let column = 0; column < map.width; column += 1) {
      const stateAtCell = map.cells[(row * map.width) + column];
      ctx.fillStyle = stateAtCell === 'occupied' ? '#071014' : stateAtCell === 'unknown' ? '#56625e' : '#1c2d32';
      ctx.fillRect(left + (column * scale), top + (row * scale), Math.ceil(scale), Math.ceil(scale));
    }
  }
  ctx.strokeStyle = '#39504d';
  ctx.lineWidth = 1;
  ctx.strokeRect(left, top, map.width * scale, map.height * scale);
}

function drawPath(ctx, transform, path, color, width, dash = []) {
  if (!path?.length) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.lineCap = 'round';
  ctx.setLineDash(dash);
  ctx.beginPath();
  path.forEach((pose, index) => {
    const point = transform.point(pose);
    if (index === 0) ctx.moveTo(point.x, point.y);
    else ctx.lineTo(point.x, point.y);
  });
  ctx.stroke();
  ctx.restore();
}

function drawHeading(ctx, point, yaw, color, length = 18) {
  const end = {
    x: point.x + (Math.cos(yaw) * length),
    y: point.y - (Math.sin(yaw) * length),
  };
  const wing = 5;
  const left = yaw + (Math.PI * 0.82);
  const right = yaw - (Math.PI * 0.82);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.beginPath();
  ctx.moveTo(point.x, point.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(end.x, end.y);
  ctx.lineTo(end.x + (Math.cos(left) * wing), end.y - (Math.sin(left) * wing));
  ctx.lineTo(end.x + (Math.cos(right) * wing), end.y - (Math.sin(right) * wing));
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawVehicle(ctx, transform, pose) {
  if (!pose) return;
  const point = transform.point(pose);
  const radius = Math.max(7, transform.scale * 0.38);
  ctx.save();
  ctx.translate(point.x, point.y);
  ctx.rotate(-pose.yaw);
  ctx.fillStyle = '#f4f0e4';
  ctx.beginPath();
  ctx.moveTo(radius, 0);
  ctx.lineTo(-radius * 0.8, radius * 0.68);
  ctx.lineTo(-radius * 0.45, 0);
  ctx.lineTo(-radius * 0.8, -radius * 0.68);
  ctx.closePath();
  ctx.fill();
  ctx.strokeStyle = '#0b161a';
  ctx.lineWidth = 1.5;
  ctx.stroke();
  ctx.restore();
}

function drawGoal(ctx, transform, pose) {
  if (!pose) return;
  const point = transform.point(pose);
  ctx.save();
  ctx.strokeStyle = '#ff7f6b';
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.arc(point.x, point.y, Math.max(6, transform.scale * 0.32), 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
  drawHeading(ctx, point, pose.yaw, '#ff7f6b', Math.max(14, transform.scale * 0.6));
}

function drawLocalWindow(ctx, transform, pose) {
  if (!pose) return;
  const point = transform.point(pose);
  ctx.save();
  ctx.strokeStyle = '#ffc85766';
  ctx.lineWidth = 1;
  ctx.setLineDash([5, 5]);
  ctx.beginPath();
  ctx.arc(point.x, point.y, (1.5 / state.map.resolution) * transform.scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.restore();
}

function drawHover(ctx, transform) {
  if (!state.hover) return;
  const { column, row } = state.hover;
  const cellLeft = transform.left + (column * transform.scale);
  const cellTop = transform.top + (row * transform.scale);
  const point = transform.point(state.hover);
  ctx.save();
  ctx.fillStyle = '#75d9c326';
  ctx.fillRect(cellLeft, cellTop, transform.scale, transform.scale);
  ctx.strokeStyle = '#75d9c3';
  ctx.lineWidth = 1;
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.moveTo(point.x - 8, point.y);
  ctx.lineTo(point.x + 8, point.y);
  ctx.moveTo(point.x, point.y - 8);
  ctx.lineTo(point.x, point.y + 8);
  ctx.stroke();
  ctx.restore();
}

function drawPickDirection(ctx) {
  if (!state.pickMode || !state.pointerStart || !state.pointerEnd) return;
  const dx = state.pointerEnd.x - state.pointerStart.canvasPoint.x;
  const dy = state.pointerEnd.y - state.pointerStart.canvasPoint.y;
  if (Math.hypot(dx, dy) < DRAG_HEADING_THRESHOLD) return;
  ctx.save();
  ctx.strokeStyle = state.pickMode === 'start' ? '#f4f0e4' : '#ff7f6b';
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(state.pointerStart.canvasPoint.x, state.pointerStart.canvasPoint.y);
  ctx.lineTo(state.pointerEnd.x, state.pointerEnd.y);
  ctx.stroke();
  ctx.restore();
}

function renderCanvas() {
  const canvas = elements.canvas;
  const bounds = canvas.getBoundingClientRect();
  const pixelRatio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.floor(bounds.width));
  const height = Math.max(1, Math.floor(bounds.height));
  canvas.width = Math.floor(width * pixelRatio);
  canvas.height = Math.floor(height * pixelRatio);
  const ctx = canvas.getContext('2d');
  ctx.setTransform(pixelRatio, 0, 0, pixelRatio, 0, 0);
  const transform = createMapTransform(state.map, width, height, state.view);

  drawMap(ctx, state.map, transform);
  drawLocalWindow(ctx, transform, state.start);
  drawPath(ctx, transform, state.result?.globalPlan, '#8cc5ff', 2.2, [4, 5]);
  drawPath(ctx, transform, state.result?.transformedGlobalPlan, '#ffc857', 3.2);
  drawPath(ctx, transform, state.result?.localPlan, '#64e1b7', 4.1);
  drawGoal(ctx, transform, state.goal);
  drawVehicle(ctx, transform, state.start);
  drawHover(ctx, transform);
  drawPickDirection(ctx);
}

function canvasPointFromEvent(event) {
  const bounds = elements.canvas.getBoundingClientRect();
  return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
}

function currentMapTransform() {
  const bounds = elements.canvas.getBoundingClientRect();
  return createMapTransform(state.map, Math.max(1, bounds.width), Math.max(1, bounds.height), state.view);
}

function updateHover(canvasPoint) {
  state.hover = canvasToMapPoint(state.map, currentMapTransform(), canvasPoint);
  if (!state.hover) {
    elements.hoverReadout.textContent = '지도 위에 커서를 올리면 좌표와 셀 상태를 표시합니다.';
    return;
  }
  const occupancy = state.hover.occupancy === 'free' ? '주행 가능' : state.hover.occupancy === 'occupied' ? '장애물' : '미확정';
  elements.hoverReadout.textContent = `x ${state.hover.x.toFixed(2)} m · y ${state.hover.y.toFixed(2)} m · cell ${state.hover.column}, ${state.hover.row} · ${occupancy}`;
}

function updateZoomReadout() {
  elements.zoomLevel.textContent = `${Math.round(state.view.zoom * 100)}%`;
}

function setPickMode(mode) {
  state.pickMode = state.pickMode === mode ? undefined : mode;
  elements.pickStart.classList.toggle('is-active', state.pickMode === 'start');
  elements.pickGoal.classList.toggle('is-active', state.pickMode === 'goal');
  elements.pickStart.setAttribute('aria-pressed', String(state.pickMode === 'start'));
  elements.pickGoal.setAttribute('aria-pressed', String(state.pickMode === 'goal'));
  elements.canvas.classList.toggle('is-picking', Boolean(state.pickMode));
  elements.pickHint.textContent = state.pickMode
    ? `${state.pickMode === 'start' ? 'Start' : 'Goal'}: 클릭은 x/y, 드래그는 x/y/yaw를 입력합니다.`
    : '선택 후 지도에서 클릭하세요. 드래그하면 방향(yaw)도 지정합니다.';
}

function clearResultForPoseEdit() {
  state.result = undefined;
  renderMetrics();
  elements.emptyState.classList.remove('is-hidden');
}

function updatePoseFromMapPoint(mode, mapPoint, vector) {
  document.querySelector(`#${mode}-x`).value = mapPoint.x.toFixed(2);
  document.querySelector(`#${mode}-y`).value = mapPoint.y.toFixed(2);
  const hasHeading = Math.hypot(vector.x, vector.y) >= DRAG_HEADING_THRESHOLD;
  if (hasHeading) {
    const yawDegrees = canvasVectorToYaw(vector) * (180 / Math.PI);
    document.querySelector(`#${mode}-yaw`).value = yawDegrees.toFixed(1);
  }
  state[mode] = readPose(mode);
  renderNav2Poses();
  clearResultForPoseEdit();
  const modeLabel = mode === 'start' ? 'Start' : 'Goal';
  setStatus(`${modeLabel} 좌표를 지도에서 입력했습니다.${hasHeading ? ' 드래그 방향으로 yaw도 설정했습니다.' : ' yaw는 기존 값을 유지합니다.'}`, 'success');
}

function resetMapInteraction() {
  state.hover = undefined;
  state.pointerStart = undefined;
  state.pointerEnd = undefined;
  state.view = { zoom: 1, offsetX: 0, offsetY: 0 };
  setPickMode();
  updateHover({ x: -1, y: -1 });
  updateZoomReadout();
}

async function loadMapFromFiles() {
  const pgm = elements.pgmFile.files[0];
  const yaml = elements.yamlFile.files[0];
  if (!pgm || !yaml) {
    setStatus('PGM과 YAML 파일을 모두 선택하세요.', 'error');
    return;
  }
  try {
    const [pgmBytes, yamlText] = await Promise.all([
      pgm.arrayBuffer().then((buffer) => new Uint8Array(buffer)),
      yaml.text(),
    ]);
    state.map = createOccupancyMap(parsePgm(pgmBytes), parseMapYaml(yamlText));
    state.mapName = `${pgm.name} · ${yaml.name}`;
    state.result = undefined;
    state.start = undefined;
    state.goal = undefined;
    resetMapInteraction();
    setMapSource(`${state.mapName} 로드됨`);
    setStatus('지도를 읽었습니다. Start 또는 Goal을 선택해 지도에서 입력할 수 있습니다.', 'success');
    renderMetrics();
    renderCanvas();
    elements.emptyState.classList.remove('is-hidden');
  } catch (error) {
    setStatus(`지도 읽기 실패: ${error.message}`, 'error');
  }
}

function useExampleMap() {
  state.map = createExampleMap();
  state.mapName = '내장 예제 지도';
  state.result = undefined;
  state.start = undefined;
  state.goal = undefined;
  resetMapInteraction();
  setMapSource('내장 예제 지도 사용 중');
  setStatus('내장 예제 지도로 바꿨습니다. Start 또는 Goal을 선택해 지도에서 입력할 수 있습니다.', 'success');
  renderMetrics();
  renderCanvas();
  elements.emptyState.classList.remove('is-hidden');
}

function runNavigation() {
  const start = readPose('start');
  const goal = readPose('goal');
  renderNav2Poses();
  const result = computeNavigation({ map: state.map, start, goal });
  state.start = start;
  state.goal = goal;
  state.result = result.ok ? result : undefined;
  renderMetrics(result);
  renderCanvas();

  if (!result.ok) {
    setStatus(`${result.code}: ${result.message}`, 'error');
    elements.emptyState.classList.remove('is-hidden');
    return;
  }
  if (!result.localPlan.length) {
    setStatus('전역 경로는 찾았지만 충돌 없는 로컬 후보가 없습니다.', 'error');
  } else {
    setStatus(`${state.mapName}에서 세 경로 레이어를 계산했습니다.`, 'success');
  }
  elements.emptyState.classList.add('is-hidden');
}

function applyZoom(factor, anchor) {
  const bounds = elements.canvas.getBoundingClientRect();
  state.view = zoomMapView(state.view, factor, anchor, Math.max(1, bounds.width), Math.max(1, bounds.height));
  updateZoomReadout();
  updateHover(anchor);
  renderCanvas();
}

function canvasCenter() {
  const bounds = elements.canvas.getBoundingClientRect();
  return { x: bounds.width / 2, y: bounds.height / 2 };
}

function onPointerDown(event) {
  if (event.button !== 0) return;
  const canvasPoint = canvasPointFromEvent(event);
  const mapPoint = canvasToMapPoint(state.map, currentMapTransform(), canvasPoint);
  if (!mapPoint) return;
  state.pointerStart = { canvasPoint, mapPoint };
  state.pointerEnd = undefined;
  elements.canvas.setPointerCapture(event.pointerId);
}

function onPointerMove(event) {
  const canvasPoint = canvasPointFromEvent(event);
  updateHover(canvasPoint);
  if (state.pointerStart && state.pickMode) state.pointerEnd = canvasPoint;
  renderCanvas();
}

function onPointerUp(event) {
  if (!state.pointerStart) return;
  const endPoint = canvasPointFromEvent(event);
  const { canvasPoint, mapPoint } = state.pointerStart;
  if (state.pickMode) {
    updatePoseFromMapPoint(state.pickMode, mapPoint, {
      x: endPoint.x - canvasPoint.x,
      y: endPoint.y - canvasPoint.y,
    });
  }
  state.pointerStart = undefined;
  state.pointerEnd = undefined;
  if (elements.canvas.hasPointerCapture(event.pointerId)) elements.canvas.releasePointerCapture(event.pointerId);
  updateHover(endPoint);
  renderCanvas();
}

function onPointerLeave() {
  if (state.pointerStart) return;
  updateHover({ x: -1, y: -1 });
  renderCanvas();
}

async function copyGoalCommand() {
  const command = elements.goalNav2Command.textContent;
  if (elements.copyGoalCommand.disabled || !command) return;

  try {
    if (navigator.clipboard?.writeText) {
      await navigator.clipboard.writeText(command);
    } else {
      const textarea = document.createElement('textarea');
      textarea.value = command;
      textarea.style.position = 'fixed';
      textarea.style.opacity = '0';
      document.body.append(textarea);
      textarea.select();
      const copied = document.execCommand('copy');
      textarea.remove();
      if (!copied) throw new Error('clipboard unavailable');
    }
    elements.copyGoalCommand.textContent = '복사됨';
    setStatus('Goal Nav2 action 명령을 클립보드에 복사했습니다.', 'success');
    window.setTimeout(() => { elements.copyGoalCommand.textContent = '명령 복사'; }, 1800);
  } catch {
    setStatus('명령 복사에 실패했습니다. 아래 명령을 직접 선택해 복사하세요.', 'error');
  }
}

elements.loadMap.addEventListener('click', loadMapFromFiles);
elements.useExampleMap.addEventListener('click', useExampleMap);
elements.runNavigation.addEventListener('click', runNavigation);
elements.pickStart.addEventListener('click', () => setPickMode('start'));
elements.pickGoal.addEventListener('click', () => setPickMode('goal'));
for (const id of ['start-x', 'start-y', 'start-yaw', 'goal-x', 'goal-y', 'goal-yaw']) {
  document.querySelector(`#${id}`).addEventListener('input', renderNav2Poses);
}
elements.copyGoalCommand.addEventListener('click', copyGoalCommand);
elements.zoomOut.addEventListener('click', () => applyZoom(1 / ZOOM_STEP, canvasCenter()));
elements.zoomIn.addEventListener('click', () => applyZoom(ZOOM_STEP, canvasCenter()));
elements.zoomFit.addEventListener('click', () => {
  state.view = { zoom: 1, offsetX: 0, offsetY: 0 };
  updateZoomReadout();
  renderCanvas();
});
elements.canvas.addEventListener('pointerdown', onPointerDown);
elements.canvas.addEventListener('pointermove', onPointerMove);
elements.canvas.addEventListener('pointerup', onPointerUp);
elements.canvas.addEventListener('pointercancel', onPointerUp);
elements.canvas.addEventListener('pointerleave', onPointerLeave);
elements.canvas.addEventListener('wheel', (event) => {
  event.preventDefault();
  applyZoom(event.deltaY < 0 ? ZOOM_STEP : 1 / ZOOM_STEP, canvasPointFromEvent(event));
}, { passive: false });
window.addEventListener('resize', renderCanvas);

setPickMode();
updateHover({ x: -1, y: -1 });
updateZoomReadout();
renderMetrics();
renderNav2Poses();
renderCanvas();
document.documentElement.dataset.nav2PathRuntime = 'ready';
