import {
  computeNavigation,
  createExampleMap,
  createOccupancyMap,
  parseMapYaml,
  parsePgm,
  worldToGrid,
} from './model.mjs';

const elements = {
  pgmFile: document.querySelector('#pgm-file'),
  yamlFile: document.querySelector('#yaml-file'),
  loadMap: document.querySelector('#load-map'),
  useExampleMap: document.querySelector('#use-example-map'),
  runNavigation: document.querySelector('#run-navigation'),
  mapSource: document.querySelector('#map-source'),
  status: document.querySelector('#status-message'),
  canvas: document.querySelector('#map-canvas'),
  emptyState: document.querySelector('#empty-state'),
  metricGlobal: document.querySelector('#metric-global'),
  metricTransformed: document.querySelector('#metric-transformed'),
  metricLocal: document.querySelector('#metric-local'),
  metricCandidates: document.querySelector('#metric-candidates'),
  metricVelocity: document.querySelector('#metric-velocity'),
  metricScore: document.querySelector('#metric-score'),
};

const state = {
  map: createExampleMap(),
  mapName: '내장 예제 지도',
  result: undefined,
  start: undefined,
  goal: undefined,
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

function makeMapTransform(map, width, height) {
  const padding = 32;
  const scale = Math.max(1, Math.min((width - (padding * 2)) / map.width, (height - (padding * 2)) / map.height));
  const mapWidth = map.width * scale;
  const mapHeight = map.height * scale;
  const left = (width - mapWidth) / 2;
  const top = (height - mapHeight) / 2;

  return {
    scale,
    left,
    top,
    point(pose) {
      const cell = worldToGrid(map, pose);
      return {
        x: left + ((cell.column + 0.5) * scale),
        y: top + ((cell.row + 0.5) * scale),
      };
    },
  };
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
  ctx.beginPath();
  ctx.moveTo(point.x - 9, point.y);
  ctx.lineTo(point.x + 9, point.y);
  ctx.moveTo(point.x, point.y - 9);
  ctx.lineTo(point.x, point.y + 9);
  ctx.stroke();
  ctx.restore();
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
  const transform = makeMapTransform(state.map, width, height);

  drawMap(ctx, state.map, transform);
  drawLocalWindow(ctx, transform, state.start);
  drawPath(ctx, transform, state.result?.globalPlan, '#8cc5ff', 2.2, [4, 5]);
  drawPath(ctx, transform, state.result?.transformedGlobalPlan, '#ffc857', 3.2);
  drawPath(ctx, transform, state.result?.localPlan, '#64e1b7', 4.1);
  drawGoal(ctx, transform, state.goal);
  drawVehicle(ctx, transform, state.start);
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
    setMapSource(`${state.mapName} 로드됨`);
    setStatus('지도를 읽었습니다. 시작·목표 좌표를 입력하세요.', 'success');
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
  setMapSource('내장 예제 지도 사용 중');
  setStatus('내장 예제 지도로 바꿨습니다. 좌표를 입력하고 실행하세요.', 'success');
  renderMetrics();
  renderCanvas();
  elements.emptyState.classList.remove('is-hidden');
}

function runNavigation() {
  const start = readPose('start');
  const goal = readPose('goal');
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

elements.loadMap.addEventListener('click', loadMapFromFiles);
elements.useExampleMap.addEventListener('click', useExampleMap);
elements.runNavigation.addEventListener('click', runNavigation);
window.addEventListener('resize', renderCanvas);

renderMetrics();
renderCanvas();
