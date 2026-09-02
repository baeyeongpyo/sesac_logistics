const DEFAULT_MANUAL_HOLD_MS = 300;
const MAX_MANUAL_HOLD_MS = 1000;
const MIN_MANUAL_SPEED = 0.1;
const MAX_MANUAL_SPEED = 1.0;
const MAX_MANUAL_ROTATION_RADIANS = (10 * Math.PI) / 180;

const MENTORPI_M1_LENGTH_M = 0.212;
const MENTORPI_M1_WIDTH_M = 0.171;

export class ManualControlSession {
  constructor() {
    this.robotId = null;
  }

  begin(robotId) {
    this.robotId = robotId;
  }

  complete() {
    const robotId = this.robotId;
    this.robotId = null;
    return robotId;
  }
}

const state = {
  snapshot: null,
  selectedRobotId: null,
  map: null,
  mapSize: null,
  mapZoom: 1,
  mapPan: { x: 0, y: 0 },
  mapPanDrag: null,
  initialPoseMode: false,
  dragStart: null,
  manualTimer: null,
  manualButton: null,
  manualSession: new ManualControlSession(),
  manualInFlight: false,
  manualSpeeds: { drive: 0.1, strafe: 0.1, rotation: 0.2 },
  toastTimer: null,
};

const select = (selector) => (typeof document === 'undefined' ? null : document.querySelector(selector));
const elements = {
  vehicleList: select('#vehicleList'),
  vehicleCount: select('#vehicleCount'),
  sourceHealth: select('#sourceHealth'),
  selectedName: select('#selectedName'),
  selectedState: select('#selectedState'),
  vehicleDetails: select('#vehicleDetails'),
  taskCard: select('#taskCard'),
  operationCount: select('#operationCount'),
  inventoryGrid: select('#inventoryGrid'),
  pgmCanvas: select('#pgmCanvas'),
  mapOverlay: select('#mapOverlay'),
  mapStage: select('#mapStage'),
  mapContent: select('#mapContent'),
  mapZoomControls: select('#mapZoomControls'),
  zoomOutButton: select('#zoomOutButton'),
  zoomResetButton: select('#zoomResetButton'),
  zoomInButton: select('#zoomInButton'),
  mapMeta: select('#mapMeta'),
  mapInstruction: select('#mapInstruction'),
  initialPoseButton: select('#initialPoseButton'),
  stopButton: select('#stopButton'),
  padStopButton: select('#padStopButton'),
  idleButton: select('#idleButton'),
  cancelButton: select('#cancelButton'),
  driveSpeedInput: select('#driveSpeedInput'),
  strafeSpeedInput: select('#strafeSpeedInput'),
  rotationSpeedInput: select('#rotationSpeedInput'),
  speedChip: select('#speedChip'),
  refreshButton: select('#refreshButton'),
  updatedAt: select('#updatedAt'),
  toast: select('#toast'),
};

export function mapToPixel(x, y, map) {
  return {
    x: rounded((x - map.origin[0]) / map.resolution),
    y: rounded(map.height - (y - map.origin[1]) / map.resolution),
  };
}

export function pixelToMap(x, y, map) {
  return {
    x: rounded(map.origin[0] + x * map.resolution),
    y: rounded(map.origin[1] + (map.height - y) * map.resolution),
  };
}

export function yawFromDrag(start, end) {
  return Math.atan2(start.y - end.y, end.x - start.x);
}

export function batteryPercent(rawValue) {
  if (!Number.isFinite(rawValue)) return null;
  const fraction = (rawValue - 6500) / (8500 - 6500);
  return Math.round(Math.max(0, Math.min(1, fraction)) * 100);
}

export function svgRotationFromYaw(yawRadians) {
  return Number(((-yawRadians * 180) / Math.PI).toFixed(3));
}

export function clampZoom(zoom) {
  return Number(Math.max(0.5, Math.min(3, zoom)).toFixed(2));
}

export function clampMapPan(pan, map, zoom) {
  const maxX = Math.max(0, (map.width * (zoom - 1)) / 2);
  const maxY = Math.max(0, (map.height * (zoom - 1)) / 2);
  return {
    x: rounded(Math.max(-maxX, Math.min(maxX, pan.x))),
    y: rounded(Math.max(-maxY, Math.min(maxY, pan.y))),
  };
}

export function rotationHoldMs(angularSpeed) {
  const duration = Math.floor((MAX_MANUAL_ROTATION_RADIANS / boundedManualSpeed(angularSpeed)) * 1000);
  return Math.max(100, Math.min(MAX_MANUAL_HOLD_MS, duration));
}

export function manualCommand(commandName, speeds) {
  const drive = boundedManualSpeed(speeds.drive);
  const strafe = boundedManualSpeed(speeds.strafe);
  const rotation = boundedManualSpeed(speeds.rotation);
  if (commandName === 'forward') return { linear_x: drive, linear_y: 0, angular_z: 0, hold_ms: DEFAULT_MANUAL_HOLD_MS };
  if (commandName === 'reverse') return { linear_x: -drive, linear_y: 0, angular_z: 0, hold_ms: DEFAULT_MANUAL_HOLD_MS };
  if (commandName === 'strafe-left') return { linear_x: 0, linear_y: strafe, angular_z: 0, hold_ms: DEFAULT_MANUAL_HOLD_MS };
  if (commandName === 'strafe-right') return { linear_x: 0, linear_y: -strafe, angular_z: 0, hold_ms: DEFAULT_MANUAL_HOLD_MS };
  if (commandName === 'rotate-left') return { linear_x: 0, linear_y: 0, angular_z: rotation, hold_ms: rotationHoldMs(rotation) };
  if (commandName === 'rotate-right') return { linear_x: 0, linear_y: 0, angular_z: -rotation, hold_ms: rotationHoldMs(rotation) };
  throw new Error('지원하지 않는 수동 조작입니다.');
}

export function manualRepeatInterval(command) {
  const holdMs = Number(command?.hold_ms);
  if (!Number.isFinite(holdMs) || holdMs <= 0) return 220;
  return Math.max(50, Math.min(220, Math.floor(holdMs * 0.7)));
}

function boundedManualSpeed(value) {
  if (!Number.isFinite(value)) return MIN_MANUAL_SPEED;
  return Math.max(MIN_MANUAL_SPEED, Math.min(MAX_MANUAL_SPEED, value));
}

export function vehicleSizeInPixels(map) {
  return {
    length: rounded(MENTORPI_M1_LENGTH_M / map.resolution),
    width: rounded(MENTORPI_M1_WIDTH_M / map.resolution),
  };
}

function rounded(value) {
  return Number(value.toFixed(6));
}

async function loadMap() {
  const [configResponse, pgmResponse] = await Promise.all([
    fetch('/api/map/config'),
    fetch('/api/map/pgm'),
  ]);
  if (!configResponse.ok || !pgmResponse.ok) {
    throw new Error('PGM 지도 파일을 불러올 수 없습니다.');
  }
  const [config, buffer] = await Promise.all([configResponse.json(), pgmResponse.arrayBuffer()]);
  const pgm = parsePgm(buffer);
  state.map = { ...config, width: pgm.width, height: pgm.height };
  state.mapSize = { width: pgm.width, height: pgm.height };
  state.mapPan = { x: 0, y: 0 };
  renderPgm(pgm);
  elements.mapOverlay.setAttribute('viewBox', `0 0 ${pgm.width} ${pgm.height}`);
  elements.mapMeta.textContent = `${pgm.width} × ${pgm.height} · ${config.resolution}m/px`;
  renderMapTransform();
  renderOverlay();
}

export function parsePgm(buffer) {
  const bytes = new Uint8Array(buffer);
  let cursor = 0;
  const whitespace = (value) => value === 9 || value === 10 || value === 13 || value === 32;
  const token = () => {
    while (cursor < bytes.length) {
      if (bytes[cursor] === 35) {
        while (cursor < bytes.length && bytes[cursor] !== 10) cursor += 1;
      } else if (whitespace(bytes[cursor])) {
        cursor += 1;
      } else break;
    }
    const start = cursor;
    while (cursor < bytes.length && !whitespace(bytes[cursor])) cursor += 1;
    return new TextDecoder().decode(bytes.slice(start, cursor));
  };
  const magic = token();
  const width = Number(token());
  const height = Number(token());
  const max = Number(token());
  if (!['P2', 'P5'].includes(magic) || !width || !height || !max) {
    throw new Error('지원하지 않는 PGM 형식입니다.');
  }
  if (magic === 'P2') {
    const pixels = new Uint16Array(width * height);
    for (let index = 0; index < pixels.length; index += 1) pixels[index] = Number(token());
    return { width, height, max, pixels };
  }
  if (bytes[cursor] === 13 && bytes[cursor + 1] === 10) cursor += 2;
  else if (whitespace(bytes[cursor])) cursor += 1;
  const count = width * height;
  const pixels = new Uint16Array(count);
  if (max < 256) {
    if (bytes.length - cursor < count) throw new Error('PGM 픽셀 데이터가 부족합니다.');
    for (let index = 0; index < count; index += 1) pixels[index] = bytes[cursor + index];
  } else {
    if (bytes.length - cursor < count * 2) throw new Error('PGM 픽셀 데이터가 부족합니다.');
    for (let index = 0; index < count; index += 1) pixels[index] = (bytes[cursor + index * 2] << 8) | bytes[cursor + index * 2 + 1];
  }
  return { width, height, max, pixels };
}

function renderPgm(pgm) {
  const canvas = elements.pgmCanvas;
  canvas.width = pgm.width;
  canvas.height = pgm.height;
  const context = canvas.getContext('2d');
  const image = context.createImageData(pgm.width, pgm.height);
  for (let index = 0; index < pgm.pixels.length; index += 1) {
    const gray = Math.round((pgm.pixels[index] / pgm.max) * 255);
    const channel = index * 4;
    image.data[channel] = gray;
    image.data[channel + 1] = gray;
    image.data[channel + 2] = gray;
    image.data[channel + 3] = 255;
  }
  context.putImageData(image, 0, 0);
}

async function refreshSnapshot() {
  try {
    const response = await fetch('/api/snapshot', { cache: 'no-store' });
    if (!response.ok) throw new Error('운영 데이터를 불러올 수 없습니다.');
    state.snapshot = await response.json();
    const selectedStillExists = state.snapshot.vehicles.some(
      (vehicle) => vehicle.robot_id === state.selectedRobotId,
    );
    if (!selectedStillExists) state.selectedRobotId = state.snapshot.vehicles[0]?.robot_id ?? null;
    renderDashboard();
  } catch (error) {
    showToast(error.message, true);
  }
}

function renderDashboard() {
  const snapshot = state.snapshot;
  if (!snapshot) return;
  elements.vehicleCount.textContent = snapshot.vehicles.length;
  elements.updatedAt.textContent = `갱신 ${formatTime(snapshot.generated_at)}`;
  renderVehicleList(snapshot.vehicles);
  renderSourceHealth(snapshot.sources);
  renderSelectedVehicle(selectedVehicle());
  renderInventory(snapshot.inventory);
  renderOverlay();
}

function renderVehicleList(vehicles) {
  if (!vehicles.length) {
    elements.vehicleList.innerHTML = '<p class="empty-state">관측된 차량이 없습니다.</p>';
    return;
  }
  elements.vehicleList.innerHTML = vehicles.map((vehicle) => {
    const selected = vehicle.robot_id === state.selectedRobotId ? 'selected' : '';
    const visualState = stateClass(vehicle.fleet_state?.state);
    const task = vehicle.active_task?.status || vehicle.active_task?.operation_id || '대기 중';
    const battery = batteryLabel(vehicle.battery?.battery_raw);
    return `<button class="vehicle-item ${selected}" data-robot-id="${escapeHtml(vehicle.robot_id)}" type="button">
      <span class="vehicle-status-dot ${visualState}"></span>
      <span><span class="vehicle-id">${escapeHtml(vehicle.robot_id)}</span><span class="vehicle-task">${escapeHtml(task)}</span></span>
      <span class="battery">${battery}</span>
    </button>`;
  }).join('');
  elements.vehicleList.querySelectorAll('[data-robot-id]').forEach((button) => {
    button.addEventListener('click', () => {
      state.selectedRobotId = button.dataset.robotId;
      renderDashboard();
    });
  });
}

function renderSourceHealth(sources) {
  elements.sourceHealth.innerHTML = Object.values(sources).map((source) => {
    const className = source.available ? 'healthy' : 'unhealthy';
    const suffix = source.available ? '정상' : '연결 불가';
    return `<span class="${className}">${escapeHtml(source.name)} ${suffix}</span>`;
  }).join('<br>');
}

function renderSelectedVehicle(vehicle) {
  const enabled = Boolean(vehicle);
  elements.initialPoseButton.disabled = !enabled;
  elements.stopButton.disabled = !enabled;
  elements.padStopButton.disabled = !enabled;
  elements.idleButton.disabled = !enabled;
  elements.cancelButton.disabled = !enabled;
  document.querySelectorAll('[data-manual]').forEach((button) => {
    button.disabled = !enabled || isActiveOperation(vehicle?.fleet_state?.state);
  });
  if (!vehicle) {
    elements.selectedName.textContent = '차량을 선택하세요';
    elements.selectedState.textContent = '—';
    elements.selectedState.className = 'state-badge state-unknown';
    elements.vehicleDetails.innerHTML = '<div><dt>위치</dt><dd>—</dd></div><div><dt>자세</dt><dd>—</dd></div><div><dt>배터리</dt><dd>—</dd></div><div><dt>적재</dt><dd>—</dd></div>';
    elements.taskCard.innerHTML = '<p class="eyebrow">CURRENT TASK</p><strong>선택된 차량이 없습니다.</strong><span>—</span>';
    return;
  }
  const pose = vehicle.pose;
  const yawDegrees = pose ? ((pose.yaw_rad * 180) / Math.PI).toFixed(1) : '—';
  const fleetState = vehicle.fleet_state?.state || 'UNKNOWN';
  elements.selectedName.textContent = vehicle.robot_id;
  elements.selectedState.textContent = fleetState;
  elements.selectedState.className = `state-badge state-${stateClass(fleetState)}`;
  elements.vehicleDetails.innerHTML = `
    <div><dt>위치</dt><dd>${pose ? `${pose.x_m.toFixed(2)}, ${pose.y_m.toFixed(2)} m` : '—'}</dd></div>
    <div><dt>자세</dt><dd>${yawDegrees === '—' ? '—' : `${yawDegrees}°`}</dd></div>
    <div><dt>배터리</dt><dd>${batteryDetail(vehicle.battery?.battery_raw)}</dd></div>
    <div><dt>적재</dt><dd>${vehicle.pallet_state?.has_pallet ? vehicle.pallet_state.payload_type : '비적재'}</dd></div>`;
  const task = vehicle.active_task;
  elements.taskCard.innerHTML = task
    ? `<p class="eyebrow">CURRENT TASK</p><strong>${escapeHtml(task.status || task.operation_id)}</strong><span>${escapeHtml(task.source_zone_id || '작업 상세 수신 대기')} → ${escapeHtml(task.destination_zone_id || '')}</span>`
    : '<p class="eyebrow">CURRENT TASK</p><strong>할당된 작업 없음</strong><span>대기 중</span>';
}

function renderInventory(inventory) {
  const operations = inventory?.transport_operations || [];
  elements.operationCount.textContent = `작업 ${operations.length}건`;
  const zones = inventory?.zone_inventory || [];
  if (!zones.length) {
    elements.inventoryGrid.innerHTML = '<p class="empty-state">재고 데이터가 없습니다.</p>';
    return;
  }
  elements.inventoryGrid.innerHTML = zones.map((zone) => `<article class="inventory-card">
    <strong>${escapeHtml(zone.zone_id)}</strong><span>${escapeHtml(inventoryBreakdown(zone.items))}</span>
    <b>${zone.current_quantity}<small> / ${zone.capacity ?? '—'}</small></b>
  </article>`).join('');
}

export function inventoryBreakdown(items) {
  if (!items?.length) return '재고 없음';
  return items.map((item) => `${item.payload_type} ${item.quantity}`).join(' · ');
}

function renderOverlay() {
  if (!state.map || !state.snapshot) return;
  const markers = state.snapshot.vehicles.filter((vehicle) => vehicle.pose).map((vehicle) => {
    const point = mapToPixel(vehicle.pose.x_m, vehicle.pose.y_m, state.map);
    const selected = vehicle.robot_id === state.selectedRobotId ? 'selected' : '';
    const rotation = svgRotationFromYaw(vehicle.pose.yaw_rad);
    const size = vehicleSizeInPixels(state.map);
    const halfLength = size.length / 2;
    const halfWidth = size.width / 2;
    const noseBase = rounded(size.length * 0.18);
    const noseTip = rounded(size.length * 0.4);
    const noseHalfWidth = rounded(size.width * 0.26);
    return `<g class="vehicle-marker ${selected}" transform="translate(${point.x} ${point.y}) rotate(${rotation})">
      <rect class="marker-body" x="${-halfLength}" y="${-halfWidth}" width="${size.length}" height="${size.width}" rx="0.3"></rect>
      <path class="marker-nose" d="M ${noseBase} -${noseHalfWidth} L ${noseTip} 0 L ${noseBase} ${noseHalfWidth} Z"></path>
    </g><text class="vehicle-label" x="${point.x}" y="${point.y - halfWidth - 2}">${escapeHtml(shortRobotId(vehicle.robot_id))}</text>`;
  }).join('');
  const preview = state.dragStart && state.dragEnd ? `<g class="pose-preview"><line x1="${state.dragStart.x}" y1="${state.dragStart.y}" x2="${state.dragEnd.x}" y2="${state.dragEnd.y}"></line><circle cx="${state.dragStart.x}" cy="${state.dragStart.y}" r="3"></circle></g>` : '';
  elements.mapOverlay.innerHTML = markers + preview;
}

function selectedVehicle() {
  return state.snapshot?.vehicles.find((vehicle) => vehicle.robot_id === state.selectedRobotId) || null;
}

function isActiveOperation(fleetState) {
  return ['DRIVE', 'PICK', 'PLACE'].includes(fleetState);
}

function stateClass(fleetState) {
  if (fleetState === 'IDLE' || fleetState === 'WAIT' || fleetState === 'INIT') return 'idle';
  if (fleetState === 'FAIL' || fleetState === 'FAILED') return 'fail';
  return fleetState ? 'active' : 'unknown';
}

function shortRobotId(robotId) {
  return robotId.replace(/^robot[_-]?/i, '').slice(-3) || robotId.slice(0, 3);
}

function batteryLabel(rawValue) {
  const percent = batteryPercent(rawValue);
  return percent === null ? '—' : `${percent}%`;
}

function batteryDetail(rawValue) {
  const percent = batteryPercent(rawValue);
  return percent === null ? '—' : `${percent}% · raw ${rawValue}`;
}

function escapeHtml(value) {
  return String(value ?? '').replace(/[&<>'"]/g, (character) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' })[character]);
}

function formatTime(value) {
  const time = new Date(value);
  return Number.isNaN(time.valueOf()) ? '—' : time.toLocaleTimeString('ko-KR', { hour: '2-digit', minute: '2-digit', second: '2-digit' });
}

async function postControl(path, body) {
  const response = await fetch(path, {
    method: 'POST',
    headers: body ? { 'Content-Type': 'application/json' } : {},
    body: body ? JSON.stringify(body) : undefined,
  });
  const result = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(result.detail || '차량 제어 요청이 거부되었습니다.');
  return result;
}

function startManual(commandName, button) {
  const vehicle = selectedVehicle();
  if (!vehicle || isActiveOperation(vehicle.fleet_state?.state)) return;
  finishManual(false);
  const command = manualCommand(commandName, state.manualSpeeds);
  const send = async () => {
    if (state.manualInFlight) return;
    state.manualInFlight = true;
    try {
      await postControl(`/api/vehicles/${encodeURIComponent(vehicle.robot_id)}/manual`, command);
    } catch (error) {
      finishManual(true);
      showToast(error.message, true);
    } finally {
      state.manualInFlight = false;
    }
  };
  state.manualSession.begin(vehicle.robot_id);
  state.manualButton = button;
  button.classList.add('pressed');
  send();
  state.manualTimer = window.setInterval(send, manualRepeatInterval(command));
}

function sendVehicleStop(robotId) {
  postControl(`/api/vehicles/${encodeURIComponent(robotId)}/stop`)
    .catch((error) => showToast(error.message, true));
}

function finishManual(sendStop = true) {
  const manualRobotId = state.manualSession.complete();
  if (state.manualTimer) window.clearInterval(state.manualTimer);
  state.manualTimer = null;
  state.manualButton?.classList.remove('pressed');
  state.manualButton = null;
  if (sendStop && manualRobotId) sendVehicleStop(manualRobotId);
  return manualRobotId;
}

function emergencyStop() {
  const manualRobotId = finishManual(true);
  if (state.selectedRobotId && state.selectedRobotId !== manualRobotId) {
    sendVehicleStop(state.selectedRobotId);
  }
}

function setInitialPoseMode(enabled) {
  state.initialPoseMode = enabled;
  state.dragStart = null;
  state.dragEnd = null;
  elements.mapStage.classList.toggle('initial-mode', enabled);
  elements.mapInstruction.classList.toggle('initial-mode', enabled);
  elements.mapInstruction.textContent = enabled
    ? '지도에서 초기 위치를 누른 뒤 원하는 방향으로 드래그하세요.'
    : '차량을 선택하면 제어할 수 있습니다.';
  elements.initialPoseButton.textContent = enabled ? 'Initial Pose 취소' : 'Initial Pose';
  renderOverlay();
}

function mapPointer(event) {
  return mapPointAtPanZoom(mapViewportPoint(event), state.map, state.mapZoom, state.mapPan);
}

function mapViewportPoint(event) {
  const rect = elements.mapStage.getBoundingClientRect();
  return {
    x: ((event.clientX - rect.left) / rect.width) * state.map.width,
    y: ((event.clientY - rect.top) / rect.height) * state.map.height,
  };
}

export function mapPointAtZoom(point, map, zoom) {
  return mapPointAtPanZoom(point, map, zoom, { x: 0, y: 0 });
}

export function mapPointAtPanZoom(point, map, zoom, pan) {
  return {
    x: rounded(Math.max(0, Math.min(map.width, ((point.x - pan.x - map.width / 2) / zoom) + map.width / 2))),
    y: rounded(Math.max(0, Math.min(map.height, ((point.y - pan.y - map.height / 2) / zoom) + map.height / 2))),
  };
}

function setMapZoom(zoom) {
  state.mapZoom = clampZoom(zoom);
  renderMapTransform();
}

function renderMapTransform() {
  if (!state.map) return;
  state.mapPan = clampMapPan(state.mapPan, state.map, state.mapZoom);
  const panXPercent = rounded((state.mapPan.x / state.map.width) * 100);
  const panYPercent = rounded((state.mapPan.y / state.map.height) * 100);
  elements.mapContent.style.transform = `translate(${panXPercent}%, ${panYPercent}%) scale(${state.mapZoom})`;
  elements.zoomResetButton.textContent = `${Math.round(state.mapZoom * 100)}%`;
}

function resetMapView() {
  state.mapZoom = 1;
  state.mapPan = { x: 0, y: 0 };
  renderMapTransform();
}

function panMapBy(delta) {
  if (!state.map) return;
  state.mapPan = clampMapPan(
    { x: state.mapPan.x + delta.x, y: state.mapPan.y + delta.y },
    state.map,
    state.mapZoom,
  );
  renderMapTransform();
}

function clearMapPanDrag() {
  state.mapPanDrag = null;
  elements.mapStage.classList.remove('panning');
}

function updateSpeedChip() {
  elements.speedChip.textContent = `전후 ${state.manualSpeeds.drive} · 횡 ${state.manualSpeeds.strafe} m/s · 회전 ${state.manualSpeeds.rotation} rad/s`;
}

function installManualSpeedInputs() {
  const inputs = [
    ['drive', elements.driveSpeedInput],
    ['strafe', elements.strafeSpeedInput],
    ['rotation', elements.rotationSpeedInput],
  ];
  inputs.forEach(([key, input]) => {
    input.addEventListener('input', () => {
      const value = Number(input.value);
      if (!Number.isFinite(value)) return;
      state.manualSpeeds[key] = boundedManualSpeed(value);
      updateSpeedChip();
    });
    input.addEventListener('blur', () => {
      input.value = String(state.manualSpeeds[key]);
    });
  });
  updateSpeedChip();
}

function installInteractions() {
  installManualSpeedInputs();
  elements.refreshButton.addEventListener('click', refreshSnapshot);
  elements.initialPoseButton.addEventListener('click', () => {
    if (selectedVehicle()) setInitialPoseMode(!state.initialPoseMode);
  });
  elements.stopButton.addEventListener('click', emergencyStop);
  elements.padStopButton.addEventListener('click', emergencyStop);
  elements.idleButton.addEventListener('click', async () => {
    const vehicle = selectedVehicle(); if (!vehicle) return;
    try { await postControl(`/api/vehicles/${encodeURIComponent(vehicle.robot_id)}/operation/idle`); showToast('Operation / Idle 요청을 전달했습니다.'); refreshSnapshot(); } catch (error) { showToast(error.message, true); }
  });
  elements.cancelButton.addEventListener('click', async () => {
    const vehicle = selectedVehicle(); if (!vehicle) return;
    try { await postControl(`/api/vehicles/${encodeURIComponent(vehicle.robot_id)}/navigation/cancel`); showToast('주행 취소 요청을 전달했습니다.'); refreshSnapshot(); } catch (error) { showToast(error.message, true); }
  });
  elements.zoomInButton.addEventListener('click', () => setMapZoom(state.mapZoom + 0.25));
  elements.zoomOutButton.addEventListener('click', () => setMapZoom(state.mapZoom - 0.25));
  elements.zoomResetButton.addEventListener('click', resetMapView);
  elements.mapZoomControls.addEventListener('pointerdown', (event) => event.stopPropagation());
  elements.mapStage.addEventListener('wheel', (event) => {
    if (!state.map) return;
    event.preventDefault();
    if (event.ctrlKey || event.metaKey) {
      setMapZoom(state.mapZoom + (event.deltaY < 0 ? 0.2 : -0.2));
      return;
    }
    const rect = elements.mapStage.getBoundingClientRect();
    const horizontalDelta = event.deltaX + (event.shiftKey ? event.deltaY : 0);
    const verticalDelta = event.shiftKey ? 0 : event.deltaY;
    panMapBy({
      x: (-horizontalDelta / rect.width) * state.map.width,
      y: (-verticalDelta / rect.height) * state.map.height,
    });
  }, { passive: false });
  document.querySelectorAll('[data-manual]').forEach((button) => {
    const commandName = button.dataset.manual;
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      button.setPointerCapture?.(event.pointerId);
      startManual(commandName, button);
    });
    ['pointerup', 'pointercancel', 'lostpointercapture'].forEach((eventName) => button.addEventListener(eventName, () => finishManual(true)));
  });
  window.addEventListener('blur', () => finishManual(true));
  elements.mapStage.addEventListener('pointerdown', (event) => {
    if (state.initialPoseMode && selectedVehicle()) {
      state.dragStart = mapPointer(event); state.dragEnd = state.dragStart;
      elements.mapStage.setPointerCapture?.(event.pointerId); renderOverlay();
      return;
    }
    if (state.initialPoseMode) return;
    state.mapPanDrag = {
      pointerId: event.pointerId,
      start: mapViewportPoint(event),
      pan: { ...state.mapPan },
    };
    elements.mapStage.setPointerCapture?.(event.pointerId);
    elements.mapStage.classList.add('panning');
  });
  elements.mapStage.addEventListener('pointermove', (event) => {
    if (state.mapPanDrag?.pointerId === event.pointerId) {
      const point = mapViewportPoint(event);
      const { start, pan } = state.mapPanDrag;
      state.mapPan = clampMapPan(
        { x: pan.x + point.x - start.x, y: pan.y + point.y - start.y },
        state.map,
        state.mapZoom,
      );
      renderMapTransform();
      return;
    }
    if (!state.dragStart) return;
    state.dragEnd = mapPointer(event); renderOverlay();
  });
  elements.mapStage.addEventListener('pointerup', async (event) => {
    if (state.mapPanDrag?.pointerId === event.pointerId) {
      clearMapPanDrag();
      return;
    }
    if (!state.dragStart || !state.dragEnd) return;
    const vehicle = selectedVehicle();
    const start = state.dragStart; const end = mapPointer(event);
    const distance = Math.hypot(end.x - start.x, end.y - start.y);
    state.dragStart = null; state.dragEnd = null; renderOverlay();
    if (distance < 3 || !vehicle) { showToast('방향을 지정할 수 있도록 조금 더 길게 드래그하세요.', true); return; }
    const point = pixelToMap(start.x, start.y, state.map);
    const yaw = yawFromDrag(start, end);
    try {
      await postControl(`/api/vehicles/${encodeURIComponent(vehicle.robot_id)}/initial-pose`, { x: point.x, y: point.y, yaw });
      setInitialPoseMode(false);
      showToast(`${vehicle.robot_id} Initial Pose 요청을 전달했습니다.`);
      refreshSnapshot();
    } catch (error) { showToast(error.message, true); }
  });
  ['pointercancel', 'lostpointercapture'].forEach((eventName) => {
    elements.mapStage.addEventListener(eventName, () => {
      clearMapPanDrag();
      if (state.dragStart) {
        state.dragStart = null;
        state.dragEnd = null;
        renderOverlay();
      }
    });
  });
}

function showToast(message, isError = false) {
  window.clearTimeout(state.toastTimer);
  elements.toast.textContent = message;
  elements.toast.classList.toggle('error', isError);
  elements.toast.classList.add('visible');
  state.toastTimer = window.setTimeout(() => elements.toast.classList.remove('visible'), 3600);
}

async function bootstrap() {
  installInteractions();
  try { await loadMap(); } catch (error) { showToast(error.message, true); }
  await refreshSnapshot();
  window.setInterval(refreshSnapshot, 4000);
}

if (typeof document !== 'undefined') bootstrap();
