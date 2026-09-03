const DEFAULT_MANUAL_HOLD_MS = 300;
const MAX_MANUAL_HOLD_MS = 1000;
const MIN_MANUAL_SPEED = 0.1;
const MAX_MANUAL_SPEED = 1.0;
const MAX_MANUAL_ROTATION_RADIANS = (10 * Math.PI) / 180;

const MENTORPI_M1_LENGTH_M = 0.212;
const MENTORPI_M1_WIDTH_M = 0.171;
const MAP_UNKNOWN_PIXEL = 205;
const MAP_VIEW_PADDING_PX = 12;

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

export class NavigationDraft {
  constructor() {
    this.mode = null;
    this.initialPose = null;
    this.goalPose = null;
    this.waypoints = [];
  }

  selectMode(mode) {
    const changed = Boolean(this.mode && mode && this.mode !== mode);
    if (changed) this.clearAll();
    this.mode = mode;
    return changed;
  }

  setPose(mode, pose) {
    const normalizedPose = normalizeNavigationPose(pose);
    if (mode === 'initial') this.initialPose = normalizedPose;
    else if (mode === 'goal') this.goalPose = normalizedPose;
    else throw new Error('지원하지 않는 위치 지정 모드입니다.');
  }

  addWaypoint(pose) {
    this.waypoints.push(normalizeNavigationPose(pose));
  }

  removeWaypoint(index) {
    if (Number.isInteger(index) && index >= 0 && index < this.waypoints.length) {
      this.waypoints.splice(index, 1);
    }
  }

  clearPose(mode) {
    if (mode === 'initial') this.initialPose = null;
    else if (mode === 'goal') this.goalPose = null;
    else throw new Error('지원하지 않는 위치 지정 모드입니다.');
  }

  clearWaypoints() {
    this.waypoints = [];
  }

  clearAll() {
    this.initialPose = null;
    this.goalPose = null;
    this.waypoints = [];
  }

  hasPending(mode) {
    if (mode === 'initial') return this.initialPose !== null;
    if (mode === 'goal') return this.goalPose !== null;
    if (mode === 'waypoints') return this.waypoints.length > 0;
    return false;
  }
}

export class NavigationExecutionSession {
  constructor() {
    this.execution = null;
  }

  begin(robotId, mode, draftRevision) {
    if (this.execution) return null;
    this.execution = { robotId, mode, draftRevision };
    return this.execution;
  }

  complete(execution) {
    if (this.execution !== execution) return false;
    this.execution = null;
    return true;
  }

  isPending() {
    return this.execution !== null;
  }
}

export function navigationExecutionMatchesDraft(execution, draftRobotId, draftRevisions) {
  return execution?.robotId === draftRobotId
    && execution.draftRevision === draftRevisions?.[execution.mode];
}

function normalizeNavigationPose(pose) {
  if (!pose || ![pose.x, pose.y, pose.yaw].every(Number.isFinite)) {
    throw new Error('유효하지 않은 지도 위치입니다.');
  }
  return { x: pose.x, y: pose.y, yaw: pose.yaw };
}

const state = {
  snapshot: null,
  selectedRobotId: null,
  map: null,
  mapSize: null,
  mapView: null,
  mapZoom: 1,
  mapPan: { x: 0, y: 0 },
  mapPanDrag: null,
  navigationMode: null,
  navigationDraft: new NavigationDraft(),
  navigationDraftRobotId: null,
  navigationDraftRevisions: { initial: 0, goal: 0, waypoints: 0 },
  navigationExecution: new NavigationExecutionSession(),
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
  mapViewport: select('#mapViewport'),
  mapContent: select('#mapContent'),
  mapZoomControls: select('#mapZoomControls'),
  zoomOutButton: select('#zoomOutButton'),
  zoomResetButton: select('#zoomResetButton'),
  zoomInButton: select('#zoomInButton'),
  mapMeta: select('#mapMeta'),
  mapInstruction: select('#mapInstruction'),
  initialPoseButton: select('#initialPoseButton'),
  goalPoseButton: select('#goalPoseButton'),
  waypointButton: select('#waypointButton'),
  navigationDraft: select('#navigationDraft'),
  navigationDraftTitle: select('#navigationDraftTitle'),
  navigationDraftMeta: select('#navigationDraftMeta'),
  waypointList: select('#waypointList'),
  clearNavigationButton: select('#clearNavigationButton'),
  executeNavigationButton: select('#executeNavigationButton'),
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

export function connectivityLabel(connectivityState) {
  if (connectivityState === 'online') return '연결됨';
  if (connectivityState === 'stale') return '응답 지연';
  if (connectivityState === 'offline') return '연결 끊김';
  return '수신 이력 없음';
}

export function connectivityAgeLabel(connectivity) {
  if (!['online', 'stale'].includes(connectivity?.state) || !Number.isFinite(connectivity?.age_sec)) {
    return '';
  }
  return ` · ${connectivity.age_sec}초 전`;
}

export function scheduleSnapshotRefresh(refresh, schedule = globalThis.setInterval) {
  return schedule(refresh, 1000);
}

export function fleetStateLabel(fleetState) {
  if (fleetState === 'WAIT' || fleetState === 'IDLE') return '대기 중';
  if (fleetState === 'INIT') return '초기화 중';
  if (fleetState === 'DRIVE') return '주행 중';
  if (fleetState === 'AUTO_DRIVE') return '자동 주행 중';
  if (fleetState === 'MANUAL_DRIVE') return '수동 주행 중';
  if (fleetState === 'PICK') return '픽업 중';
  if (fleetState === 'PLACE') return '적치 중';
  if (fleetState === 'STOPPED') return '정지';
  if (fleetState === 'FAIL' || fleetState === 'FAILED') return '오류';
  return '상태 수신 대기';
}

export function displayedFleetState(vehicle) {
  return vehicle?.display_state || vehicle?.fleet_state?.state;
}

export function taskLabel(task, fleetState) {
  if (fleetState === 'STOPPED') return '정지 요청됨';
  if (fleetState === 'FAIL' || fleetState === 'FAILED') return '운행 복구 대기';
  if (task?.status === 'TO_PICK') return '픽업 구역으로 이동';
  if (task?.status === 'PICKING') return '파렛트 픽업 중';
  if (task?.status === 'TO_PLACE') return '적치 구역으로 이동';
  if (task?.status === 'PLACING') return '파렛트 적치 중';
  if (task?.status === 'RECOVERY_REQUIRED') return '운행 복구 대기';
  if (fleetState === 'DRIVE' || fleetState === 'PICK' || fleetState === 'PLACE') return '작업 정보 수신 대기';
  return '할당된 작업 없음';
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
  state.mapView = operationalMapView(pgm);
  state.mapSize = { width: state.mapView.width, height: state.mapView.height };
  state.mapPan = { x: 0, y: 0 };
  renderPgm(pgm, state.mapView);
  elements.mapOverlay.setAttribute('viewBox', `0 0 ${state.mapView.width} ${state.mapView.height}`);
  elements.mapViewport.style.setProperty('--map-aspect', `${state.mapView.width} / ${state.mapView.height}`);
  const displayWidthM = state.mapView.width * config.resolution;
  const displayHeightM = state.mapView.height * config.resolution;
  elements.mapMeta.textContent = `${displayWidthM.toFixed(1)} × ${displayHeightM.toFixed(1)}m · 가로 보기`;
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

function renderPgm(pgm, view) {
  const canvas = elements.pgmCanvas;
  canvas.width = view.width;
  canvas.height = view.height;
  const context = canvas.getContext('2d');
  const image = context.createImageData(view.width, view.height);
  const { sourceBounds } = view;
  for (let sourceY = sourceBounds.y; sourceY < sourceBounds.y + sourceBounds.height; sourceY += 1) {
    for (let sourceX = sourceBounds.x; sourceX < sourceBounds.x + sourceBounds.width; sourceX += 1) {
      const sourceIndex = (sourceY * pgm.width) + sourceX;
      const targetX = sourceBounds.height - 1 - (sourceY - sourceBounds.y);
      const targetY = sourceX - sourceBounds.x;
      const channel = ((targetY * view.width) + targetX) * 4;
      const gray = Math.round((pgm.pixels[sourceIndex] / pgm.max) * 255);
      image.data[channel] = gray;
      image.data[channel + 1] = gray;
      image.data[channel + 2] = gray;
      image.data[channel + 3] = 255;
    }
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
    if (!selectedStillExists) {
      state.selectedRobotId = state.snapshot.vehicles[0]?.robot_id ?? null;
      resetNavigationDraft();
    }
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
  renderNavigationDraft();
  renderOverlay();
}

function renderVehicleList(vehicles) {
  if (!vehicles.length) {
    elements.vehicleList.innerHTML = '<p class="empty-state">관측된 차량이 없습니다.</p>';
    return;
  }
  elements.vehicleList.innerHTML = vehicles.map((vehicle) => {
    const selected = vehicle.robot_id === state.selectedRobotId ? 'selected' : '';
    const connectivity = vehicle.connectivity || { state: 'unconfirmed' };
    const visualState = connectivityClass(connectivity.state);
    const fleetState = displayedFleetState(vehicle);
    const task = taskLabel(vehicle.active_task, fleetState);
    const battery = batteryLabel(vehicle.battery?.battery_raw);
    return `<button class="vehicle-item ${selected}" data-robot-id="${escapeHtml(vehicle.robot_id)}" type="button">
      <span class="vehicle-status-dot ${visualState}"></span>
      <span>
        <span class="vehicle-id">${escapeHtml(vehicle.robot_id)}</span>
        <span class="vehicle-status"><span class="vehicle-operation-state ${stateClass(fleetState)}">${fleetStateLabel(fleetState)}</span><span class="vehicle-connectivity ${visualState}">${connectivityLabel(connectivity.state)}</span></span>
        <span class="vehicle-task">${escapeHtml(task)}</span>
      </span>
      <span class="battery">${battery}</span>
    </button>`;
  }).join('');
  elements.vehicleList.querySelectorAll('[data-robot-id]').forEach((button) => {
    button.addEventListener('click', () => {
      if (state.selectedRobotId !== button.dataset.robotId) resetNavigationDraft();
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
  const navigationEnabled = enabled && !isActiveOperation(vehicle?.fleet_state?.state);
  elements.initialPoseButton.disabled = !navigationEnabled;
  elements.goalPoseButton.disabled = !navigationEnabled;
  elements.waypointButton.disabled = !navigationEnabled;
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
    elements.taskCard.innerHTML = '<p class="eyebrow">현재 작업</p><strong>선택된 차량이 없습니다.</strong><span>—</span>';
    return;
  }
  const pose = vehicle.pose;
  const connectivity = vehicle.connectivity || { state: 'unconfirmed' };
  const connectivityState = connectivityClass(connectivity.state);
  const connectivityAge = connectivityAgeLabel(connectivity);
  const yawDegrees = pose ? ((pose.yaw_rad * 180) / Math.PI).toFixed(1) : '—';
  const fleetState = displayedFleetState(vehicle) || 'UNKNOWN';
  elements.selectedName.textContent = vehicle.robot_id;
  elements.selectedState.textContent = fleetStateLabel(fleetState);
  elements.selectedState.className = `state-badge state-${stateClass(fleetState)}`;
  elements.vehicleDetails.innerHTML = `
    <div><dt>위치</dt><dd>${pose ? `${pose.x_m.toFixed(2)}, ${pose.y_m.toFixed(2)} m` : '—'}</dd></div>
    <div><dt>자세</dt><dd>${yawDegrees === '—' ? '—' : `${yawDegrees}°`}</dd></div>
    <div><dt>배터리</dt><dd>${batteryDetail(vehicle.battery?.battery_raw)}</dd></div>
    <div><dt>적재</dt><dd>${vehicle.pallet_state?.has_pallet ? vehicle.pallet_state.payload_type : '비적재'}</dd></div>
    <div><dt>통신</dt><dd class="connectivity-${connectivityState}">${connectivityLabel(connectivity.state)}${connectivityAge}</dd></div>`;
  const task = vehicle.active_task;
  const taskRoute = task?.source_zone_id && task?.destination_zone_id
    ? `${task.source_zone_id} → ${task.destination_zone_id}`
    : task ? '작업 상세 수신 대기' : fleetStateLabel(fleetState);
  elements.taskCard.innerHTML = `<p class="eyebrow">현재 작업</p><strong>${escapeHtml(taskLabel(task, fleetState))}</strong><span>${escapeHtml(taskRoute)}</span>`;
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
    const point = mapPixelToRotatedDisplay(
      mapToPixel(vehicle.pose.x_m, vehicle.pose.y_m, state.map),
      state.mapView,
    );
    const selected = vehicle.robot_id === state.selectedRobotId ? 'selected' : '';
    const rotation = svgRotationFromYaw(mapYawToRotatedDisplay(vehicle.pose.yaw_rad));
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
  elements.mapOverlay.innerHTML = markers + renderNavigationOverlay() + preview;
}

function renderNavigationOverlay() {
  if (state.navigationDraftRobotId !== state.selectedRobotId) return '';
  const draft = state.navigationDraft;
  const waypointPixels = draft.waypoints.map((waypoint) => mapPixelToRotatedDisplay(
    mapToPixel(waypoint.x, waypoint.y, state.map),
    state.mapView,
  ));
  const route = waypointPixels.length > 1
    ? `<polyline class="waypoint-route" points="${waypointPixels.map((point) => `${point.x},${point.y}`).join(' ')}"></polyline>`
    : '';
  const waypoints = draft.waypoints.map((waypoint, index) => renderNavigationMarker(
    waypoint,
    'waypoint',
    String(index + 1),
  )).join('');
  return route
    + renderNavigationMarker(draft.initialPose, 'initial', 'I')
    + renderNavigationMarker(draft.goalPose, 'goal', 'G')
    + waypoints;
}

function renderNavigationMarker(pose, kind, label) {
  if (!pose) return '';
  const point = mapPixelToRotatedDisplay(mapToPixel(pose.x, pose.y, state.map), state.mapView);
  const displayYaw = mapYawToRotatedDisplay(pose.yaw);
  const arrowLength = 5;
  const arrowEnd = {
    x: rounded(point.x + Math.cos(displayYaw) * arrowLength),
    y: rounded(point.y - Math.sin(displayYaw) * arrowLength),
  };
  return `<g class="navigation-marker ${kind}">
    <line x1="${point.x}" y1="${point.y}" x2="${arrowEnd.x}" y2="${arrowEnd.y}"></line>
    <circle cx="${point.x}" cy="${point.y}" r="2.2"></circle>
    <text x="${point.x}" y="${point.y + 0.9}">${label}</text>
  </g>`;
}

function selectedVehicle() {
  return state.snapshot?.vehicles.find((vehicle) => vehicle.robot_id === state.selectedRobotId) || null;
}

function isActiveOperation(fleetState) {
  return ['DRIVE', 'PICK', 'PLACE'].includes(fleetState);
}

function stateClass(fleetState) {
  if (fleetState === 'IDLE' || fleetState === 'WAIT' || fleetState === 'INIT') return 'idle';
  if (fleetState === 'STOPPED') return 'stopped';
  if (fleetState === 'FAIL' || fleetState === 'FAILED') return 'fail';
  return fleetState ? 'active' : 'unknown';
}

function connectivityClass(connectivityState) {
  if (connectivityState === 'online') return 'online';
  if (connectivityState === 'stale') return 'stale';
  if (connectivityState === 'offline') return 'offline';
  return 'unconfirmed';
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

function setNavigationMode(mode) {
  if (state.navigationDraft.selectMode(mode)) {
    state.navigationDraftRobotId = null;
    Object.keys(state.navigationDraftRevisions).forEach(incrementNavigationDraftRevision);
  }
  state.navigationMode = mode;
  state.dragStart = null;
  state.dragEnd = null;
  const enabled = Boolean(mode);
  elements.mapStage.classList.toggle('navigation-mode', enabled);
  elements.mapInstruction.classList.toggle('navigation-mode', enabled);
  elements.mapInstruction.textContent = enabled
    ? navigationInstruction(mode)
    : '차량을 선택하면 제어할 수 있습니다.';
  elements.initialPoseButton.classList.toggle('active', mode === 'initial');
  elements.goalPoseButton.classList.toggle('active', mode === 'goal');
  elements.waypointButton.classList.toggle('active', mode === 'waypoints');
  renderNavigationDraft();
  renderOverlay();
}

function resetNavigationDraft() {
  state.navigationDraft = new NavigationDraft();
  state.navigationDraftRobotId = null;
  Object.keys(state.navigationDraftRevisions).forEach(incrementNavigationDraftRevision);
  setNavigationMode(null);
}

function incrementNavigationDraftRevision(mode) {
  state.navigationDraftRevisions[mode] += 1;
}

function navigationInstruction(mode) {
  if (mode === 'initial') return '지도에서 초기 위치를 누른 뒤 원하는 방향으로 드래그하세요. 실행 전까지 전송되지 않습니다.';
  if (mode === 'goal') return '지도에서 목표 위치를 누른 뒤 원하는 방향으로 드래그하세요. 실행 전까지 전송되지 않습니다.';
  return '지도에서 waypoint를 순서대로 누른 뒤 원하는 방향으로 드래그하세요. 여러 point를 추가할 수 있습니다.';
}

function renderNavigationDraft() {
  const mode = state.navigationMode;
  elements.navigationDraft.hidden = !mode;
  if (!mode) return;
  const vehicle = selectedVehicle();
  const ownsDraft = state.navigationDraftRobotId === vehicle?.robot_id;
  const hasPending = ownsDraft && state.navigationDraft.hasPending(mode);
  const navigationReady = Boolean(vehicle) && !isActiveOperation(vehicle.fleet_state?.state);
  elements.executeNavigationButton.disabled = !navigationReady || !hasPending || state.navigationExecution.isPending();
  elements.clearNavigationButton.disabled = !hasPending;
  elements.navigationDraftTitle.textContent = navigationDraftTitle(mode);

  if (mode === 'waypoints') {
    const waypoints = ownsDraft ? state.navigationDraft.waypoints : [];
    elements.navigationDraftMeta.textContent = waypoints.length
      ? `${waypoints.length}개 point가 실행 대기 중입니다. 목록 순서대로 전송됩니다.`
      : '첫 번째 point의 위치와 방향을 지도에서 지정하세요.';
    elements.waypointList.hidden = false;
    elements.waypointList.innerHTML = waypoints.map((waypoint, index) => `<li>
      <span><strong>Point ${index + 1}</strong>${formatNavigationPose(waypoint)}</span>
      <button class="waypoint-remove" type="button" data-waypoint-index="${index}" aria-label="Point ${index + 1} 삭제">삭제</button>
    </li>`).join('') || '<li class="waypoint-empty">아직 지정된 point가 없습니다.</li>';
    elements.waypointList.querySelectorAll('[data-waypoint-index]').forEach((button) => {
      button.addEventListener('click', () => {
        state.navigationDraft.removeWaypoint(Number(button.dataset.waypointIndex));
        incrementNavigationDraftRevision('waypoints');
        renderNavigationDraft();
        renderOverlay();
      });
    });
    return;
  }

  const pose = ownsDraft
    ? (mode === 'initial' ? state.navigationDraft.initialPose : state.navigationDraft.goalPose)
    : null;
  elements.navigationDraftMeta.textContent = pose
    ? `${formatNavigationPose(pose)} 실행을 누르면 차량으로 전송합니다.`
    : '지도에서 위치와 방향을 지정하세요.';
  elements.waypointList.hidden = true;
  elements.waypointList.innerHTML = '';
}

function navigationDraftTitle(mode) {
  if (mode === 'initial') return 'Initial Pose 지정';
  if (mode === 'goal') return 'Goal Pose 지정';
  return 'Follow Waypoints 지정';
}

function formatNavigationPose(pose) {
  return `x ${pose.x.toFixed(2)} · y ${pose.y.toFixed(2)} · yaw ${(pose.yaw * 180 / Math.PI).toFixed(0)}°`;
}

function clearNavigationDraft(mode) {
  if (!mode) return;
  if (mode === 'waypoints') state.navigationDraft.clearWaypoints();
  else state.navigationDraft.clearPose(mode);
  incrementNavigationDraftRevision(mode);
  if (!state.navigationDraft.initialPose && !state.navigationDraft.goalPose && !state.navigationDraft.waypoints.length) {
    state.navigationDraftRobotId = null;
  }
}

function clearActiveNavigationDraft() {
  clearNavigationDraft(state.navigationMode);
  renderNavigationDraft();
  renderOverlay();
}

async function executeNavigationDraft() {
  const vehicle = selectedVehicle();
  const mode = state.navigationMode;
  if (!vehicle || !mode || state.navigationDraftRobotId !== vehicle.robot_id || !state.navigationDraft.hasPending(mode)) return;
  const execution = state.navigationExecution.begin(
    vehicle.robot_id,
    mode,
    state.navigationDraftRevisions[mode],
  );
  if (!execution) return;
  renderNavigationDraft();
  const robotPath = `/api/vehicles/${encodeURIComponent(vehicle.robot_id)}`;
  const draft = state.navigationDraft;
  const request = mode === 'initial'
    ? { path: `${robotPath}/initial-pose`, body: draft.initialPose, message: 'Initial Pose 요청을 전달했습니다.' }
    : mode === 'goal'
      ? { path: `${robotPath}/navigation/goal`, body: draft.goalPose, message: 'Goal Pose 요청을 전달했습니다.' }
      : { path: `${robotPath}/navigation/waypoints`, body: { waypoints: draft.waypoints }, message: `${draft.waypoints.length}개 Follow Waypoint 요청을 전달했습니다.` };
  try {
    await postControl(request.path, request.body);
    if (navigationExecutionMatchesDraft(
      execution,
      state.navigationDraftRobotId,
      state.navigationDraftRevisions,
    )) {
      clearNavigationDraft(execution.mode);
      if (state.navigationMode === execution.mode) setNavigationMode(null);
      else {
        renderNavigationDraft();
        renderOverlay();
      }
    }
    showToast(`${vehicle.robot_id} ${request.message}`);
    refreshSnapshot();
  } catch (error) {
    showToast(error.message, true);
  } finally {
    state.navigationExecution.complete(execution);
    renderNavigationDraft();
  }
}

function mapPointer(event) {
  if (!state.mapView) return null;
  return mapPointAtPanZoom(mapViewportPoint(event), state.mapView, state.mapZoom, state.mapPan);
}

function mapViewportPoint(event) {
  const rect = elements.mapViewport.getBoundingClientRect();
  return {
    x: ((event.clientX - rect.left) / rect.width) * state.mapView.width,
    y: ((event.clientY - rect.top) / rect.height) * state.mapView.height,
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

export function operationalMapView(pgm, padding = MAP_VIEW_PADDING_PX) {
  let minX = pgm.width;
  let minY = pgm.height;
  let maxX = -1;
  let maxY = -1;
  for (let y = 0; y < pgm.height; y += 1) {
    for (let x = 0; x < pgm.width; x += 1) {
      if (pgm.pixels[(y * pgm.width) + x] === MAP_UNKNOWN_PIXEL) continue;
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }
  const sourceBounds = maxX < 0
    ? { x: 0, y: 0, width: pgm.width, height: pgm.height }
    : {
      x: Math.max(0, minX - padding),
      y: Math.max(0, minY - padding),
      width: Math.min(pgm.width, maxX + padding + 1) - Math.max(0, minX - padding),
      height: Math.min(pgm.height, maxY + padding + 1) - Math.max(0, minY - padding),
    };
  return {
    sourceBounds,
    width: sourceBounds.height,
    height: sourceBounds.width,
  };
}

export function mapPixelToRotatedDisplay(point, view) {
  const { sourceBounds } = view;
  return {
    x: sourceBounds.y + sourceBounds.height - point.y,
    y: point.x - sourceBounds.x,
  };
}

export function rotatedDisplayToMapPixel(point, view) {
  const { sourceBounds } = view;
  return {
    x: sourceBounds.x + point.y,
    y: sourceBounds.y + sourceBounds.height - point.x,
  };
}

export function mapYawToRotatedDisplay(yaw) {
  return normalizedYaw(yaw - (Math.PI / 2));
}

export function rotatedDisplayYawToMapYaw(yaw) {
  return normalizedYaw(yaw + (Math.PI / 2));
}

function normalizedYaw(yaw) {
  const fullTurn = Math.PI * 2;
  return ((yaw + Math.PI) % fullTurn + fullTurn) % fullTurn - Math.PI;
}

function setMapZoom(zoom) {
  state.mapZoom = clampZoom(zoom);
  renderMapTransform();
}

function renderMapTransform() {
  if (!state.map) return;
  state.mapPan = clampMapPan(state.mapPan, state.mapView, state.mapZoom);
  const panXPercent = rounded((state.mapPan.x / state.mapView.width) * 100);
  const panYPercent = rounded((state.mapPan.y / state.mapView.height) * 100);
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
    state.mapView,
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
    if (selectedVehicle()) setNavigationMode('initial');
  });
  elements.goalPoseButton.addEventListener('click', () => {
    if (selectedVehicle()) setNavigationMode('goal');
  });
  elements.waypointButton.addEventListener('click', () => {
    if (selectedVehicle()) setNavigationMode('waypoints');
  });
  elements.clearNavigationButton.addEventListener('click', clearActiveNavigationDraft);
  elements.executeNavigationButton.addEventListener('click', executeNavigationDraft);
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
    const rect = elements.mapViewport.getBoundingClientRect();
    const horizontalDelta = event.deltaX + (event.shiftKey ? event.deltaY : 0);
    const verticalDelta = event.shiftKey ? 0 : event.deltaY;
    panMapBy({
      x: (-horizontalDelta / rect.width) * state.mapView.width,
      y: (-verticalDelta / rect.height) * state.mapView.height,
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
  elements.mapViewport.addEventListener('pointerdown', (event) => {
    if (!state.mapView) return;
    if (state.navigationMode && selectedVehicle()) {
      state.dragStart = mapPointer(event); state.dragEnd = state.dragStart;
      elements.mapViewport.setPointerCapture?.(event.pointerId); renderOverlay();
      return;
    }
    if (state.navigationMode) return;
    state.mapPanDrag = {
      pointerId: event.pointerId,
      start: mapViewportPoint(event),
      pan: { ...state.mapPan },
    };
    elements.mapViewport.setPointerCapture?.(event.pointerId);
    elements.mapStage.classList.add('panning');
  });
  elements.mapViewport.addEventListener('pointermove', (event) => {
    if (state.mapPanDrag?.pointerId === event.pointerId) {
      const point = mapViewportPoint(event);
      const { start, pan } = state.mapPanDrag;
      state.mapPan = clampMapPan(
        { x: pan.x + point.x - start.x, y: pan.y + point.y - start.y },
        state.mapView,
        state.mapZoom,
      );
      renderMapTransform();
      return;
    }
    if (!state.dragStart) return;
    state.dragEnd = mapPointer(event); renderOverlay();
  });
  elements.mapViewport.addEventListener('pointerup', async (event) => {
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
    const rawPoint = rotatedDisplayToMapPixel(start, state.mapView);
    const point = pixelToMap(rawPoint.x, rawPoint.y, state.map);
    const yaw = rotatedDisplayYawToMapYaw(yawFromDrag(start, end));
    const mode = state.navigationMode;
    if (!mode) return;
    if (state.navigationDraftRobotId && state.navigationDraftRobotId !== vehicle.robot_id) {
      resetNavigationDraft();
      return;
    }
    const pose = { x: point.x, y: point.y, yaw };
    state.navigationDraftRobotId = vehicle.robot_id;
    if (mode === 'waypoints') state.navigationDraft.addWaypoint(pose);
    else state.navigationDraft.setPose(mode, pose);
    incrementNavigationDraftRevision(mode);
    renderNavigationDraft();
    renderOverlay();
    const label = mode === 'initial' ? 'Initial Pose' : mode === 'goal' ? 'Goal Pose' : `Point ${state.navigationDraft.waypoints.length}`;
    showToast(`${label}를 지정했습니다. 실행을 누르면 전송합니다.`);
  });
  ['pointercancel', 'lostpointercapture'].forEach((eventName) => {
    elements.mapViewport.addEventListener(eventName, () => {
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
  scheduleSnapshotRefresh(refreshSnapshot);
}

if (typeof document !== 'undefined') bootstrap();
