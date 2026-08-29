const decoder = new TextDecoder();

export const NAVIGATION_CONFIG = Object.freeze({
  robotRadius: 0.16,
  inflationRadius: 0.28,
  localWindowSize: 3.0,
  goalTolerance: 0.25,
  minVelocityX: -0.08,
  maxVelocityX: 0.22,
  maxVelocityTheta: 0.75,
  velocityXSamples: 15,
  velocityThetaSamples: 20,
  simulationTime: 1.5,
  linearGranularity: 0.04,
  angularGranularity: 0.025,
  criticScales: Object.freeze({
    obstacle: 0.02,
    pathAlign: 24,
    pathDistance: 24,
    goalAlign: 18,
    goalDistance: 18,
    rotateToGoal: 28,
  }),
});

function isWhitespace(byte) {
  return byte === 9 || byte === 10 || byte === 13 || byte === 32;
}

function nextHeaderToken(bytes, offset) {
  let index = offset;
  while (index < bytes.length) {
    if (isWhitespace(bytes[index])) {
      index += 1;
      continue;
    }
    if (bytes[index] === 35) {
      while (index < bytes.length && bytes[index] !== 10 && bytes[index] !== 13) {
        index += 1;
      }
      continue;
    }
    break;
  }

  const start = index;
  while (index < bytes.length && !isWhitespace(bytes[index]) && bytes[index] !== 35) {
    index += 1;
  }
  if (start === index) {
    throw new Error('PGM header가 완전하지 않습니다.');
  }
  return { value: decoder.decode(bytes.slice(start, index)), offset: index };
}

function readPositiveInteger(bytes, offset, label) {
  const token = nextHeaderToken(bytes, offset);
  const value = Number(token.value);
  if (!Number.isInteger(value) || value <= 0) {
    throw new Error(`PGM ${label} 값이 올바르지 않습니다.`);
  }
  return { value, offset: token.offset };
}

function consumeP5Separator(bytes, offset) {
  if (!isWhitespace(bytes[offset])) {
    throw new Error('P5 header 뒤에 binary pixel 구분자가 없습니다.');
  }
  if (bytes[offset] === 13 && bytes[offset + 1] === 10) {
    return offset + 2;
  }
  return offset + 1;
}

/** Parse P2 or P5 Portable Graymap bytes without a browser or ROS dependency. */
export function parsePgm(input) {
  const bytes = input instanceof Uint8Array ? input : new Uint8Array(input);
  const magic = nextHeaderToken(bytes, 0);
  if (magic.value !== 'P2' && magic.value !== 'P5') {
    throw new Error('지원하지 않는 지도 형식입니다. P2 또는 P5 PGM 파일을 선택하세요.');
  }

  const width = readPositiveInteger(bytes, magic.offset, 'width');
  const height = readPositiveInteger(bytes, width.offset, 'height');
  const maxValue = readPositiveInteger(bytes, height.offset, 'max value');
  if (maxValue.value > 255) {
    throw new Error('16-bit PGM 지도는 지원하지 않습니다. max value가 255 이하인 파일을 선택하세요.');
  }

  const pixelCount = width.value * height.value;
  let pixels;
  if (magic.value === 'P5') {
    const pixelOffset = consumeP5Separator(bytes, maxValue.offset);
    if (bytes.length - pixelOffset !== pixelCount) {
      throw new Error(`P5 pixel 수가 지도 크기와 맞지 않습니다. ${pixelCount}개가 필요합니다.`);
    }
    pixels = Array.from(bytes.slice(pixelOffset));
  } else {
    const values = [];
    let offset = maxValue.offset;
    while (true) {
      try {
        const token = nextHeaderToken(bytes, offset);
        values.push(Number(token.value));
        offset = token.offset;
      } catch (error) {
        if (error.message !== 'PGM header가 완전하지 않습니다.') throw error;
        break;
      }
    }
    if (values.length !== pixelCount || values.some((value) => !Number.isInteger(value) || value < 0 || value > maxValue.value)) {
      throw new Error(`P2 pixel 수 또는 값이 올바르지 않습니다. ${pixelCount}개가 필요합니다.`);
    }
    pixels = values;
  }

  return { width: width.value, height: height.value, maxValue: maxValue.value, pixels };
}

function yamlNumber(text, key, fallback) {
  const match = text.match(new RegExp(`^\\s*${key}:\\s*([^#\\r\\n]+)`, 'm'));
  if (!match) return fallback;
  const value = Number(match[1].trim());
  if (!Number.isFinite(value)) {
    throw new Error(`지도 YAML의 ${key} 값이 숫자가 아닙니다.`);
  }
  return value;
}

/** Read only the map_server YAML keys this static visualizer needs. */
export function parseMapYaml(text) {
  if (typeof text !== 'string') {
    throw new Error('지도 YAML은 text여야 합니다.');
  }
  const resolution = yamlNumber(text, 'resolution');
  const originMatch = text.match(/^\s*origin:\s*\[\s*([^,\]]+)\s*,\s*([^,\]]+)\s*,/m);
  if (!Number.isFinite(resolution) || resolution <= 0 || !originMatch) {
    throw new Error('지도 YAML에는 양수 resolution과 [x, y, yaw] origin이 필요합니다.');
  }
  const origin = [Number(originMatch[1]), Number(originMatch[2])];
  if (!origin.every(Number.isFinite)) {
    throw new Error('지도 YAML origin 값이 숫자가 아닙니다.');
  }
  return {
    resolution,
    origin,
    negate: yamlNumber(text, 'negate', 0),
    occupiedThresh: yamlNumber(text, 'occupied_thresh', 0.65),
    freeThresh: yamlNumber(text, 'free_thresh', 0.25),
  };
}

/** Convert PGM intensity into map_server-compatible occupancy states. */
export function createOccupancyMap(pgm, metadata) {
  if (!pgm || !metadata || pgm.pixels.length !== pgm.width * pgm.height) {
    throw new Error('PGM과 지도 metadata가 일치하지 않습니다.');
  }
  const cells = pgm.pixels.map((pixel) => {
    const occupancy = metadata.negate ? pixel / pgm.maxValue : 1 - (pixel / pgm.maxValue);
    if (occupancy > metadata.occupiedThresh) return 'occupied';
    if (occupancy < metadata.freeThresh) return 'free';
    return 'unknown';
  });
  return {
    width: pgm.width,
    height: pgm.height,
    resolution: metadata.resolution,
    origin: [...metadata.origin],
    cells,
  };
}

export function worldToGrid(map, pose) {
  const column = Math.floor((pose.x - map.origin[0]) / map.resolution);
  const fromBottom = Math.floor((pose.y - map.origin[1]) / map.resolution);
  return { column, row: map.height - 1 - fromBottom };
}

export function gridToWorld(map, cell) {
  return {
    x: map.origin[0] + ((cell.column + 0.5) * map.resolution),
    y: map.origin[1] + ((map.height - cell.row - 0.5) * map.resolution),
  };
}

export function createExampleMap() {
  const width = 20;
  const height = 14;
  const pixels = Array.from({ length: width * height }, (_, index) => {
    const row = Math.floor(index / width);
    const column = index % width;
    const wall = row === 0 || row === height - 1 || column === 0 || column === width - 1 || (column === 9 && row > 2 && row < 11);
    return wall ? 0 : 255;
  });
  return createOccupancyMap(
    { width, height, maxValue: 255, pixels },
    { resolution: 0.2, origin: [-2, -1.4], negate: 0, occupiedThresh: 0.65, freeThresh: 0.25 },
  );
}

class MinPriorityQueue {
  constructor() {
    this.items = [];
  }

  push(priority, value) {
    const entry = { priority, value };
    this.items.push(entry);
    let index = this.items.length - 1;
    while (index > 0) {
      const parent = Math.floor((index - 1) / 2);
      if (this.items[parent].priority <= entry.priority) break;
      this.items[index] = this.items[parent];
      index = parent;
    }
    this.items[index] = entry;
  }

  pop() {
    if (this.items.length === 0) return undefined;
    const first = this.items[0];
    const last = this.items.pop();
    if (this.items.length > 0) {
      let index = 0;
      while (true) {
        const left = (index * 2) + 1;
        const right = left + 1;
        if (left >= this.items.length) break;
        const child = right < this.items.length && this.items[right].priority < this.items[left].priority ? right : left;
        if (this.items[child].priority >= last.priority) break;
        this.items[index] = this.items[child];
        index = child;
      }
      this.items[index] = last;
    }
    return first;
  }

  get size() {
    return this.items.length;
  }
}

const NEIGHBORS = Object.freeze([
  { column: -1, row: -1, multiplier: Math.SQRT2 },
  { column: 0, row: -1, multiplier: 1 },
  { column: 1, row: -1, multiplier: Math.SQRT2 },
  { column: -1, row: 0, multiplier: 1 },
  { column: 1, row: 0, multiplier: 1 },
  { column: -1, row: 1, multiplier: Math.SQRT2 },
  { column: 0, row: 1, multiplier: 1 },
  { column: 1, row: 1, multiplier: Math.SQRT2 },
]);

function cellIndex(map, cell) {
  return (cell.row * map.width) + cell.column;
}

function isInMap(map, cell) {
  return Number.isInteger(cell.column)
    && Number.isInteger(cell.row)
    && cell.column >= 0
    && cell.column < map.width
    && cell.row >= 0
    && cell.row < map.height;
}

function hasFinitePose(pose) {
  return pose && Number.isFinite(pose.x) && Number.isFinite(pose.y) && Number.isFinite(pose.yaw);
}

function normalizeAngle(angle) {
  return Math.atan2(Math.sin(angle), Math.cos(angle));
}

function angleDistance(first, second) {
  return Math.abs(normalizeAngle(first - second));
}

function poseDistance(first, second) {
  return Math.hypot(first.x - second.x, first.y - second.y);
}

/** Approximate each cell's distance from obstacles with an 8-connected distance field. */
function buildClearanceField(map) {
  const clearance = new Float64Array(map.cells.length);
  clearance.fill(Infinity);
  const queue = new MinPriorityQueue();

  for (let index = 0; index < map.cells.length; index += 1) {
    if (map.cells[index] === 'occupied') {
      clearance[index] = 0;
      queue.push(0, index);
    }
  }

  while (queue.size > 0) {
    const { priority: distance, value: index } = queue.pop();
    if (distance !== clearance[index]) continue;
    const row = Math.floor(index / map.width);
    const column = index % map.width;
    for (const neighbor of NEIGHBORS) {
      const cell = { column: column + neighbor.column, row: row + neighbor.row };
      if (!isInMap(map, cell)) continue;
      const neighborIndex = cellIndex(map, cell);
      const candidate = distance + (map.resolution * neighbor.multiplier);
      if (candidate >= clearance[neighborIndex]) continue;
      clearance[neighborIndex] = candidate;
      queue.push(candidate, neighborIndex);
    }
  }

  return clearance;
}

function inflationCost(clearance, config) {
  if (clearance <= config.robotRadius) return Infinity;
  if (clearance >= config.inflationRadius) return 0;
  const ratio = (config.inflationRadius - clearance) / (config.inflationRadius - config.robotRadius);
  return ratio * ratio;
}

function isTraversable(map, clearanceField, cell, config) {
  if (!isInMap(map, cell)) return false;
  const index = cellIndex(map, cell);
  return map.cells[index] !== 'occupied' && clearanceField[index] > config.robotRadius;
}

function resolveGoalCell(map, clearanceField, goal, config) {
  const preferred = worldToGrid(map, goal);
  if (isTraversable(map, clearanceField, preferred, config)) return preferred;

  let best;
  for (let row = 0; row < map.height; row += 1) {
    for (let column = 0; column < map.width; column += 1) {
      const cell = { column, row };
      if (!isTraversable(map, clearanceField, cell, config)) continue;
      const pose = gridToWorld(map, cell);
      const distance = poseDistance(pose, goal);
      if (distance > config.goalTolerance || (best && distance >= best.distance)) continue;
      best = { cell, distance };
    }
  }
  return best?.cell;
}

function findGlobalPath(map, clearanceField, startCell, goalCell, config) {
  const length = map.cells.length;
  const distance = new Float64Array(length);
  distance.fill(Infinity);
  const previous = new Int32Array(length);
  previous.fill(-1);
  const startIndex = cellIndex(map, startCell);
  const goalIndex = cellIndex(map, goalCell);
  const queue = new MinPriorityQueue();

  distance[startIndex] = 0;
  queue.push(0, startIndex);

  while (queue.size > 0) {
    const current = queue.pop();
    if (current.priority !== distance[current.value]) continue;
    if (current.value === goalIndex) break;
    const row = Math.floor(current.value / map.width);
    const column = current.value % map.width;

    for (const neighbor of NEIGHBORS) {
      const cell = { column: column + neighbor.column, row: row + neighbor.row };
      if (!isTraversable(map, clearanceField, cell, config)) continue;
      const neighborIndex = cellIndex(map, cell);
      const clearancePenalty = inflationCost(clearanceField[neighborIndex], config);
      const travelCost = map.resolution * neighbor.multiplier * (1 + (3 * clearancePenalty));
      const candidate = current.priority + travelCost;
      if (candidate >= distance[neighborIndex]) continue;
      distance[neighborIndex] = candidate;
      previous[neighborIndex] = current.value;
      queue.push(candidate, neighborIndex);
    }
  }

  if (!Number.isFinite(distance[goalIndex])) return undefined;
  const cells = [];
  for (let index = goalIndex; index !== -1; index = previous[index]) {
    cells.push({ column: index % map.width, row: Math.floor(index / map.width) });
  }
  return cells.reverse();
}

function cellsToPoses(map, cells, goalYaw) {
  return cells.map((cell, index) => {
    const pose = gridToWorld(map, cell);
    const next = cells[index + 1];
    if (!next) return { ...pose, yaw: goalYaw };
    const nextPose = gridToWorld(map, next);
    return { ...pose, yaw: Math.atan2(nextPose.y - pose.y, nextPose.x - pose.x) };
  });
}

function transformPlan(globalPlan, start, config) {
  const localRadius = config.localWindowSize / 2;
  return globalPlan.filter((pose) => poseDistance(pose, start) <= localRadius);
}

function sampleVelocity(minimum, maximum, count, index) {
  if (count === 1) return minimum;
  return minimum + (((maximum - minimum) * index) / (count - 1));
}

function simulateTrajectory(start, velocityX, velocityTheta, map, clearanceField, config) {
  const poses = [{ ...start }];
  const linearStep = Math.abs(velocityX) > 0 ? config.linearGranularity / Math.abs(velocityX) : Infinity;
  const angularStep = Math.abs(velocityTheta) > 0 ? config.angularGranularity / Math.abs(velocityTheta) : Infinity;
  const timeStep = Math.min(0.05, linearStep, angularStep);
  let elapsed = 0;
  let pose = { ...start };

  while (elapsed < config.simulationTime) {
    const delta = Math.min(timeStep, config.simulationTime - elapsed);
    const yaw = normalizeAngle(pose.yaw + (velocityTheta * delta));
    pose = {
      x: pose.x + (velocityX * Math.cos(yaw) * delta),
      y: pose.y + (velocityX * Math.sin(yaw) * delta),
      yaw,
    };
    const cell = worldToGrid(map, pose);
    if (!isTraversable(map, clearanceField, cell, config)) return undefined;
    poses.push(pose);
    elapsed += delta;
  }
  return poses;
}

function closestPlanPose(pose, plan) {
  return plan.reduce((closest, candidate) => (
    poseDistance(candidate, pose) < poseDistance(closest, pose) ? candidate : closest
  ));
}

function scoreTrajectory(trajectory, transformedPlan, goal, clearanceField, map, config) {
  const finalPose = trajectory.at(-1);
  const nearestPathPose = closestPlanPose(finalPose, transformedPlan);
  const obstacleRisk = trajectory.reduce((total, pose) => {
    const clearance = clearanceField[cellIndex(map, worldToGrid(map, pose))];
    return total + inflationCost(clearance, config);
  }, 0) / trajectory.length;
  const goalDistance = poseDistance(finalPose, goal);
  const goalAlign = angleDistance(finalPose.yaw, goal.yaw);
  const rotateToGoal = goalDistance <= config.goalTolerance ? goalAlign : 0;

  return (obstacleRisk * config.criticScales.obstacle)
    + (angleDistance(finalPose.yaw, nearestPathPose.yaw) * config.criticScales.pathAlign)
    + (poseDistance(finalPose, nearestPathPose) * config.criticScales.pathDistance)
    + (goalAlign * config.criticScales.goalAlign)
    + (goalDistance * config.criticScales.goalDistance)
    + (rotateToGoal * config.criticScales.rotateToGoal);
}

function inputError(code, message) {
  return { ok: false, code, message };
}

/**
 * Return a dependency-free NavFn/DWB-inspired approximation.
 * It intentionally mirrors the three plan layers, not Nav2's C++ implementation.
 */
export function computeNavigation({ map, start, goal, config = {} }) {
  if (!map || !Array.isArray(map.cells) || !hasFinitePose(start) || !hasFinitePose(goal)) {
    return inputError('INVALID_INPUT', '지도와 시작·목표 위치를 확인하세요.');
  }
  const activeConfig = {
    ...NAVIGATION_CONFIG,
    ...config,
    criticScales: { ...NAVIGATION_CONFIG.criticScales, ...config.criticScales },
  };
  const clearanceField = buildClearanceField(map);
  const startCell = worldToGrid(map, start);
  if (!isTraversable(map, clearanceField, startCell, activeConfig)) {
    return inputError('START_IN_COLLISION', '시작 위치가 지도 밖이거나 충돌 영역에 있습니다.');
  }
  const goalCell = resolveGoalCell(map, clearanceField, goal, activeConfig);
  if (!goalCell) {
    return inputError('GOAL_UNREACHABLE', '목표 위치 또는 허용 오차 안에 주행 가능한 셀이 없습니다.');
  }
  const globalCells = findGlobalPath(map, clearanceField, startCell, goalCell, activeConfig);
  if (!globalCells) {
    return inputError('GLOBAL_PATH_NOT_FOUND', '시작과 목표 사이에서 전역 경로를 찾지 못했습니다.');
  }

  const resolvedGoal = { ...gridToWorld(map, goalCell), yaw: goal.yaw };
  const globalPlan = cellsToPoses(map, globalCells, resolvedGoal.yaw);
  const transformedGlobalPlan = transformPlan(globalPlan, start, activeConfig);
  let candidateCount = 0;
  let rejectedCandidateCount = 0;
  let selected;

  for (let xIndex = 0; xIndex < activeConfig.velocityXSamples; xIndex += 1) {
    const velocityX = sampleVelocity(activeConfig.minVelocityX, activeConfig.maxVelocityX, activeConfig.velocityXSamples, xIndex);
    for (let thetaIndex = 0; thetaIndex < activeConfig.velocityThetaSamples; thetaIndex += 1) {
      const velocityTheta = sampleVelocity(-activeConfig.maxVelocityTheta, activeConfig.maxVelocityTheta, activeConfig.velocityThetaSamples, thetaIndex);
      candidateCount += 1;
      const trajectory = simulateTrajectory(start, velocityX, velocityTheta, map, clearanceField, activeConfig);
      if (!trajectory) {
        rejectedCandidateCount += 1;
        continue;
      }
      const score = scoreTrajectory(trajectory, transformedGlobalPlan, resolvedGoal, clearanceField, map, activeConfig);
      if (!selected || score < selected.score) {
        selected = { trajectory, velocityX, velocityTheta, score };
      }
    }
  }

  return {
    ok: true,
    globalPlan,
    transformedGlobalPlan,
    localPlan: selected?.trajectory ?? [],
    summary: {
      candidateCount,
      rejectedCandidateCount,
      selectedVx: selected?.velocityX ?? null,
      selectedVtheta: selected?.velocityTheta ?? null,
      score: selected?.score ?? null,
      unknownCellsTraversed: globalCells.filter((cell) => map.cells[cellIndex(map, cell)] === 'unknown').length,
      resolvedGoal,
      reason: selected ? undefined : 'NO_VALID_LOCAL_PLAN',
    },
  };
}
