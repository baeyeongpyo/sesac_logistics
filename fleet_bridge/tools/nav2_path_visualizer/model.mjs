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
