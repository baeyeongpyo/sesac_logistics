import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';

const toolRoot = fileURLToPath(new URL('..', import.meta.url));

test('standalone page exposes local map inputs and all three navigation layers', () => {
  const html = readFileSync(`${toolRoot}/index.html`, 'utf8');
  const app = readFileSync(`${toolRoot}/app.mjs`, 'utf8');

  assert.match(html, /id="pgm-file"/);
  assert.match(html, /id="yaml-file"/);
  assert.match(html, /id="use-example-map"/);
  assert.match(html, /id="start-x"/);
  assert.match(html, /id="goal-yaw"/);
  assert.match(html, /Global plan/);
  assert.match(html, /Transformed plan/);
  assert.match(html, /Local plan/);
  assert.match(html, /id="map-canvas"/);
  assert.match(app, /computeNavigation/);
  assert.doesNotMatch(app, /\bfetch\s*\(/);
});
