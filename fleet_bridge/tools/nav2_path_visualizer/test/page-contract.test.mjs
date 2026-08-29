import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import { createContext, runInContext, Script } from 'node:vm';

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

test('page embeds navigation code so file:// has no JavaScript subresource CORS request', () => {
  const html = readFileSync(`${toolRoot}/index.html`, 'utf8');
  const model = readFileSync(`${toolRoot}/model.mjs`, 'utf8').trim();
  const app = readFileSync(`${toolRoot}/app.mjs`, 'utf8').trim();

  assert.doesNotMatch(html, /<script[^>]+\bsrc=/);
  assert.ok(html.includes(`<script>\n(() => {\n${model}\n})();\n</script>`));
  assert.ok(html.includes(`<script>\n(() => {\n${app}\n})();\n</script>`));
  assert.doesNotMatch(app, /^import\s/m);
});

test('navigation model can run as a classic browser script', () => {
  const model = readFileSync(`${toolRoot}/model.mjs`, 'utf8');
  const sandbox = createContext({ TextDecoder });

  runInContext(model, sandbox);

  assert.equal(typeof sandbox.Nav2PathModel?.computeNavigation, 'function');
  assert.equal(typeof sandbox.Nav2PathModel?.parsePgm, 'function');
});

test('embedded model and app do not redeclare lexical bindings in the page scope', () => {
  const html = readFileSync(`${toolRoot}/index.html`, 'utf8');
  const scripts = [...html.matchAll(/<script>\n([\s\S]*?)\n<\/script>/g)].map((match) => match[1]);

  assert.equal(scripts.length, 2);
  assert.doesNotThrow(() => new Script(scripts.join('\n')));
});

test('standalone build can refresh an already bundled page', () => {
  assert.doesNotThrow(() => {
    execFileSync(process.execPath, [`${toolRoot}/build-standalone.mjs`], { stdio: 'pipe' });
  });
});
