import test from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { existsSync, mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, normalize, resolve } from 'node:path';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import { fileURLToPath } from 'node:url';

const toolRoot = fileURLToPath(new URL('..', import.meta.url));
const pageName = 'graph-route-explainer.html';
const pagePath = join(toolRoot, pageName);
const chromePath = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome';

function wait(milliseconds) {
  return new Promise((resolveWait) => setTimeout(resolveWait, milliseconds));
}

function reservePort() {
  return new Promise((resolvePort) => {
    const server = createServer();
    server.listen(0, '127.0.0.1', () => {
      const { port } = server.address();
      server.close(() => resolvePort(port));
    });
  });
}

function startStaticServer(root) {
  const rootPath = resolve(root);
  const server = createServer((request, response) => {
    const relativePath = request.url === '/' ? pageName : request.url.slice(1).split('?')[0];
    const filePath = resolve(rootPath, normalize(relativePath));
    if (!filePath.startsWith(`${rootPath}/`) || !existsSync(filePath)) {
      response.writeHead(404).end('Not found');
      return;
    }
    response.writeHead(200, { 'content-type': 'text/html; charset=utf-8' });
    response.end(readFileSync(filePath));
  });
  return new Promise((resolveServer) => {
    server.listen(0, '127.0.0.1', () => {
      resolveServer({ server, port: server.address().port });
    });
  });
}

function readJson(url) {
  return fetch(url).then((response) => response.json());
}

async function waitForDebuggerTarget(port, expectedUrl) {
  for (let attempt = 0; attempt < 80; attempt += 1) {
    try {
      const targets = await readJson(`http://127.0.0.1:${port}/json/list`);
      const target = targets.find((item) => item.type === 'page' && item.url === expectedUrl);
      if (target) return target.webSocketDebuggerUrl;
    } catch {
      // Chrome's debugging endpoint is not ready yet.
    }
    await wait(50);
  }
  throw new Error('Chrome DevTools target을 찾지 못했습니다.');
}

async function evaluate(webSocketDebuggerUrl, expression) {
  const socket = new WebSocket(webSocketDebuggerUrl);
  await new Promise((resolveOpen, rejectOpen) => {
    socket.addEventListener('open', resolveOpen, { once: true });
    socket.addEventListener('error', rejectOpen, { once: true });
  });
  const result = await new Promise((resolveResult, rejectResult) => {
    socket.addEventListener('message', (event) => {
      const message = JSON.parse(event.data);
      if (message.id === 1) resolveResult(message);
    });
    socket.addEventListener('error', rejectResult, { once: true });
    socket.send(JSON.stringify({
      id: 1,
      method: 'Runtime.evaluate',
      params: { expression, returnByValue: true, awaitPromise: true },
    }));
  });
  socket.close();
  if (result.result.exceptionDetails) {
    throw new Error(result.result.exceptionDetails.text);
  }
  return result.result.result.value;
}

async function waitForPageContent(webSocketDebuggerUrl) {
  for (let attempt = 0; attempt < 80; attempt += 1) {
    const ready = await evaluate(webSocketDebuggerUrl, `document.readyState === 'complete'
      && document.title === 'Graph Router와 Graph Planner'
      && document.querySelectorAll('[data-map-node]').length >= 6`);
    if (ready) return;
    await wait(50);
  }
  throw new Error('Graph Route 설명 페이지가 렌더링 완료 상태가 되지 않았습니다.');
}

async function stopChrome(chrome) {
  if (chrome.exitCode !== null) return;
  const exited = once(chrome, 'exit');
  chrome.kill();
  await exited;
}

test('지도에서 출발지와 도착지를 고르면 Router 경로와 Planner Path가 함께 생성된다', async () => {
  assert.ok(existsSync(pagePath), 'Graph Route 설명용 단독 HTML 페이지가 있어야 합니다.');
  assert.ok(existsSync(chromePath), 'Chrome이 있어야 정적 페이지의 실제 동작을 검사할 수 있습니다.');

  const { server, port } = await startStaticServer(toolRoot);
  const debugPort = await reservePort();
  const profilePath = mkdtempSync(join(tmpdir(), 'graph-route-explainer-'));
  const pageUrl = `http://127.0.0.1:${port}/${pageName}`;
  const chrome = spawn(chromePath, [
    '--headless=new',
    '--no-first-run',
    '--no-default-browser-check',
    `--user-data-dir=${profilePath}`,
    `--remote-debugging-port=${debugPort}`,
    pageUrl,
  ], { stdio: 'ignore' });

  try {
    const debuggerUrl = await waitForDebuggerTarget(debugPort, pageUrl);
    await waitForPageContent(debuggerUrl);

    const initialState = await evaluate(debuggerUrl, `({
      phase: document.body.dataset.phase,
      start: document.querySelector('#start-node').textContent.trim(),
      goal: document.querySelector('#goal-node').textContent.trim(),
      route: document.querySelector('#route-plan').textContent.trim(),
    })`);

    assert.deepEqual(initialState, {
      phase: 'pick-start',
      start: '미지정',
      goal: '미지정',
      route: '출발지와 도착지를 지도에서 차례로 선택하세요.',
    });

    const plannedState = await evaluate(debuggerUrl, `(() => {
      document.querySelector('[data-map-node="dock-a"]').click();
      document.querySelector('[data-map-node="dock-d"]').click();
      return {
        phase: document.body.dataset.phase,
        start: document.querySelector('#start-node').textContent.trim(),
        goal: document.querySelector('#goal-node').textContent.trim(),
        route: document.querySelector('#route-plan').textContent.trim(),
        selectedLanes: document.querySelectorAll('[data-lane].is-selected').length,
        plannerPath: document.querySelector('#planner-path').getAttribute('points'),
        plannerOutput: document.querySelector('#planner-output').textContent.trim(),
      };
    })()`);

    assert.deepEqual(plannedState, {
      phase: 'planned',
      start: 'Dock A',
      goal: 'Dock D',
      route: 'lane-01 → lane-05 → lane-06',
      selectedLanes: 3,
      plannerPath: '100,400 240,400 400,400 700,400',
      plannerOutput: 'nav_msgs/Path · 4 poses · map frame',
    });
  } finally {
    await stopChrome(chrome);
    await new Promise((resolveClose) => server.close(resolveClose));
    rmSync(profilePath, { recursive: true, force: true });
  }
});
