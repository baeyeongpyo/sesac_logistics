import test from 'node:test';
import assert from 'node:assert/strict';
import { createServer } from 'node:http';
import { existsSync, mkdtempSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join, normalize, resolve } from 'node:path';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';

const toolRoot = fileURLToPath(new URL('..', import.meta.url));
const pagePath = join(toolRoot, 'nav2-stack-explainer.html');
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
    const relativePath = request.url === '/' ? 'nav2-stack-explainer.html' : request.url.slice(1).split('?')[0];
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
      && document.title === 'MentorPi Nav2 목표 주행 흐름'
      && document.querySelectorAll('[data-stage-button]').length === 7`);
    if (ready) return;
    await wait(50);
  }
  throw new Error('설명 페이지가 렌더링 완료 상태가 되지 않았습니다.');
}

test('Nav2 흐름 설명 페이지는 단계 선택과 장애물 상황 전환을 실제 브라우저에서 반영한다', async () => {
  assert.ok(existsSync(pagePath), '설명용 단독 HTML 페이지가 있어야 합니다.');
  assert.ok(existsSync(chromePath), 'Chrome이 있어야 정적 페이지의 실제 동작을 검사할 수 있습니다.');

  const { server, port } = await startStaticServer(toolRoot);
  const debugPort = await reservePort();
  const profilePath = mkdtempSync(join(tmpdir(), 'nav2-stack-explainer-'));
  const pageUrl = `http://127.0.0.1:${port}/nav2-stack-explainer.html`;
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
      title: document.title,
      stage: document.body.dataset.stage,
      scenario: document.body.dataset.scenario,
      stages: document.querySelectorAll('[data-stage-button]').length,
      scenarios: document.querySelectorAll('[data-scenario-button]').length,
    })`);

    assert.deepEqual(initialState, {
      title: 'MentorPi Nav2 목표 주행 흐름',
      stage: 'fleet',
      scenario: 'normal',
      stages: 7,
      scenarios: 3,
    });

    const switchedState = await evaluate(debuggerUrl, `(() => {
      document.querySelector('[data-stage-button="dwb"]').click();
      document.querySelector('[data-scenario-button="blocked"]').click();
      return {
        stage: document.body.dataset.stage,
        scenario: document.body.dataset.scenario,
        detail: document.querySelector('#stage-detail-title').textContent.trim(),
        scenarioText: document.querySelector('#scenario-description').textContent.trim(),
        dwbPressed: document.querySelector('[data-stage-button="dwb"]').getAttribute('aria-pressed'),
        blockedPressed: document.querySelector('[data-scenario-button="blocked"]').getAttribute('aria-pressed'),
      };
    })()`);

    assert.deepEqual(switchedState, {
      stage: 'dwb',
      scenario: 'blocked',
      detail: 'DWB · 로컬 제어',
      scenarioText: 'Collision Monitor가 감속 또는 정지를 우선합니다. 장시간 진행하지 못하면 Nav2 액션은 실패할 수 있습니다.',
      dwbPressed: 'true',
      blockedPressed: 'true',
    });
  } finally {
    chrome.kill();
    await new Promise((resolveClose) => server.close(resolveClose));
    rmSync(profilePath, { recursive: true, force: true });
  }
});
