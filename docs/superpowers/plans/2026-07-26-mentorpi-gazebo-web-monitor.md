# MentorPi Gazebo Web Monitor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 외부 사용자가 Gazebo/ROS 네트워크에 직접 접속하지 않고 HTTPS/WSS 브라우저로 창고와 두 로봇을 모니터링하게 한다.

**Architecture:** Ubuntu 24.04 기반 `gazebo-web-bridge`가 Harmonic `gz-launch7` WebSocket plugin을 실행하고, `web-gateway`는 gzweb 3.0.1 정적 앱과 WebSocket reverse proxy를 제공한다. 원본 9002 포트는 Compose 내부에만 존재하고 외부에는 gateway만 공개한다.

**Tech Stack:** Gazebo Harmonic 8.x, gz-launch7 WebsocketServer, gzweb 3.0.1, Node.js 24, Vite, TypeScript, Vitest, Caddy 2, Docker Compose

## Global Constraints

- 웹은 모델 소스 편집 기능을 제공하지 않는다.
- 웹 모니터 장애가 Gazebo와 SLAM을 중단시키지 않는다.
- 외부에는 gateway의 HTTP/HTTPS 포트만 공개한다.
- Gazebo Transport, ROS 2 DDS, WebSocket 원본 9002 포트를 공개하지 않는다.
- production에서는 TLS와 인증 정보를 서버 secret으로 주입한다.

## File Map

- `vehicle_simulator_model/ubuntu/websocket.gzlaunch`: Harmonic WebSocket server 설정
- `vehicle_simulator_model/ubuntu/Dockerfile.webbridge`: Ubuntu 24.04 WebSocket bridge
- `vehicle_simulator_model/ubuntu/web/`: gzweb monitor frontend
- `vehicle_simulator_model/ubuntu/web/Caddyfile`: static files, auth, WSS proxy
- `vehicle_simulator_model/ubuntu/web/Dockerfile`: Node build + Caddy runtime
- `vehicle_simulator_model/ubuntu/compose.yaml`: bridge/gateway web profile
- `vehicle_simulator_model/ubuntu/test/test_web_bundle.py`: 배포 보안 계약

---

### Task 1: Harmonic WebSocket bridge

**Files:**
- Create: `vehicle_simulator_model/ubuntu/websocket.gzlaunch`
- Create: `vehicle_simulator_model/ubuntu/Dockerfile.webbridge`
- Create: `vehicle_simulator_model/ubuntu/test/test_web_bundle.py`
- Modify: `vehicle_simulator_model/ubuntu/compose.yaml`

**Interfaces:**
- Consumes: Gazebo Transport partition `mentorpi-sim`
- Produces: internal WebSocket endpoint `gazebo-web-bridge:9002`

- [ ] **Step 1: Write the failing bridge contract**

```python
class WebBundleTest(unittest.TestCase):
    def test_websocket_bridge_is_internal_and_authenticated(self):
        launch = (BUNDLE / 'websocket.gzlaunch').read_text()
        compose = (BUNDLE / 'compose.yaml').read_text()
        self.assertIn('gz::launch::WebsocketServer', launch)
        self.assertIn('gz-launch-websocket-server', launch)
        self.assertIn('<port>9002</port>', launch)
        self.assertIn('<publication_hz>30</publication_hz>', launch)
        self.assertIn('<max_connections>8</max_connections>', launch)
        self.assertIn('gazebo-web-bridge:', compose)
        self.assertIn('expose:', compose)
        self.assertIn(\"- '9002'\", compose)
        self.assertNotIn(\"- '9002:9002'\", compose)
```

- [ ] **Step 2: Run and verify missing bridge assets**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_web_bundle.py -v
```

Expected: ERROR because the launch and Dockerfile do not exist.

- [ ] **Step 3: Add WebSocket launch configuration**

```xml
<?xml version="1.0"?>
<gz version="1.0">
  <plugin name="gz::launch::WebsocketServer"
          filename="gz-launch-websocket-server">
    <port>9002</port>
    <publication_hz>30</publication_hz>
    <max_connections>8</max_connections>
  </plugin>
</gz>
```

- [ ] **Step 4: Add the Ubuntu 24.04 bridge image**

Install `gz-harmonic` from the OSRF repository and copy `websocket.gzlaunch`. Copy source assets to the exact installed paths used by scene resource URIs:

```dockerfile
COPY ros2_ws/src/mentorpi_description \
  /opt/mentorpi_ws/install/mentorpi_description/share/mentorpi_description
COPY ros2_ws/src/mentorpi_gz_sim \
  /opt/mentorpi_ws/install/mentorpi_gz_sim/share/mentorpi_gz_sim
ENV GZ_SIM_RESOURCE_PATH=/opt/mentorpi_ws/install/mentorpi_description/share:/opt/mentorpi_ws/install/mentorpi_gz_sim/share
CMD ["gz", "launch", "-v", "4", "/etc/mentorpi/websocket.gzlaunch"]
```

Use Ubuntu 24.04 for the web bridge to avoid Jammy libwebsockets file-descriptor allocation behavior while keeping the ROS runtime on Ubuntu 22.04.

- [ ] **Step 5: Add the internal Compose service**

Use profile `web`, `GZ_PARTITION=mentorpi-sim`, `expose: ['9002']`, no `ports`, restart policy, and no `depends_on` condition that would stop Gazebo when the bridge fails.

- [ ] **Step 6: Run tests and commit**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_web_bundle.py -v
docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
```

Expected: PASS.

```bash
git add vehicle_simulator_model/ubuntu/websocket.gzlaunch \
        vehicle_simulator_model/ubuntu/Dockerfile.webbridge \
        vehicle_simulator_model/ubuntu/compose.yaml \
        vehicle_simulator_model/ubuntu/test/test_web_bundle.py
git commit -m "feat: Harmonic WebSocket bridge 추가"
```

### Task 2: Minimal gzweb monitoring frontend

**Files:**
- Create: `vehicle_simulator_model/ubuntu/web/package.json`
- Create: `vehicle_simulator_model/ubuntu/web/package-lock.json`
- Create: `vehicle_simulator_model/ubuntu/web/index.html`
- Create: `vehicle_simulator_model/ubuntu/web/src/config.ts`
- Create: `vehicle_simulator_model/ubuntu/web/src/config.test.ts`
- Create: `vehicle_simulator_model/ubuntu/web/src/main.ts`
- Create: `vehicle_simulator_model/ubuntu/web/src/style.css`
- Create: `vehicle_simulator_model/ubuntu/web/tsconfig.json`
- Create: `vehicle_simulator_model/ubuntu/web/vite.config.ts`

**Interfaces:**
- Produces: `resolveWebSocketUrl(origin, override) -> string`
- Produces: full-window `#gz-scene` monitor connected to same-origin `/gzws`

- [ ] **Step 1: Scaffold exact pinned dependencies**

Use Node 24 and:

```json
{
  "scripts": {
    "build": "tsc --noEmit && vite build",
    "test": "vitest run"
  },
  "dependencies": {
    "gzweb": "3.0.1"
  },
  "devDependencies": {
    "typescript": "5.9.3",
    "vite": "7.3.1",
    "vitest": "3.2.4"
  }
}
```

Run `npm install --package-lock-only` with Node 24 to create the lockfile.

- [ ] **Step 2: Write the failing URL tests**

```typescript
import { describe, expect, it } from "vitest";
import { resolveWebSocketUrl } from "./config";

describe("resolveWebSocketUrl", () => {
  it("uses secure same-origin websocket behind HTTPS", () => {
    expect(resolveWebSocketUrl({ protocol: "https:", host: "sim.example.com" }))
      .toBe("wss://sim.example.com/gzws");
  });

  it("uses ws for local HTTP", () => {
    expect(resolveWebSocketUrl({ protocol: "http:", host: "localhost:8080" }))
      .toBe("ws://localhost:8080/gzws");
  });

  it("accepts an explicit operator override", () => {
    expect(resolveWebSocketUrl(
      { protocol: "https:", host: "sim.example.com" },
      "wss://staging.example.com/gzws",
    )).toBe("wss://staging.example.com/gzws");
  });
});
```

- [ ] **Step 3: Run and verify failure**

Run:

```bash
cd vehicle_simulator_model/ubuntu/web
npm ci
npm test
```

Expected: FAIL because `config.ts` is missing.

- [ ] **Step 4: Implement URL resolution and monitor bootstrap**

`config.ts` returns a trimmed explicit override or `${wsScheme}://${origin.host}/gzws`.

`main.ts` creates:

```typescript
const manager = new SceneManager({
  elementId: "gz-scene",
  websocketUrl: resolveWebSocketUrl(window.location, params.get("ws") ?? undefined),
});

manager.getConnectionStatusAsObservable().subscribe((ready) => {
  status.textContent = ready ? "connected" : "connecting";
});

window.addEventListener("resize", () => manager.resize());
window.addEventListener("beforeunload", () => manager.destroy());
```

The page contains only a connection badge and full-window scene. It has no model editing controls.

- [ ] **Step 5: Run tests and production build**

Run:

```bash
npm test
npm run build
```

Expected: tests pass and `dist/` is generated.

- [ ] **Step 6: Commit**

```bash
git add vehicle_simulator_model/ubuntu/web
git commit -m "feat: gzweb 기반 Gazebo 모니터 화면 추가"
```

### Task 3: Caddy gateway and secured Compose exposure

**Files:**
- Create: `vehicle_simulator_model/ubuntu/web/Caddyfile`
- Create: `vehicle_simulator_model/ubuntu/web/Dockerfile`
- Create: `vehicle_simulator_model/ubuntu/web/.env.example`
- Modify: `vehicle_simulator_model/ubuntu/compose.yaml`
- Modify: `vehicle_simulator_model/ubuntu/run.sh`
- Modify: `vehicle_simulator_model/ubuntu/test/test_web_bundle.py`

**Interfaces:**
- Consumes: `WEB_ADDRESS`, `WEB_USERNAME`, `WEB_PASSWORD_HASH`
- Produces: host `${WEB_PORT:-8080}` in local mode or Caddy-managed 443 in production

- [ ] **Step 1: Add failing gateway security assertions**

Assert:

```python
caddy = (BUNDLE / 'web/Caddyfile').read_text()
compose = (BUNDLE / 'compose.yaml').read_text()
self.assertIn('basic_auth', caddy)
self.assertIn('reverse_proxy gazebo-web-bridge:9002', caddy)
self.assertIn('handle_path /gzws*', caddy)
self.assertIn('web-gateway:', compose)
self.assertIn('${WEB_PORT:-8080}:8080', compose)
self.assertNotIn('10317:', compose)
self.assertNotIn('10318:', compose)
```

- [ ] **Step 2: Run and verify failure**

Run:

```bash
python3 -m unittest vehicle_simulator_model/ubuntu/test/test_web_bundle.py -v
```

Expected: FAIL because gateway assets are missing.

- [ ] **Step 3: Implement multi-stage frontend image**

```dockerfile
FROM node:24-alpine AS build
WORKDIR /app
COPY package.json package-lock.json ./
RUN npm ci
COPY . .
RUN npm run build

FROM caddy:2.10-alpine
COPY Caddyfile /etc/caddy/Caddyfile
COPY --from=build /app/dist /srv
```

- [ ] **Step 4: Configure Caddy**

Use an explicit local listener and production-overridable address:

```caddyfile
{$WEB_ADDRESS:http://:8080} {
  basic_auth {
    {$WEB_USERNAME:mentorpi} {$WEB_PASSWORD_HASH}
  }

  handle_path /gzws* {
    reverse_proxy gazebo-web-bridge:9002
  }

  handle {
    root * /srv
    try_files {path} /index.html
    file_server
  }
}
```

`run.sh web-up` must reject an empty `WEB_PASSWORD_HASH`. The hash is generated with `caddy hash-password` and stored only in the server environment.

- [ ] **Step 5: Add gateway service and commands**

The gateway depends on the bridge but Gazebo/SLAM do not depend on gateway. Add `web-up`, `web-down`, and `web-logs`. `web-up` starts `gazebo-server`, `sim-adapter`, `gazebo-web-bridge`, `web-gateway`.

- [ ] **Step 6: Run static tests and config validation**

Run:

```bash
WEB_PASSWORD_HASH='$2a$14$test-only-hash' \
  docker compose -f vehicle_simulator_model/ubuntu/compose.yaml config --quiet
python3 -m unittest \
  vehicle_simulator_model/ubuntu/test/test_bundle.py \
  vehicle_simulator_model/ubuntu/test/test_web_bundle.py -v
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add vehicle_simulator_model/ubuntu/web \
        vehicle_simulator_model/ubuntu/compose.yaml \
        vehicle_simulator_model/ubuntu/run.sh \
        vehicle_simulator_model/ubuntu/test/test_web_bundle.py
git commit -m "feat: 인증된 Gazebo 웹 gateway 추가"
```

### Task 4: Browser monitoring integration and documentation

**Files:**
- Create: `vehicle_simulator_model/ubuntu/web/e2e/monitor.spec.ts`
- Create: `vehicle_simulator_model/ubuntu/web/playwright.config.ts`
- Modify: `vehicle_simulator_model/ubuntu/web/package.json`
- Modify: `vehicle_simulator_model/ubuntu/README.md`
- Modify: `vehicle_simulator_model/ubuntu/test/test_web_bundle.py`

**Interfaces:**
- Verifies: authenticated page, WSS connection, canvas render, robot scene updates

- [ ] **Step 1: Add Playwright dependency and E2E contract**

Add `@playwright/test` pinned to `1.55.1` and:

```typescript
test("renders the Gazebo scene", async ({ page }) => {
  await page.goto(process.env.MONITOR_URL ?? "http://localhost:8080", {
    waitUntil: "networkidle",
  });
  await expect(page.locator("[data-testid=connection-status]"))
    .toHaveText("connected", { timeout: 30_000 });
  await expect(page.locator("#gz-scene canvas")).toHaveCount(1);
});
```

Use `httpCredentials` from `WEB_USERNAME` and `WEB_PASSWORD` in Playwright config:

```typescript
export default defineConfig({
  use: {
    httpCredentials: {
      username: process.env.WEB_USERNAME ?? "mentorpi",
      password: process.env.WEB_PASSWORD ?? "",
    },
  },
});
```

- [ ] **Step 2: Document local and production operation**

Document password hash generation, `web-up`, browser URL, HTTPS domain setup, logs, WebSocket health, and the explicit limitation that model changes happen on the development PC.

- [ ] **Step 3: Build and start the full web profile**

Run:

```bash
cd vehicle_simulator_model/ubuntu
export WEB_USERNAME=mentorpi
export WEB_PASSWORD='local-monitor-password'
export WEB_PASSWORD_HASH="$(docker run --rm caddy:2.10-alpine caddy hash-password --plaintext "$WEB_PASSWORD")"
./run.sh web-up
```

Expected: four services run; only gateway is host-published.

- [ ] **Step 4: Verify network boundary**

Run:

```bash
docker compose ps
curl -u "$WEB_USERNAME:$WEB_PASSWORD" -fsS http://localhost:8080/ >/dev/null
! curl -fsS http://localhost:9002/
```

Expected: authenticated gateway succeeds and host port 9002 is unreachable.

- [ ] **Step 5: Run browser E2E**

Run:

```bash
cd web
MONITOR_URL=http://localhost:8080 \
WEB_USERNAME="$WEB_USERNAME" \
WEB_PASSWORD="$WEB_PASSWORD" \
npx playwright test e2e/monitor.spec.ts
```

Expected: connected badge and one rendered canvas.

- [ ] **Step 6: Prove monitor failure isolation**

Run:

```bash
docker compose stop web-gateway gazebo-web-bridge
docker compose exec gazebo-server gz topic -e -n 1 -t /world/mentorpi_warehouse/stats
docker compose exec sim-adapter ros2 topic echo --once /robot_1/odom
```

Expected: Gazebo and adapter continue producing data.

- [ ] **Step 7: Commit**

```bash
git add vehicle_simulator_model/ubuntu/web \
        vehicle_simulator_model/ubuntu/README.md \
        vehicle_simulator_model/ubuntu/test/test_web_bundle.py
git commit -m "test: Gazebo 웹 모니터 통합 검증 추가"
```
