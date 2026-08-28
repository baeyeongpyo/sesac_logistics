# Foxglove 사내망 직접 접속 설계

## 목적

원격 Linux 서버의 headless Gazebo 시뮬레이션을 개발 PC의 로컬 Foxglove
Studio에서 관찰하고 제어한다. 이 단계에서는 HTTP reverse proxy나 브라우저
viewer를 사용하지 않는다.

## 범위

- Foxglove Bridge를 browser viewer/Caddy Compose 구성에서 분리한다.
- `SIM_NETWORK_MODE=lan` 서버 profile에서 Bridge가 ROS 2 DDS discovery에
  연결되고 TCP 8765을 사내망에 직접 제공하게 한다.
- `sim-up`, Bridge 시작·중지·로그 조회 명령과 `.env.server.example`, README,
  자동 검증을 이 실행 구조에 맞춘다.
- Caddy, noVNC, X11 기반 browser viewer 및 public viewer profile은 이번
  실행 경로에서 제거한다. 외부 공개용 TLS·인증 reverse proxy는 후속 작업이다.

## 실행 구조

```text
개발 PC Foxglove Studio -- ws://server-lan-ip:8765 --> Foxglove Bridge
                                                        |
                                                     ROS 2 DDS
                                                        |
                                                sim-adapter / Gazebo headless
```

LAN profile의 Bridge는 server host network에서 실행해 host-network의
`dds-discovery` 및 `sim-adapter`와 같은 DDS 통신 경로를 사용한다. Bridge
포트는 사내망에서 직접 접근 가능하며, host firewall은 신뢰된 개발자 CIDR만
8765/TCP에 허용해야 한다. Gazebo Transport와 DDS discovery 포트는 개발 PC에
공개하지 않는다.

## 오류 처리와 검증

- LAN profile에 `GZ_SERVER_IP`가 없으면 기존처럼 Docker 실행 전에 실패한다.
- Bridge는 `sim-adapter` healthcheck 이후에 시작한다.
- Compose 렌더링 검증은 LAN에서 Bridge가 host network, loopback DDS
  discovery, 8765 공개 상태인지 확인한다.
- launcher 검증은 `sim-up`이 Bridge를 함께 시작하고 `foxglove-down` 및
  `foxglove-logs`가 Bridge만 대상으로 하는지 확인한다.
- browser viewer/Caddy 파일과 실행 명령이 headless 운영 문서 및 launcher에
  남지 않는지 확인한다.
