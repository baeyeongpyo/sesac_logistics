# MentorPi Ubuntu Docker Bundle

이 폴더 하나만 Ubuntu 서버에 `scp`하면 실행할 수 있는 MentorPi Gazebo Sim
번들이다. Ubuntu GPU에서 EGL로 렌더링하고 VirtualGL이 프레임을 SSH X11으로
전송한다. 따라서 Ubuntu는 서버 전용 OS여도 되며, Ubuntu에 로컬 GUI/VNC/X11
세션을 만들 필요가 없다. `ros2_ws` 소스와 Docker 구성은 모두 이 폴더 안에 있고,
빌드 산출물(`build`, `install`, `log`)은 포함하지 않는다.

## Ubuntu 서버 전제 조건

- x86_64 Ubuntu, NVIDIA/Intel/AMD GPU와 `/dev/dri/renderD128`
- 실행 중인 Docker Engine과 Docker Compose v2
- Docker 명령 권한이 있는 사용자
- SSH 서버의 X11 forwarding 허용 설정 (`X11Forwarding yes`)

서버에서 다음으로 GPU 장치와 Docker를 확인한다.

```bash
test -e /dev/dri/renderD128 && echo 'DRI GPU node found'
docker version
docker compose version
```

## Mac에서 전송하고 SSH X11으로 접속

Mac에는 XQuartz가 실행 중이어야 한다. XQuartz 설정에서 **Allow connections from
network clients**를 켠 뒤 XQuartz를 재시작한다.

개발 머신의 저장소 루트에서 번들을 전송한다.

```bash
scp -r deploy/ubuntu <ubuntu-user>@<ubuntu-host>:~/mentorpi-ubuntu
```

GUI를 보려면 일반 `ssh` 대신 반드시 trusted X11 forwarding인 `ssh -Y`로 접속한다.

```bash
ssh -Y <ubuntu-user>@<ubuntu-host>
cd ~/mentorpi-ubuntu
echo "$DISPLAY"
test -r "$XAUTHORITY" && echo 'Xauthority cookie found'
```

`DISPLAY`는 보통 `localhost:10.0`처럼 표시된다. 이는 Ubuntu의 SSH X11 프록시이며,
Gazebo의 OpenGL 렌더링 위치가 Mac으로 바뀌는 것은 아니다.

## 빌드와 headless 실행

처음 한 번 이미지를 빌드하고 ROS 패키지를 확인한다.

```bash
./run.sh build
./run.sh test
```

GUI 없이 두 로봇 시뮬레이션을 실행한다.

```bash
./run.sh headless
```

중지는 실행 터미널에서 `Ctrl+C`다. 다른 SSH 터미널에서 포크를 올리려면:

```bash
cd ~/mentorpi-ubuntu
./run.sh fork-up headless
```

## Mac에 Gazebo GUI 표시

앞 절차의 `ssh -Y` 세션에서 다음만 실행한다.

```bash
cd ~/mentorpi-ubuntu
./run.sh gui
```

`run.sh gui`는 SSH가 설정한 `DISPLAY`와 `XAUTHORITY` 쿠키를 컨테이너에 read-only로
전달한다. 컨테이너는 `/dev/dri/renderD128`에서 VirtualGL의
`vglrun -d egl -c proxy`로 Gazebo Sim을 실행한다. 즉 GPU 렌더링은 Ubuntu에서 끝나고
완성된 화면만 SSH X11을 통해 Mac XQuartz에 나타난다. `xhost +`,
`host.docker.internal`, `/tmp/.X11-unix` 마운트는 이 방식에서 필요하지 않다.
Gazebo 서버·GUI·브리지는 모두 같은 컨테이너에 있으므로 `IGN_IP=127.0.0.1`로
고정되어 있다. 이는 Docker 호스트 NIC를 Gazebo Transport가 잘못 선택해 GUI가 흰
화면에서 멈추는 문제를 막는다.

GUI가 실행된 상태에서 같은 ROS 네트워크로 포크를 올리려면 별도 SSH 터미널에서:

```bash
cd ~/mentorpi-ubuntu
./run.sh fork-up gui
```

## 문제 확인

`DISPLAY is empty`이면 `ssh -Y`가 아닌 세션이다. Mac에서 `ssh -Y`로 다시 접속한다.
`Xauthority is not readable`이면 해당 SSH 세션에서 `echo "$XAUTHORITY"`와
`xauth list`를 확인한다. GUI 로그에 `requesting list of world names`가 반복되면
배포 폴더의 `compose.yaml`에 `IGN_IP: 127.0.0.1`이 있는지 확인하고 재실행한다.
`/dev/dri/renderD128` 오류는 Ubuntu 서버 GPU/DRI 설정 문제다.

## 소스 수정과 재전송

이 폴더가 MentorPi 실행 자산의 단일 원본이다. ROS 패키지는
`ros2_ws/src`에서 직접 수정하고, Docker 구성도 이 폴더의 `Dockerfile`,
`compose.yaml`, `run.sh`만 수정한다. 별도의 저장소 루트 워크스페이스나 동기화
스크립트는 없다.

수정 후에는 이 폴더 전체를 다시 전송하고 Ubuntu에서 이미지를 재빌드한다.

```bash
scp -r deploy/ubuntu <ubuntu-user>@<ubuntu-host>:~/mentorpi-ubuntu
ssh -Y <ubuntu-user>@<ubuntu-host>
cd ~/mentorpi-ubuntu
./run.sh build
```

기존 서버 폴더를 갱신할 때는 `rsync -av --delete deploy/ubuntu/` 방식을 권장한다.
