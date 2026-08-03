# fork_control

`/fork/command` 토픽으로 메시지 `UP`·`DOWN`·`STOP` 명령을 받아 포크 모터를 제어하는 `ament_python` 패키지

## GPIO 핀 설정

| 역할 | BCM GPIO | `/dev/gpiochip0` line | 코드 설정 |
|---|---:|---:|---|
| 포크 UP 출력 | 17 | 17 | `Motor(forward=17)` |
| 포크 DOWN 출력 | 18 | 18 | `Motor(backward=18)` |
| 하단 리미트 입력 | 27 | 27 | `Button(27, pull_up=False)` |
| 상단 리미트 입력 | 22 | 22 | `Button(22, pull_up=False)` |

## Raspberry Pi 5 `gpiochip` 이슈

### 확인된 증상
ROS 2 Jazzy Docker 컨테이너에서 실제 GPIO 장치는 `/dev/gpiochip0`이었지만, 당시 설치된 gpiozero의 `LGPIOFactory`가 Raspberry Pi 5를 감지하는 경우, 생성자 인자를 덮어쓰고 존재하지 않는 `gpiochip4`를 선택하는 버그가 있었음

```text
gpiozero requested gpiochip4
factory FAILED: error: 'can not open gpiochip'
```

따라서 아래 파일을 수정하여 사용해야함
대상 파일:

```text
/usr/lib/python3/dist-packages/gpiozero/pins/lgpio.py
```

수정 전:

```python
chip = 4 if (self._get_revision() & 0xff0) >> 4 == 0x17 else 0
```

수정 후:

```python
chip = 0
```

## 컨테이너 GPIO 권한

컨테이너 생성시 `--privileged`로 라즈베리파이 장치 노출하여야 GPIO에 접근할 수 있음
root 유저가 아닌 일반 유저의 경우, 
`/dev/gpiochip0`에 접근할 수 있도록, 호스트와 동일한 GID의의 `gpiohost` 그룹을 컨테이너에 만들고, 사용자를 추가하여야 함
새 셸로 다시 접속해야 반영됨

확인 명령:

```bash
id
ls -l /dev/gpiochip*
test -r /dev/gpiochip0 && echo READABLE || echo NOT_READABLE
gpioinfo gpiochip0
```


