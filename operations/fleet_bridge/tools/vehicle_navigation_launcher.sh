#!/usr/bin/env bash

LOG_DIR="$HOME/log"
LOG_FILE="$LOG_DIR/navigation.log"
PID_FILE="$LOG_DIR/navigation.pid"

mkdir -p "$LOG_DIR"

# 필요하면 ROS 환경 설정을 추가하세요.
# source /opt/ros/humble/setup.bash
# source "$HOME/mentorpi_ws/install/setup.bash"

get_running_pid() {
    [[ -f "$PID_FILE" ]] || return 1

    local pid
    read -r pid < "$PID_FILE"

    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    kill -0 "$pid" 2>/dev/null || return 1

    printf '%s\n' "$pid"
}

start() {
    local pid

    if pid="$(get_running_pid)"; then
        echo "Navigation이 이미 실행 중입니다. PID=$pid"
        return 0
    fi

    # 오래된 PID 파일 제거
    rm -f -- "$PID_FILE"

    nohup ros2 launch mentorpi_scan_filter filtered_navigation.launch.py \
        robot_name:=/ \
        master_name:=/ \
        sim:=false \
        map:=map_0821 \
        >>"$LOG_FILE" 2>&1 < /dev/null &

    pid=$!
    printf '%s\n' "$pid" > "$PID_FILE"

    sleep 1

    if kill -0 "$pid" 2>/dev/null; then
        echo "Navigation을 시작했습니다. PID=$pid"
        echo "로그: $LOG_FILE"
    else
        rm -f -- "$PID_FILE"
        echo "Navigation 실행에 실패했습니다." >&2
        echo "로그를 확인하세요: $LOG_FILE" >&2
        return 1
    fi
}

stop() {
    local pid

    if ! pid="$(get_running_pid)"; then
        rm -f -- "$PID_FILE"
        echo "Navigation이 실행 중이 아닙니다."
        return 0
    fi

    echo "Navigation을 종료합니다. PID=$pid"

    # ROS 2 launch에 Ctrl+C와 같은 SIGINT 전달
    kill -INT "$pid"

    for ((count = 0; count < 10; count++)); do
        if ! kill -0 "$pid" 2>/dev/null; then
            rm -f -- "$PID_FILE"
            echo "Navigation을 종료했습니다."
            return 0
        fi

        sleep 1
    done

    echo "10초 안에 종료되지 않았습니다. PID=$pid" >&2
    return 1
}

status() {
    local pid

    if pid="$(get_running_pid)"; then
        echo "Navigation이 실행 중입니다."
        ps -p "$pid" -o pid=,ppid=,stat=,etime=,command=
        return 0
    fi

    if [[ -f "$PID_FILE" ]]; then
        echo "오래된 PID 파일을 삭제합니다."
        rm -f -- "$PID_FILE"
    fi

    echo "Navigation이 실행 중이 아닙니다."
    return 3
}

case "${1:-status}" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    restart)
        stop && start
        ;;
    status)
        status
        ;;
    *)
        echo "사용법: $0 {start|stop|restart|status}"
        exit 2
        ;;
esac