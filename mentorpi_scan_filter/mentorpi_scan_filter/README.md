# MentorPi Scan Filter

이 독립 ROS 2 패키지는 포크 자기반사 구간 -24도부터 +22도를 inf로
바꾸고 /scan_filtered 토픽으로 발행한다. ros2_ws-main과
sesac_logistics_local의 원본 파일은 수정하지 않는다.

## MentorPi에 배치하고 빌드

    mkdir -p ~/scan_filter_ws/src
    cp -r mentorpi_scan_filter ~/scan_filter_ws/src/
    cd ~/scan_filter_ws
    source /opt/ros/humble/setup.bash
    source ~/ros2_ws/install/setup.bash
    source <sesac_ws>/install/setup.bash
    colcon build --symlink-install
    source install/setup.bash

## 필터 단독 확인

    ros2 launch mentorpi_scan_filter scan_filter.launch.py
    ros2 topic hz /scan_filtered

RViz의 LaserScan 토픽을 /scan_filtered로 선택해 포크 반사점이
사라지는지 확인한다.

## 필터 적용 SLAM

원본 slam.launch.py는 별도로 실행하지 않는다.

    ros2 launch mentorpi_scan_filter filtered_slam.launch.py \
      robot_name:=/ master_name:=/ sim:=false enable_save:=false

지도 저장:

    cd ~/ros2_ws/src/slam/maps
    ros2 run nav2_map_server map_saver_cli \
      -f map_01 \
      --ros-args -p map_subscribe_transient_local:=true

## 필터 적용 Navigation

기존 mentorpi_safe_navigation.launch.py는 별도로 실행하지 않는다.

    ros2 launch mentorpi_scan_filter filtered_navigation.launch.py \
      robot_name:=/ master_name:=/ map:=map_01

## 연결 확인

    ros2 topic hz /scan_raw
    ros2 topic hz /scan_filtered
    ros2 node info /slam_toolbox
    ros2 node info /amcl
    ros2 node info /controller_server
    ros2 node info /planner_server

각 노드의 Subscription 목록이 scan_filtered를 가리키는지 실차에서
확인한 후 바퀴를 띄운 상태로 최초 시험한다.
