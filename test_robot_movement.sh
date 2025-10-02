#!/bin/bash

echo "========================================="
echo "로봇 이동 테스트"
echo "========================================="

# 환경 설정
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Gazebo 시뮬레이션 시작..."
ros2 launch tetra_gazebo_sim gazebo_sim.launch.py > /tmp/robot_move_test.log 2>&1 &
GAZEBO_PID=$!
echo "   PID: $GAZEBO_PID"

echo "   20초 대기 중 (초기화)..."
sleep 20

echo ""
echo "2. 초기 위치 확인..."
INITIAL_ODOM=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep "position:" -A 3)
echo "$INITIAL_ODOM"

echo ""
echo "3. 로봇에 전진 명령 전송 (0.3 m/s, 5초)..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10 > /dev/null 2>&1 &
CMD_PID=$!

sleep 5

echo ""
echo "4. 명령 중지..."
kill $CMD_PID 2>/dev/null
sleep 2

echo ""
echo "5. 최종 위치 확인..."
FINAL_ODOM=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep "position:" -A 3)
echo "$FINAL_ODOM"

echo ""
echo "6. 회전 테스트 (0.5 rad/s, 3초)..."
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --rate 10 > /dev/null 2>&1 &
CMD_PID=$!

sleep 3

kill $CMD_PID 2>/dev/null
sleep 1

echo ""
echo "7. Gazebo 종료..."
kill -2 $GAZEBO_PID 2>/dev/null
sleep 3
if ps -p $GAZEBO_PID > /dev/null 2>&1; then
    kill -9 $GAZEBO_PID 2>/dev/null
fi

echo "   ✅ 완료"
echo "========================================="
echo ""
echo "로그 파일: /tmp/robot_move_test.log"
