#!/bin/bash

echo "========================================="
echo "IMU 발행 테스트"
echo "========================================="

# 환경 설정
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

echo "1. Fake IMU Publisher 시작..."
ros2 run robot_gazebo_sim fake_imu_publisher.py > /tmp/fake_imu.log 2>&1 &
IMU_PID=$!
echo "   PID: $IMU_PID"

sleep 3

echo ""
echo "2. /imu 토픽 확인..."
if ros2 topic list | grep -q "^/imu$"; then
    echo "   ✅ /imu 토픽 존재"
else
    echo "   ❌ /imu 토픽을 찾을 수 없습니다"
fi

echo ""
echo "3. IMU 데이터 수신 테스트..."
if timeout 3 ros2 topic echo /imu --once > /dev/null 2>&1; then
    echo "   ✅ IMU 데이터 수신 성공!"
    echo ""
    echo "   첫 번째 메시지:"
    timeout 1 ros2 topic echo /imu --once
else
    echo "   ❌ IMU 데이터 수신 실패"
fi

echo ""
echo "4. 정리 중..."
kill $IMU_PID 2>/dev/null
wait $IMU_PID 2>/dev/null
echo "   ✅ 완료"

echo "========================================="
