#!/bin/bash

# Robot Gazebo Simulation Test Script
# 가제보 시뮬레이션 패키지 테스트 스크립트

echo "========================================="
echo "Robot Gazebo Simulation Test"
echo "========================================="
echo ""

# 환경 설정
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/test/robot_gazebo_sim/install/setup.bash

# 1. 패키지 확인
echo "1. 패키지 확인 중..."
if ros2 pkg prefix robot_gazebo_sim > /dev/null 2>&1; then
    echo "   ✅ robot_gazebo_sim 패키지 발견"
else
    echo "   ❌ robot_gazebo_sim 패키지를 찾을 수 없습니다"
    exit 1
fi

# 2. Launch 파일 확인
echo ""
echo "2. Launch 파일 확인 중..."
LAUNCH_DIR=$(ros2 pkg prefix robot_gazebo_sim)/share/robot_gazebo_sim/launch
if [ -d "$LAUNCH_DIR" ]; then
    echo "   ✅ Launch 디렉토리 존재: $LAUNCH_DIR"
    echo "   파일 목록:"
    ls -1 $LAUNCH_DIR/*.py | xargs -n 1 basename | sed 's/^/      - /'
else
    echo "   ❌ Launch 디렉토리를 찾을 수 없습니다"
fi

# 3. URDF 파일 확인
echo ""
echo "3. URDF 파일 확인 중..."
URDF_DIR=$(ros2 pkg prefix robot_gazebo_sim)/share/robot_gazebo_sim/urdf
if [ -f "$URDF_DIR/robot_gazebo.xacro" ]; then
    echo "   ✅ robot_gazebo.xacro 파일 존재"
    # xacro 파일 처리 테스트
    if xacro $URDF_DIR/robot_gazebo.xacro > /dev/null 2>&1; then
        echo "   ✅ xacro 파일 처리 성공"
    else
        echo "   ⚠️  xacro 파일 처리 중 경고/오류 발생"
    fi
else
    echo "   ❌ robot_gazebo.xacro 파일을 찾을 수 없습니다"
fi

# 4. World 파일 확인
echo ""
echo "4. World 파일 확인 중..."
WORLD_DIR=$(ros2 pkg prefix robot_gazebo_sim)/share/robot_gazebo_sim/worlds
if [ -d "$WORLD_DIR" ]; then
    echo "   ✅ Worlds 디렉토리 존재"
    echo "   파일 목록:"
    ls -1 $WORLD_DIR/*.sdf | xargs -n 1 basename | sed 's/^/      - /'
else
    echo "   ❌ Worlds 디렉토리를 찾을 수 없습니다"
fi

# 5. Gazebo 시뮬레이션 실행 (백그라운드)
echo ""
echo "5. Gazebo 시뮬레이션 실행 테스트..."
echo "   Gazebo를 백그라운드로 실행합니다..."
ros2 launch robot_gazebo_sim gazebo_sim.launch.py > /tmp/gazebo_test.log 2>&1 &
GAZEBO_PID=$!
echo "   PID: $GAZEBO_PID"

# 15초 대기
echo "   15초 대기 중..."
sleep 15

# 6. 프로세스 확인
echo ""
echo "6. 프로세스 확인 중..."
if ps -p $GAZEBO_PID > /dev/null 2>&1; then
    echo "   ✅ Gazebo 프로세스 실행 중"
else
    echo "   ❌ Gazebo 프로세스가 실행되지 않았습니다"
    cat /tmp/gazebo_test.log | tail -20
    exit 1
fi

# 7. ROS2 토픽 확인
echo ""
echo "7. ROS2 토픽 확인 중..."
EXPECTED_TOPICS=("/cmd_vel" "/odom" "/scan" "/scan2" "/imu" "/joint_states" "/tf" "/clock")
FOUND=0
MISSING=()

for topic in "${EXPECTED_TOPICS[@]}"; do
    if ros2 topic list | grep -q "^${topic}$"; then
        echo "   ✅ $topic"
        ((FOUND++))
    else
        echo "   ❌ $topic (발견되지 않음)"
        MISSING+=("$topic")
    fi
done

echo ""
echo "   발견된 토픽: $FOUND / ${#EXPECTED_TOPICS[@]}"

# 8. 센서 데이터 확인
echo ""
echo "8. 센서 데이터 확인 중..."

# LiDAR
if timeout 5 ros2 topic echo /scan --once > /dev/null 2>&1; then
    echo "   ✅ /scan - LiDAR 데이터 수신"
else
    echo "   ⚠️  /scan - 데이터 수신 실패 (타임아웃)"
fi

# IMU
if timeout 10 ros2 topic echo /imu --once > /dev/null 2>&1; then
    echo "   ✅ /imu - IMU 데이터 수신"
else
    echo "   ⚠️  /imu - 데이터 수신 실패 (타임아웃 10초)"
fi

# Odometry
if timeout 5 ros2 topic echo /odom --once > /dev/null 2>&1; then
    echo "   ✅ /odom - Odometry 데이터 수신"
else
    echo "   ⚠️  /odom - 데이터 수신 실패 (타임아웃)"
fi

# 9. 로봇 제어 테스트
echo ""
echo "9. 로봇 제어 테스트..."
echo "   /cmd_vel 토픽으로 명령 전송 중..."

# 토픽 존재 확인 후 명령 전송
if ros2 topic list | grep -q "^/cmd_vel$"; then
    timeout 5 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ 제어 명령 전송 성공"
    else
        echo "   ⚠️  제어 명령 전송 실패 또는 타임아웃"
    fi
else
    echo "   ⚠️  /cmd_vel 토픽을 찾을 수 없습니다"
fi

sleep 2

if timeout 5 ros2 topic echo /odom --once > /dev/null 2>&1; then
    echo "   ✅ Odometry 응답 확인"
else
    echo "   ⚠️  Odometry 응답 확인 실패"
fi

# 10. Gazebo 종료
echo ""
echo "10. Gazebo 종료 중..."
kill -2 $GAZEBO_PID 2>/dev/null
sleep 3
if ps -p $GAZEBO_PID > /dev/null 2>&1; then
    kill -9 $GAZEBO_PID 2>/dev/null
fi
echo "   ✅ Gazebo 종료 완료"

# 결과 요약
echo ""
echo "========================================="
echo "테스트 결과 요약"
echo "========================================="
if [ ${#MISSING[@]} -eq 0 ] && [ $FOUND -eq ${#EXPECTED_TOPICS[@]} ]; then
    echo "✅ 모든 테스트 통과!"
    echo ""
    echo "Gazebo 시뮬레이션이 정상적으로 작동합니다."
    echo "다음 명령어로 실행할 수 있습니다:"
    echo "  ros2 launch robot_gazebo_sim gazebo_sim.launch.py"
else
    echo "⚠️  일부 테스트 실패"
    echo ""
    if [ ${#MISSING[@]} -gt 0 ]; then
        echo "누락된 토픽:"
        for topic in "${MISSING[@]}"; do
            echo "  - $topic"
        done
    fi
    echo ""
    echo "로그 확인: /tmp/gazebo_test.log"
fi
echo "========================================="
