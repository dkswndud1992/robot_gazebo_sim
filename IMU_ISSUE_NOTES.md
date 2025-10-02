# IMU 센서 문제 해결 노트

## 현재 상황
- IMU 토픽 (`/imu`)은 존재하지만 데이터가 발행되지 않음
- Gazebo Harmonic (Gz Sim 8)에서 IMU 센서 엔티티가 생성되지 않음
- 브리지는 정상 작동

## 시도한 해결책
1. ✅ IMU 센서 설정 최적화 (`always_on`, `visualize`)
2. ✅ imu_link에 물리적 속성 추가 (inertial, collision)
3. ✅ IMU를 base_link로 이동
4. ❌ 여전히 센서 생성되지 않음

## 대체 솔루션

### 옵션 1: IMU 플러그인 명시적 추가 (권장)
Gazebo 플러그인을 사용하여 IMU 시뮬레이션:

```xml
<gazebo>
  <plugin filename="libgz-sim-imu-system.so" name="gz::sim::systems::Imu">
    <topic>imu</topic>
    <frame_id>base_link</frame_id>
    <update_rate>100</update_rate>
  </plugin>
</gazebo>
```

### 옵션 2: robot_localization 사용
IMU 없이 wheel odometry와 LiDAR만으로 로봇 위치 추정:
- `/odom` 토픽 (이미 정상 작동 ✅)
- `/scan` 토픽 (이미 정상 작동 ✅)
- AMCL 또는 Cartographer 사용

### 옵션 3: Gazebo Classic 사용
Gazebo Classic (Gazebo 11)에서는 IMU 센서가 안정적으로 작동

## Navigation2에 미치는 영향

**좋은 소식:** IMU 없이도 Navigation2는 정상 작동합니다!

Navigation2에 필수적인 센서:
- ✅ LiDAR (`/scan`) - 정상 작동
- ✅ Odometry (`/odom`) - 정상 작동
- ⚠️ IMU (`/imu`) - 선택사항 (정확도 향상용)

대부분의 실내 로봇은 IMU 없이도 잘 작동하며, 특히:
- 평평한 지면
- 안정적인 wheel odometry
- 양질의 LiDAR 데이터

가 있으면 충분합니다.

## 권장 조치

**현재 상태로 진행하세요!**

1. IMU는 "nice-to-have"이지 필수가 아닙니다
2. 모든 핵심 기능이 작동 중:
   - ✅ 로봇 제어 (`/cmd_vel`)
   - ✅ Odometry (`/odom`)
   - ✅ LiDAR (`/scan`, `/scan2`)
   - ✅ Joint states
   - ✅ TF transforms

3. Navigation2와 SLAM (Cartographer)을 바로 테스트할 수 있습니다

4. 필요시 나중에 IMU 문제를 해결할 수 있습니다

## 참고 자료
- Gazebo Harmonic IMU 이슈: https://github.com/gazebosim/gz-sim/issues
- Navigation2 센서 요구사항: https://navigation.ros.org/setup_guides/sensors/setup_sensors.html
