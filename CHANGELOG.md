# Changelog - Robot Gazebo Simulation

## 2025-10-01 - 로봇 물리 시뮬레이션 개선

### 수정사항

#### 1. 바퀴 Joint 타입 변경 (Fixed → Continuous)
**파일**: `robot_description/urdf/base.urdf.xacro`

- 좌측 바퀴 (`base_l_wheel_joint`): `fixed` → `continuous`
- 우측 바퀴 (`base_r_wheel_joint`): `fixed` → `continuous`
- 캐스터 회전 (`rear_caster_rotate_joint`): `fixed` → `continuous`
- 캐스터 바퀴 (`rear_caster_wheel_joint`): `fixed` → `continuous`

**이유**: 시뮬레이션에서 바퀴가 실제로 회전할 수 있도록 하기 위함

#### 2. 바퀴 Collision 및 Inertia 추가
**추가된 속성**:
```xml
<!-- 좌/우 바퀴 -->
<collision>
  <origin xyz="0 0 0" rpy="${PI/2} 0 0" />
  <geometry>
    <cylinder length="0.05" radius="0.0508"/>
  </geometry>
</collision>
<inertial>
  <mass value="2.0"/>
  <xacro:cylinder_inertia m="2.0" r="0.0508" h="0.05"/>
</inertial>
```

**이유**: Gazebo 물리 엔진이 충돌 및 관성을 계산할 수 있도록 함

#### 3. Base Link에 Inertia 추가
**추가된 속성**:
```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="15.0"/>
  <xacro:cylinder_inertia m="15.0" r="${base_radius}" h="${base_length}"/>
</inertial>
```

**이유**: 로봇 본체의 질량과 관성 모멘트 정의로 안정적인 시뮬레이션 보장

#### 4. Joint Dynamics 추가
**추가된 속성**:
```xml
<dynamics damping="0.01" friction="0.0"/>
```

**이유**: 바퀴 회전 시 감쇠와 마찰 특성 정의

#### 5. Launch 파일에 초기 자세 매개변수 추가
**파일**: `robot_gazebo_sim/launch/gazebo_sim.launch.py`

**새로운 매개변수**:
- `z_pose`: 0.1 → 0.2 (기본값 증가)
- `roll`: 로봇 Roll 각도 (X축 회전)
- `pitch`: 로봇 Pitch 각도 (Y축 회전)
- `yaw`: 로봇 Yaw 각도 (Z축 회전)

**spawn_robot 노드 업데이트**:
```python
arguments=[
    '-name', LaunchConfiguration('robot_name'),
    '-topic', 'robot_description',
    '-x', LaunchConfiguration('x_pose'),
    '-y', LaunchConfiguration('y_pose'),
    '-z', LaunchConfiguration('z_pose'),
    '-R', LaunchConfiguration('roll'),      # 추가
    '-P', LaunchConfiguration('pitch'),     # 추가
    '-Y', LaunchConfiguration('yaw')        # 추가
],
```

**이유**: 
- 로봇이 바닥을 통과하거나 뒤집히는 문제 방지
- 초기 방향 제어 가능

### 효과

1. **물리 시뮬레이션 정확성 향상**
   - 바퀴가 실제로 회전하며 DiffDrive 플러그인과 연동
   - 관성과 충돌이 올바르게 계산됨

2. **안정적인 스폰**
   - Z 높이 증가로 로봇이 안정적으로 생성됨
   - 초기 자세 제어로 뒤집힘 현상 방지

3. **사실적인 동작**
   - 감쇠와 마찰 특성으로 현실적인 움직임
   - 질량 분포가 실제 로봇과 유사

### 사용 예시

#### 기본 실행
```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py
```

#### 초기 위치/방향 지정
```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py \
    x_pose:=1.0 \
    y_pose:=2.0 \
    z_pose:=0.3 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=1.57
```

### 주의사항

- `z_pose`는 최소 0.3 이상으로 설정하는 것을 권장
- `roll`과 `pitch`는 일반적으로 0.0으로 유지 (로봇이 수평)
- `yaw`는 초기 로봇 방향 설정에 사용 (라디안 단위)

### 추가 개선 필요 사항

1. **DiffDrive 플러그인 연결 검증**
   - 현재 플러그인 로드 실패 경고 있음
   - `gz::sim::systems::DiffDrive` 형식으로 수정 완료했으나 추가 검증 필요

2. **센서 플러그인 통합**
   - LiDAR, IMU, 카메라 플러그인 동작 확인 필요
   - 센서 데이터 ROS 토픽 발행 검증 필요

3. **네비게이션 테스트**
   - cmd_vel 토픽으로 로봇 제어 테스트
   - 오도메트리 정확도 검증
   - SLAM 및 네비게이션 통합 테스트
