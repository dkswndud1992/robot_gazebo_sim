# Autonomous Robot Gazebo Simulation Package

이 패키지는 자율주행 로봇의 Gazebo Harmonic 시뮬레이션을 위한 통합 패키지입니다.

ROS2 Jazzy + Gazebo Harmonic 환경에서 자율주행 로봇의 완전한 시뮬레이션 환경을 제공합니다.

## 패키지 구성

```
robot_gazebo_sim/
├── launch/                    # Launch 파일들
│   ├── gazebo_sim.launch.py      # 기본 Gazebo 시뮬레이### 5. "package 'ros_gz_sim' not found" 오류
```bash
# ROS2 Gazebo 패키지 재설치
sudo apt install ros-jazz### 4. 로봇 제어 테스트
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# 5. Gazebo에서 시뮬레이션이 실행 중인지 확인
# - GUI 왼쪽 하단의 재생 버튼 상태 확인
# - RTF(Real Time Factor) 값이 0 이상인지 확인
```

## 주요 특징-gz-sim ros-jazzy-ros-gz-bridge
```

### 6. "package 'joint_state_publisher' not found" 오류
```bash
sudo apt install ros-jazzy-joint-state-publisher
```

### 7. 로봇이 스폰되지 않는 경우m_bringup.launch.py     # 로봇 시스템 시뮬레이션
│   ├── sim_navigation.launch.py  # 네비게이션 시뮬레이션
│   └── full_sim.launch.py        # 완전한 시뮬레이션
├── worlds/                    # Gazebo 월드 파일들
│   ├── robot_office.sdf          # 오피스 환경
│   └── empty_world.sdf           # 빈 환경
├── urdf/                      # 로봇 모델 파일들
│   └── robot_gazebo.xacro        # Gazebo용 URDF
├── params/                    # 설정 파일들
│   ├── ekf_sim.yaml              # EKF 설정
│   ├── slam_toolbox_sim.yaml     # SLAM 설정
│   ├── amcl_localization_sim.yaml # AMCL 설정
│   └── navigation_sim.yaml       # Navigation2 설정
├── rviz/                      # RViz 설정
│   └── sim_config.rviz           # 시뮬레이션용 RViz 설정
├── config/                    # 추가 설정 파일들
├── package.xml                # 패키지 정보
├── CMakeLists.txt            # 빌드 설정
└── README.md                 # 이 파일
```

## 필요한 종속성

### 시스템 요구사항
- Ubuntu 24.04 (Noble)
- ROS2 Jazzy
- Gazebo Harmonic

### 시스템 패키지 설치
```bash
# Gazebo Harmonic 및 ROS2 Gazebo 브리지
sudo apt update
sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge

# 필수 ROS2 패키지
sudo apt install ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-xacro
sudo apt install ros-jazzy-robot-state-publisher

# Navigation2 관련 패키지 (선택사항 - 네비게이션 사용 시)
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-robot-localization
sudo apt install ros-jazzy-slam-toolbox

# 제어 및 인터페이스 패키지 (선택사항)
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-teleop-twist-joy
sudo apt install ros-jazzy-rosbridge-server
sudo apt install ros-jazzy-rosapi
```

### 워크스페이스 종속성
- `robot_description` - 로봇 기본 모델
- `robot_navigation2` - 네비게이션 설정
- `robot_interface` - 로봇 인터페이스

## 빌드 방법

```bash
cd ~/ros2_ws
colcon build --packages-select robot_gazebo_sim
source install/setup.bash
```

## 사용 방법

### 환경 설정
매번 터미널을 열 때마다 실행:
```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. 기본 Gazebo 시뮬레이션 (권장)
로봇과 환경만 로드:
-```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py
```
- Gazebo GUI 창이 열리고 자율주행 로봇이 오피스 환경에 스폰됩니다
- 기본 센서 (2x LiDAR, IMU, 카메라) 데이터가 ROS 토픽으로 발행됩니다
- **중요**: Gazebo GUI 왼쪽 하단의 **재생 버튼(▶️)을 클릭**하여 시뮬레이션을 시작하세요

#### 초기 위치 및 방향 설정
로봇의 위치가 이상한 경우 (공중에 떠있거나 바닥 관통), 초기 위치를 조정하세요:
```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py \
    x_pose:=0.0 \
    y_pose:=0.0 \
    z_pose:=0.1 \
    roll:=0.0 \
    pitch:=0.0 \
    yaw:=0.0
```

**⚠️ z_pose 값 가이드:**
| z_pose 값 | 결과 | 계산 | 비고 |
|-----------|------|------|------|
| **0.055** (기본값) | ✅ 이상적 - 바닥 접촉 | 바퀴 반지름(0.05) + 여유(0.005) | **권장 값** |
| 0.05 | ✅ 정상 | 바퀴 반지름과 정확히 일치 | 최소값 |
| 0.06-0.08 | ✅ 정상 작동 | 약간의 여유 | 안전 |
| 0.1 | ⚠️ 약간 뜸 | 바닥에서 5cm 떠있음 | 너무 높음 |
| 0.15 이상 | ❌ 공중에 떠서 낙하 | 10cm 이상 떠있음 | 비정상 |

**📐 계산 방법 (예)**:
- 예: 바퀴 반지름(샘플 값): **0.05m (5cm)**
- 바닥(ground_plane): **z = 0**
- 이상적인 z_pose: **바퀴 반지름 + 작은 여유(예: 0.005m)** = **0.055m (예시)**

**다른 파라미터:**
- `x_pose`, `y_pose`: 로봇의 수평 위치 (기본: 0, 0)
- `roll`, `pitch`, `yaw`: 로봇의 회전 (기본: 0, 0, 0 = 정면)

#### 로봇 추적하기
Gazebo GUI에서 로봇 움직임을 쉽게 확인하려면:
1. 로봇을 **우클릭** → **"Follow"** 선택
2. 카메라가 자동으로 로봇을 따라갑니다

### 2. 로봇 시스템 시뮬레이션
EKF, 조이스틱, 웹 인터페이스 포함:
```bash
ros2 launch robot_gazebo_sim sim_bringup.launch.py
```

### 3. SLAM 포함 완전한 시뮬레이션
```bash
ros2 launch robot_gazebo_sim full_sim.launch.py
```
- SLAM, 네비게이션, RViz 모두 실행됩니다

### 4. 기존 맵으로 네비게이션
```bash
ros2 launch robot_gazebo_sim full_sim.launch.py slam:=false
```

### 5. 다른 월드 파일 사용
```bash
# 빈 환경 사용
ros2 launch robot_gazebo_sim gazebo_sim.launch.py world_file:=$(ros2 pkg prefix robot_gazebo_sim)/share/robot_gazebo_sim/worlds/empty_world.sdf
```

## Launch 파일 설명

### gazebo_sim.launch.py
가장 기본적인 Gazebo 시뮬레이션 launch 파일입니다.

**실행 내용:**
- Gazebo 시뮬레이터 실행
- `robot_state_publisher` 노드 (URDF → TF 변환)
- 로봇 스폰 (Gazebo에 로봇 생성)
- ROS-Gazebo 브리지 (토픽 연결)
- TF 브리지 (좌표계 변환)

**주요 파라미터:**
- `use_sim_time`: 시뮬레이션 시간 사용 (기본값: true)
- `world_file`: 사용할 월드 파일 경로
- `robot_name`: 로봇 이름 (기본값: robot)
- `x_pose`, `y_pose`, `z_pose`: 로봇 초기 위치

**참고:**
- `joint_state_publisher`는 포함되지 **않음** (Gazebo 플러그인이 이미 제공)
- Gazebo 종료 시 모든 노드가 함께 종료됨

### sim_bringup.launch.py
- `gazebo_sim.launch.py` 포함
- EKF 로컬라이제이션
- 조이스틱/키보드 제어
- rosbridge 웹 인터페이스

### sim_navigation.launch.py
- SLAM 또는 AMCL 로컬라이제이션
- Navigation2 스택
- 경로 계획 및 추적

### full_sim.launch.py
- 모든 시스템 통합 실행
- RViz 시각화 포함
- 완전한 자율 네비게이션 환경

## 제어 방법

### 키보드 제어
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 조이스틱 제어
조이스틱이 연결되면 자동으로 인식되어 제어 가능합니다.

### RViz 네비게이션
- "2D Nav Goal" 도구로 목표점 설정
- "2D Pose Estimate"로 초기 위치 설정 (AMCL 사용 시)

## 주요 토픽

### 센서 데이터
- `/scan` - 전면 라이다
- `/scan2` - 후면 라이다  
- `/imu` - IMU 센서
- `/camera/image_raw` - 카메라 이미지
- `/camera/color/image_raw` - RealSense 컬러
- `/camera/depth/image_rect_raw` - RealSense 깊이

### 제어/상태
- `/cmd_vel` - 속도 명령 (ROS2 토픽, 내부적으로 `/model/robot/cmd_vel`로 브리지됨)
- `/odom` - 오도메트리 (ROS2 토픽, 내부적으로 `/model/robot/odometry`에서 브리지됨)
- `/joint_states` - 조인트 상태
- `/tf`, `/tf_static` - 변환 정보

**참고**: Gazebo Harmonic의 DiffDrive 플러그인은 자동으로 `/model/<robot_name>/` 네임스페이스를 사용합니다. ros_gz_bridge가 자동으로 ROS2 토픽으로 리매핑합니다.

### 네비게이션
- `/map` - SLAM 맵 또는 로드된 맵
- `/plan` - 계획된 경로
- `/local_costmap/costmap` - 로컬 코스트맵
- `/global_costmap/costmap` - 글로벌 코스트맵

## 맵 저장

SLAM으로 맵을 생성한 후 저장:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

## 설정 커스터마이징

### 로봇 물리 특성 변경
`urdf/robot_gazebo.xacro` 파일에서 센서 설정이나 물리 특성을 수정할 수 있습니다.

### 월드 환경 변경
`worlds/` 디렉토리에 새로운 `.sdf` 파일을 생성하거나 기존 파일을 수정합니다.

### 네비게이션 파라미터 조정
`params/navigation_sim.yaml`에서 경로 계획, 장애물 회피 등의 설정을 조정합니다.

### EKF 센서 융합 설정
`params/ekf_sim.yaml`에서 센서 데이터 융합 설정을 변경합니다.

## 문제 해결

### 1. 로봇이 움직이지 않는 경우 (가장 흔한 문제!)
**증상**: `/cmd_vel`로 명령을 보내도 로봇이 Gazebo에서 움직이지 않음

**원인 및 해결**:
1. **시뮬레이션이 일시정지 상태** (가장 흔함)
   - Gazebo GUI 왼쪽 하단의 **재생 버튼(▶️)을 클릭**
   
2. **Odometry는 변하는데 시각적으로 안보임**
   - 로봇을 우클릭 → "Follow" 선택하여 카메라가 로봇을 따라가게 설정
   - RTF(Real Time Factor) 값이 너무 낮은지 확인 (성능 문제일 수 있음)

3. **실제로 명령이 전달되지 않음**
   ```bash
   # Odometry 데이터가 변하는지 확인
   ros2 topic echo /odom --once
   
   # 5초 간격으로 위치 변화 확인
   ros2 topic echo /odom | grep "position:" -A3
   ```
   
4. **토픽 연결 확인**
   ```bash
   # DiffDrive가 올바른 토픽을 구독하는지 확인
   gz topic -l | grep cmd_vel
   # 출력: /model/robot/cmd_vel (정상)
   
   # ROS2 브리지가 리매핑하는지 확인
   ros2 topic info /cmd_vel
   ```

### 2. 로봇이 공중에 떠 있거나 바닥을 통과하는 경우
**증상**: 
- 로봇이 공중에 떠 있다가 떨어짐
- 로봇이 바닥을 뚫고 내려감
- 시뮬레이션 시작 시 로봇이 불안정함

**원인**: `z_pose` 값이 부적절함

**해결 방법**:

1. **로봇이 공중에 떠 있는 경우** (가장 흔함)
   ```bash
   # Gazebo 종료 (Ctrl+C)
   # z_pose 값을 낮춰서 재실행
   ros2 launch robot_gazebo_sim gazebo_sim.launch.py z_pose:=0.1
   ```
   **원인**: 기본값이 너무 높거나, 명시적으로 높은 값을 입력함
   
2. **로봇이 바닥을 통과하는 경우**
   ```bash
   # z_pose 값을 약간 높여서 재실행
   ros2 launch robot_gazebo_sim gazebo_sim.launch.py z_pose:=0.12
   ```
   **원인**: z_pose 값이 너무 낮아서 충돌 감지 전에 바닥 아래로 스폰됨

**권장 z_pose 값: 0.055** (기본값, 바퀴 반지름 + 작은 여유)

**빠른 확인 방법**:
- Gazebo GUI에서 로봇을 클릭하고 "Transform" 탭에서 Z 위치 확인
- 정상: Z ≈ 0.05-0.08 (바퀴가 바닥에 접촉)
- 약간 뜸: Z = 0.1-0.15 (바닥에서 5-10cm 위)
- 비정상: Z > 0.2 (공중에 떠서 낙하) 또는 Z < 0.04 (바닥 아래/관통)

### 3. 메시(STL) 파일을 찾을 수 없는 오류
**증상**: 
- 로봇이 보이지 않거나 빨간색 박스/선으로만 표시됨
- 에러 로그: `[Err] Unable to find file with URI [model://robot_description/meshes/...]`
- 경고 로그: `[Wrn] Failed to load mesh from [model://robot_description/meshes/...]`

**원인**: 
Gazebo Harmonic이 `package://` URI를 `model://` URI로 변환하지만, `GZ_SIM_RESOURCE_PATH` 환경 변수가 올바르게 설정되지 않아 메시 파일을 찾지 못함

**해결**:
1. **패키지 재빌드** (권장):
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot_description robot_gazebo_sim
   source install/setup.bash
   ros2 launch robot_gazebo_sim gazebo_sim.launch.py
   ```

2. **환경 변수 수동 설정** (임시 해결):
   ```bash
   export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install:$GZ_SIM_RESOURCE_PATH
   ros2 launch robot_gazebo_sim gazebo_sim.launch.py
   ```

3. **메시 파일 확인**:
   ```bash
   # 메시 파일이 설치되었는지 확인
   ls ~/ros2_ws/install/robot_description/share/robot_description/meshes/
   ```

**참고**: Launch 파일이 자동으로 `GZ_SIM_RESOURCE_PATH`를 설정하므로, 정상적으로는 이 오류가 발생하지 않아야 합니다. 이 오류가 계속되면 패키지를 재빌드하세요.

### 4. Gazebo 실행 오류
```bash
# 환경 변수 확인 및 재설정
export GZ_VERSION=harmonic
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 5. "package 'ros_gz_sim' not found" 오류
```bash
# ROS2 Gazebo 패키지 재설치
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

### 3. "package 'joint_state_publisher' not found" 오류
```bash
sudo apt install ros-jazzy-joint-state-publisher
```

### 4. 로봇이 스폰되지 않는 경우
- `robot_state_publisher`가 정상 작동하는지 확인:
```bash
ros2 topic echo /robot_description --once
```
- 로그 확인:
```bash
cat ~/.ros/log/latest/*/stdout.log
```

### 5. 브리지 연결 문제
토픽이 전달되지 않는 경우:
```bash
# 브리지 상태 확인
ros2 topic list | grep -E "cmd_vel|odom|scan|imu"

# 수동 브리지 테스트
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist
```

### 6. xacro 파일 처리 오류
```bash
# xacro 수동 실행으로 URDF 확인
cd ~/ros2_ws
xacro install/robot_gazebo_sim/share/robot_gazebo_sim/urdf/robot_gazebo.xacro
```

### 7. 빌드 오류
```bash
# 클린 빌드
cd ~/ros2_ws
rm -rf build/ install/ log/
colcon build --packages-select interfaces robot_interface robot_gazebo_sim
```

### 8. 성능 최적화
시뮬레이션이 느린 경우:
```bash
# GUI 없이 헤드리스 모드 실행
export GZ_SIM_RESOURCE_PATH=~/ros2_ws/install/robot_gazebo_sim/share/robot_gazebo_sim
gz sim -s -r worlds/robot_office.sdf
```
- 센서 업데이트 주기 감소 (urdf/robot_gazebo.xacro 수정)
- 파티클 수 조정 (AMCL 사용 시)
- 물리 엔진 업데이트 주기 조정

## 개발 및 확장

### 새로운 센서 추가
1. `urdf/robot_gazebo.xacro`에 센서 정의 추가
2. `launch/gazebo_sim.launch.py`에 브리지 토픽 추가
3. 필요한 경우 파라미터 파일 업데이트

### 새로운 월드 생성
1. `worlds/` 디렉토리에 `.sdf` 파일 생성
2. Launch 파일에서 `world_file` 파라미터로 지정

### 추가 네비게이션 플러그인
1. `params/navigation_sim.yaml`에 플러그인 설정 추가
2. 필요한 의존성을 `package.xml`에 추가

## 실행 확인

시뮬레이션이 정상적으로 실행되었는지 확인:

```bash
# 1. Gazebo가 실행 중인지 확인
ps aux | grep gz

# 2. ROS2 토픽 확인
ros2 topic list

# 3. 센서 데이터 확인
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic echo /odom --once

# 4. 로봇 제어 테스트
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

## 주요 특징

✅ **완전 통합 시뮬레이션**: 모든 Gazebo 관련 파일을 단일 패키지로 통합  
✅ **센서 시뮬레이션**: 2개의 LiDAR, IMU, RealSense 카메라  
✅ **차동 구동 제어**: Differential drive 플러그인 내장  
✅ **ROS2 브리지**: 모든 센서 및 제어 토픽 자동 연결  
✅ **네비게이션 지원**: SLAM, AMCL, Nav2 완벽 지원  
✅ **커스터마이징 가능**: 월드, 센서, 파라미터 쉽게 수정 가능  

## 알려진 제한사항 및 참고사항

### Gazebo Harmonic 특이사항
1. **Joint State Publisher 불필요**
   - Gazebo에는 `JointStatePublisher` 플러그인이 내장되어 있어 `/joint_states` 토픽을 자동 발행
   - `ros_gz_bridge`가 이를 ROS2로 브리지하므로 별도의 `joint_state_publisher` 노드 불필요
   - 실제 하드웨어에서는 `joint_state_publisher`가 필요하지만, 시뮬레이션에서는 중복

2. **DiffDrive 플러그인 토픽 네임스페이스**
   - Gazebo Harmonic의 DiffDrive는 자동으로 `/model/<robot_name>/` 접두사를 추가합니다
   - URDF에서 절대 경로(`/cmd_vel`)를 지정해도 무시됩니다
   - ros_gz_bridge의 remapping 기능으로 해결됩니다

3. **Mesh 파일 경로 처리**
   - **URDF에서**: `package://robot_description/meshes/xxx.stl` 형식 사용
   - **Gazebo 내부 변환**: `package://` → `model://` URI로 변환
   - **경로 탐색**: `GZ_SIM_RESOURCE_PATH` 환경 변수를 통해 메시 파일 검색
   - **자동 설정**: Launch 파일이 자동으로 환경 변수 설정 (`install` 디렉토리 포함)
   - **주의**: 패키지를 빌드하면 메시 파일이 `install/robot_description/share/robot_description/meshes/`에 심볼릭 링크로 설치됨

4. **물리 엔진 제한**
   - DART 물리 엔진은 일부 복잡한 메시 충돌을 지원하지 않습니다
   - 캐스터 휠은 단순 collision geometry로 대체됩니다

5. **시각화**
   - Gazebo가 일시정지 상태로 시작되면 수동으로 재생 버튼을 눌러야 합니다
   - 로봇 추적은 기본적으로 비활성화되어 있습니다 (우클릭 → Follow로 활성화)

### 성능 고려사항
- 센서가 많고 복잡한 환경일수록 시뮬레이션 속도가 느려집니다
- RTF(Real Time Factor) 값으로 성능을 모니터링하세요 (1.0이 실시간)
- 필요시 센서 업데이트 주기를 낮춰서 성능을 향상시킬 수 있습니다

## 로봇이 움직이지 않는 경우 문제 해결

`cmd_vel` 토픽을 발행해도 로봇이 움직이지 않는다면 다음을 확인하세요:

### 1. Gazebo 시뮬레이션 재생 상태 확인
**가장 흔한 원인입니다!**
- Gazebo GUI 좌측 하단의 **재생 버튼(▶)이 눌려있는지** 확인
- 일시정지 상태(||)에서는 로봇이 움직이지 않습니다

### 2. cmd_vel 메시지 전달 확인

cmd_vel 명령이 Gazebo까지 전달되는지 확인:
```bash
# ROS 토픽 확인
ros2 topic echo /cmd_vel

# Gazebo 토픽 확인 (메시지가 도착하는지)
gz topic -e -t /model/robot/cmd_vel -n 5
```

### 3. 브리지 동작 확인

ros_gz_bridge가 올바르게 메시지를 전달하는지 확인:
```bash
# 브리지 프로세스 확인
ps aux | grep parameter_bridge

# 브리지 로그 확인 (launch 터미널 출력)
# "Creating ROS->GZ Bridge: [/model/robot/cmd_vel ..." 메시지 확인
```

### 4. DiffDrive 플러그인 로딩 확인

Gazebo 로그에서 DiffDrive 플러그인이 로드되었는지 확인:
```bash
# Launch 터미널 출력에서 다음 메시지 확인
# "DiffDrive subscribing to twist messages on [/model/robot/cmd_vel]"
```

### 5. 로봇의 실제 속도 확인

오도메트리를 통해 로봇이 실제로 움직이는지 확인:
```bash
# cmd_vel 명령 발행
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 10

# 다른 터미널에서 odom의 속도 확인
ros2 topic echo /odom --field twist.twist.linear.x

# 값이 0이 아니면 로봇이 움직이는 중입니다
```

### 6. 테스트 절차

완전한 테스트 절차:
```bash
# 1. Gazebo 실행
ros2 launch robot_gazebo_sim gazebo_sim.launch.py

# 2. Gazebo GUI에서 재생 버튼(▶) 클릭

# 3. 다른 터미널에서 cmd_vel 발행
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --rate 10

# 4. odom 확인
ros2 topic echo /odom --field pose.pose.position

# x, y 값이 변하면 로봇이 움직이는 것입니다
```

### 7. 알려진 제한사항

- **바퀴 슬립**: 마찰 계수가 낮거나 접촉 파라미터가 잘못 설정되면 바퀴가 헛돌 수 있습니다
- **과도한 접촉 강성**: `kp` 값이 너무 높으면 시뮬레이션이 불안정해질 수 있습니다
- **Z축 위치**: 로봇이 너무 높거나 낮게 spawn되면 접촉이 불안정해집니다
  - 권장 z_pose: 0.06 (바퀴 반지름 0.0508에 근접)

### 8. 고급 디버깅

시뮬레이션이 실행 중인지 확인:
```bash
# Gazebo 토픽 리스트
gz topic -l

# 모델 리스트
gz model --list

# 로봇의 조인트 확인
gz model -m robot --joint

# 특정 조인트의 속도가 변하는지 확인 (별도 스크립트 필요)
watch -n 0.5 "gz model -m robot --joint | grep -A 5 'base_l_wheel_joint'"
```

## 버전 정보

- **ROS2**: Jazzy Jalisco
- **Gazebo**: Harmonic (v8.9.0)
- **Ubuntu**: 24.04 Noble
- **패키지 버전**: 1.0.0

## 라이센스

MIT License

## 기여 및 지원

버그 리포트나 기능 요청은 GitHub 이슈로 등록해 주세요.
