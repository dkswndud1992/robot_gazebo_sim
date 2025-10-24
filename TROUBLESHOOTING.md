# 로봇 가제보 시뮬레이션 문제 해결 가이드

## 현재 상태
- ✅ Gazebo Harmonic 설치 완료 (v8.9.0)
- ✅ ROS2 Jazzy 환경 구성 완료
- ✅ robot_gazebo_sim 패키지 빌드 성공
- ✅ Gazebo 서버 및 GUI 실행 성공
- ✅ 로봇 스폰 성공 ("Entity creation successful")
- ✅ ROS2 토픽 브리지 작동 (8/8 토픽)

## 문제 원인

### 1. 중복 프레임 에러
```
Warning [Utils.cc:115] Non-unique name[camera] detected 2 times in XML
Warning [Utils.cc:115] Non-unique name[camera1_link] detected 2 times in XML  
Warning [Utils.cc:115] Non-unique name[front_bumper] detected 2 times in XML
[Err] [UserCommands.cc:1152] Error Code 2: Msg: frame with name[camera] already exists.
```

**원인**: `robot.xacro`에서 이미 정의된 링크들이 중복 생성되고 있음

**수정 완료**:
- `camera_joint`, `front_bumper_joint`, `camera1_joint`로 조인트 이름 변경
- `/home/robot/ros2_ws/src/robot_description/urdf/robot.xacro` 파일 수정됨

### 2. STL 메시 파일 로딩
- 메시 파일들이 존재함 (/home/robot/ros2_ws/src/robot_description/meshes/)
- install 디렉토리에도 복사됨
- 하지만 Gazebo에서 로딩 확인 필요

## 해결 방안

### 방법 1: 현재 코드 재빌드 및 실행 (조인트명 수정 반영)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_description robot_gazebo_sim --symlink-install
source install/setup.bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py
```

### 방법 2: Gazebo GUI에서 직접 확인

1. Gazebo 실행 후 왼쪽 Entity Tree 패널 확인
2. `robot_gazebo` 모델이 있는지 확인
3. 모델 선택 후 Component Inspector에서 Pose 확인 (0, 0, 0.1 예상)
4. View → Frames 활성화하여 프레임 시각화
5. View → Collisions 활성화하여 충돌 영역 확인

### 방법 3: 간단한 URDF로 테스트

메시 파일 없이 기본 도형만으로 테스트:

```bash
cd ~/ros2_ws
source install/setup.bash

# 간단한 박스 로봇으로 테스트
gz model --spawn-file=/opt/ros/jazzy/share/gazebo_plugins/models/box.sdf --model-name=test_box
```

### 방법 4: 메시 경로 확인

```bash
# Gazebo 리소스 경로 확인
gz topic -e -t /gazebo/resource_paths

# 패키지 경로 확인
ros2 pkg prefix robot_description

# 메시 파일 존재 확인
ls -la $(ros2 pkg prefix robot_description)/share/robot_description/meshes/
```

## 추가 디버깅

### 로그 레벨 높여서 실행
```bash
ros2 launch robot_gazebo_sim gazebo_sim.launch.py --log-level debug
```

### URDF를 SDF로 변환하여 확인
```bash
cd ~/ros2_ws
source install/setup.bash
xacro src/robot_gazebo_sim/urdf/robot_gazebo.xacro > /tmp/robot.urdf
gz sdf -p /tmp/robot.urdf > /tmp/robot.sdf
cat /tmp/robot.sdf | less
```

### Gazebo 명령어로 모델 정보 확인
```bash
# 월드 내 모델 리스트
gz model --list

# 특정 모델 정보
gz model --info --model robot_gazebo

# 모델 위치
gz model --pose --model robot_gazebo
```

## 예상 문제점들

1. **메시 scale 문제**: STL 파일이 0.001 scale로 매우 작을 수 있음
2. **투명도 문제**: 일부 링크가 transparent material로 설정됨
3. **카메라 위치**: Gazebo 카메라가 로봇을 보고 있지 않을 수 있음
4. **Physics 엔진**: dartsim이 제대로 로드되지 않았을 가능성

## 다음 단계

1. 중복 프레임 에러가 해결되었는지 확인 (재빌드 후)
2. Gazebo GUI에서 Entity Tree 확인
3. 카메라를 원점으로 이동 (View → Move To → 0,0,1 입력)
4. Frames/Collisions 시각화 활성화
5. 여전히 안 보이면 simple box로 스폰 테스트

## 참고 자료

- Gazebo Harmonic 공식 문서: https://gazebosim.org/docs/harmonic
- ROS2 - Gazebo 통합: https://github.com/gazebosim/ros_gz
- URDF to SDF 변환: http://sdformat.org/tutorials?tut=pose_frame_semantics
