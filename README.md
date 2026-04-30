# KRAC_2026: Tutorial

## 개발 환경 (Ch0 · Ch1)

| 항목 | 버전 |
|------|------|
| OS | Ubuntu 22.04 |
| Firmware | PX4 v1.16.0 |
| Middleware | ROS2 Humble |
| Agent | Micro-XRCE-DDS-Agent v2.4.3 |
| Simulation | Gazebo Harmonic (v8.x) |
| Ground Control | QGC v4.4.4 / v4.4.5 |
| Python | 3.10.12 |
| OpenCV | 4.5.4 |

- 네이티브 Ubuntu 환경 권장 (WSL2는 GPU·물리포트 제약 있음)
- Ubuntu 설치 시 언어를 **영어**로 설정할 것 (한글 설정 시 `ros_gz_bridge` apt 설치 제한)
- Ubuntu 24.05 자동 업그레이드 **거부**할 것

### 핵심 설치 순서
1. Ubuntu 22.04 설치
2. ROS2 Humble 설치 → `.bashrc`에 `source /opt/ros/humble/setup.bash` 추가
3. PX4 Firmware 클론 후 `git checkout v1.16.0`
4. Micro-XRCE-DDS-Agent 설치
5. `px4_msgs` 전용 workspace 구축 후 `.bashrc`에 source 추가
6. QGC v4.4.5 (또는 v4.4.4) 설치
7. `ros-humble-ros-gzharmonic` 설치 (`ros_gz_bridge`)
8. OpenCV 호환성 문제 시 NumPy를 `<2` 버전으로 다운그레이드

---

## VTOL 자율비행 및 정밀착륙 시뮬레이션 (Ch4)

### 1. 임무 개요

출발지(A)에서 이륙 → 고정익 전환 → 참고점(B·C·D) 순회 → 회전익 전환 → ArUco 마커 기반 정밀착륙

### 2. Airframe 설정

PX4 기본 `standard_vtol`에 LiDAR·카메라를 탑재한 커스텀 모델(`standard_vtol_sensors`) 생성.

```bash
# 모델 SDF 생성
cd ~/PX4-Autopilot/Tools/simulation/gz/models/
mkdir -p standard_vtol_sensors/
nano standard_vtol_sensors/model.sdf

# Airframe 파일 등록
cd ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
cp 4004_gz_standard_vtol 4022_gz_standard_vtol_sensors
# CMakeLists.txt에도 4022_gz_standard_vtol_sensors 항목 추가

# 시뮬레이션 확인
cd ~/PX4-Autopilot
make px4_sitl gz_standard_vtol_sensors
```

### 3. World 설정 (바람 환경)

`aruco_windy.sdf`를 생성하여 바람 및 ArUco 마커 환경 구성.

```bash
cd ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp aruco.sdf aruco_windy.sdf
# aruco_windy.sdf 내 <enable_wind>를 true로 수정
# <wind><linear_velocity>5 2 0</linear_velocity></wind> 추가
```

동적 바람 세기 변화는 Gazebo의 **WindEffects 플러그인**(`libgz-sim8-wind-effects-system.so`)을 런타임에 추가하여 구현.

### 4. 자율비행

- **회전익**: `offboard_control_mode`에서 `position` 또는 `velocity` 중 하나만 `true`로 설정
- **고정익**: `position`과 `velocity` 모두 `true`, `trajectory_setpoint`에도 두 값 모두 제공
- **전환(VTOL Transition)**: `VEHICLE_CMD_DO_VTOL_TRANSITION` 사용 — 회전익=`3`, 고정익=`4`
- 천이 실패 시 QGC → Parameters → `VT_F_TRANS_THR` 값을 올릴 것 (airframe 파일에서 영구 설정 권장)
- 좌표계: **NED 기준** (필요 시 변환)

### 5. Gazebo → ROS 토픽 변환

LiDAR 등 센서 데이터는 Gazebo 토픽으로 발행되므로 bridge가 필요.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco_windy/model/standard_vtol_sensors_0/link/lidar_sensor_link/sensor/lidar/scan/points\
  @sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked
```

### 6. 영상정보 처리

Python + OpenCV로 ArUco 마커 인식 노드(`marker_recognition`) 작성.

```bash
ros2 run imagery_processing marker_recognition \
  --ros-args -p camera_source:=1 -p world:="aruco_windy" -p airframe:="standard_vtol_sensors"
# camera_source: 0=물리카메라, 1=시뮬레이션(UDP)
```

- LiDAR `PointCloud2` 데이터(little-endian)를 `struct.unpack('<f', ...)` 로 파싱하여 고도 계산
- 영상 창은 `mission_mode == LANDING`일 때만 표시 (리소스 절약)

### 7. 정밀착륙

실제 환경의 바람·진동 오차를 ArUco 마커로 보정.

| 방식 | 특징 |
|------|------|
| **좌표 기준 유도** | 안정적, 강풍 대응 약함, 하강 속력 직관성 낮음 |
| **속도 기준 유도** | 외부 요인 대응 유리, 정밀도 다소 낮음 |

```bash
# 위치 기반, 수동 출발
ros2 run flight_control landing_test \
  --ros-args -p start_param:=0 -p land_param:=0 -p descent_param:=1.0

# 속도 기반, 지정 좌표 자동 출발
ros2 run flight_control landing_test \
  --ros-args -p start_param:=1 -p start_x_param:=5 -p start_y_param:=6 \
  -p start_z_param:=10 -p land_param:=1 -p descent_param:=0.5
```

> 실행 전 `marker_recognition` 노드와 `ros_gz_bridge` 모두 구동 필요

### 8. 짐벌 제어

MAVLink Gimbal Protocol v2 (`gimbal_manager`) 사용.

```cpp
// 제어 권한 부여
publish_vehicle_command_(VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE, 1, 1);

// 착륙 시 카메라를 수직 아래로
publish_vehicle_command_(VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW, -90, 0, nan, nan);
// pitch rate/yaw rate를 지정 안 할 경우 반드시 NaN 부여
```

### 9. 통합 시뮬레이션 실행 순서

각 단계마다 별도의 터미널 창 필요.

```bash
# 1. Agent
MicroXRCEAgent udp4 -p 8888

# 2. QGC
./QGroundControl-x86_64.AppImage

# 3. Gazebo
cd PX4-Autopilot/
PX4_GZ_WORLD=aruco_windy make px4_sitl gz_standard_vtol_sensors

# 4. ROS-Gazebo Bridge
cd ros2_gz_ws && source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/aruco_windy/model/standard_vtol_sensors_0/link/lidar_sensor_link/sensor/lidar/scan/points\
  @sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked

# 5. 영상정보 처리 노드
cd ws_KRAC && source install/setup.bash
ros2 run imagery_processing marker_recognition \
  --ros-args -p camera_source:=1 -p world:="aruco_windy" -p airframe:="standard_vtol_sensors"

# 6. 자율비행 노드
ros2 run flight_control landing_test \
  --ros-args -p start_param:=2 -p land_param:=1 -p descent_param:=0.5
```

---

## 참고 코드 (GitHub)

| 내용 | 링크 |
|------|------|
| `model.sdf` (standard_vtol_sensors) | [링크](https://github.com/seunguniii/space_y/blob/main/resources/standard_vtol_sensors/model.sdf) |
| Airframe 파일 (4022) | [링크](https://github.com/seunguniii/space_y/blob/main/resources/standard_vtol_sensors/4022_gz_standard_vtol_sensors) |
| `aruco_windy.sdf` | [링크](https://github.com/seunguniii/space_y/blob/main/resources/world/aruco_windy.sdf) |
| WindEffects Plugin XML | [링크](https://github.com/seunguniii/space_y/blob/main/resources/world/inner.xml) |
| VTOL 자율비행 (`flight_test.cpp`) | [링크](https://github.com/seunguniii/space_y/blob/main/tutorial/src/flight/src/flight_test.cpp) |
| 영상처리 · LiDAR 고도 (`aruco_marker.py`) | [링크](https://github.com/seunguniii/space_y/blob/main/tutorial/src/video/video/aruco_marker.py) |
| 정밀착륙 (`land_test.cpp`) | [링크](https://github.com/seunguniii/space_y/blob/main/tutorial/src/flight/src/land_test.cpp) |
| 통합본 (`flight.cpp`) | [링크](https://github.com/seunguniii/space_y/blob/main/tutorial/src/flight/src/flight.cpp) |

---

## 유용한 외부 링크

- [PX4 uORB 메시지 목록](https://docs.px4.io/main/en/msg_docs/)
- [MAVLink Common Messages](https://mavlink.io/en/messages/common.html)
- [Gimbal Protocol v2](https://mavlink.io/en/services/gimbal_v2.html)
- [Gazebo SDF 레퍼런스](http://sdformat.org/)
- [Gazebo 센서 구현](https://gazebosim.org/docs/latest/sensors/)
- [ROS2 Humble 튜토리얼](https://docs.ros.org/en/humble/index.html)
- [PX4 ROS2 가이드](https://docs.px4.io/v1.16/en/ros2/user_guide)


