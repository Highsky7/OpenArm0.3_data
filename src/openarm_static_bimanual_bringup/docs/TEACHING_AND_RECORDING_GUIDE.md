# OpenArm Bimanual VLA 데이터 수집 및 재생 가이드

이 문서는 OpenArm Static Bimanual 로봇의 VLA (Vision-Language-Action) 모델 학습용 데이터 수집 시스템 사용법을 설명합니다.

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [중력 보상 모드](#2-중력-보상-모드)
3. [VLA 데이터 녹화](#3-vla-데이터-녹화)
4. [키보드 그리퍼 제어](#4-키보드-그리퍼-제어)
5. [데이터 재생](#5-데이터-재생)
6. [문제 해결](#6-문제-해결)

---

## 1. 시스템 개요

### 데이터셋 구성

| Feature | Shape | 설명 |
|---------|-------|------|
| `observation.state` | (16,) | 16-DOF 조인트 위치 |
| `action` | (16,) | 다음 프레임 조인트 위치 |
| `observation.images.top` | (256, 256, 3) | 상단 카메라 |
| `observation.images.wrist_left` | (256, 256, 3) | 왼쪽 손목 카메라 |
| `observation.images.wrist_right` | (256, 256, 3) | 오른쪽 손목 카메라 |
| `language_instruction` | string | 작업 설명 |

### 카메라 토픽

| 토픽 | Feature Key |
|------|-------------|
| `camera1/cam1/color/image_raw` | `observation.images.top` |
| `camera1/cam2/color/image_raw` | `observation.images.wrist_left` |
| `camera1/cam3/color/image_raw` | `observation.images.wrist_right` |

> [!NOTE]
> 이미지는 원본 640×480에서 **256×256**으로 리사이즈되어 저장됩니다.

---

## 2. 중력 보상 모드

로봇 팔을 손으로 자유롭게 조작할 수 있는 Teaching 모드입니다.

### 실행

```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py
```

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `use_mock_hardware` | `false` | 시뮬레이션 모드 |
| `active_arms` | `both` | 활성화할 팔 |
| `rviz` | `true` | RViz 시각화 |

---

## 3. VLA 데이터 녹화

녹화를 위해 **4개의 터미널**이 필요합니다.

### Terminal 1: 로봇 인프라 + 중력 보상

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py \
  task_description:="pick and place red cube"
```

### Terminal 2: 그리퍼 컨트롤러 활성화 + 키보드 제어

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

# 1. 그리퍼 컨트롤러 활성화
ros2 control switch_controllers \
  --activate left_gripper_controller right_gripper_controller

# 2. 키보드 그리퍼 제어 실행
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

### Terminal 3: 데이터 녹화

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 run openarm_static_bimanual_bringup lerobot_vla_recorder.py
```

### Terminal 4: 카메라 (enable_cameras:=true 사용 시)

```bash
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
  camera_name1:=cam_1 camera_name2:=cam_2 camera_name3:=cam_3 \
  camera_namespace1:=camera camera_namespace2:=camera camera_namespace3:=camera \
  serial_no1:='_346222072155' serial_no2:='_247122072494' serial_no3:='_247122074423'
```

> [!NOTE]
> 시리얼 번호는 `rs-enumerate-devices -s` 명령으로 확인할 수 있습니다.

### 녹화 키보드 (Terminal 3)

| 키 | 동작 |
|:--:|------|
| `r` | 에피소드 녹화 시작 |
| `s` | 에피소드 저장 |
| `q` | 데이터셋 저장 후 종료 |

### 그리퍼 키보드 (Terminal 2)

| 키 | 동작 |
|:--:|------|
| `q` | 왼쪽 그리퍼 열기 |
| `w` | 왼쪽 그리퍼 닫기 |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| `ESC` | 종료 |

### 워크플로우

1. **Terminal 4**: 카메라 런치 실행 (카메라 사용 시)
2. **Terminal 1**: 로봇 인프라 + 중력 보상 launch 실행
3. **Terminal 2**: 그리퍼 컨트롤러 활성화 → 키보드 제어 실행
4. **Terminal 3**: 녹화기 실행
5. **Terminal 3에서** `r` 키로 녹화 시작
6. 로봇 조작 + **Terminal 2에서** 그리퍼 제어
7. **Terminal 3에서** `s` 키로 에피소드 저장
8. 5~7 반복
9. **Terminal 3에서** `q` 키로 완료

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `record_rate` | `50.0` | 녹화 주파수 (Hz) |
| `dataset_name` | `openarm_bimanual` | 데이터셋 이름 |
| `save_dir` | `~/lerobot_datasets` | 저장 경로 |
| `task_description` | `bimanual manipulation task` | VLA 언어 지시 |
| `enable_cameras` | `true` | 카메라 녹화 활성화 |

### 저장 구조

```
~/lerobot_datasets/openarm_bimanual/
├── data/
│   └── train-00000.parquet
├── videos/
│   ├── observation.images.top/
│   │   └── episode_000000.mp4
│   ├── observation.images.wrist_left/
│   │   └── episode_000000.mp4
│   └── observation.images.wrist_right/
│       └── episode_000000.mp4
└── info.json
```

---

## 4. 키보드 그리퍼 제어 (참고)

> [!NOTE]
> 그리퍼 제어는 **녹화 과정의 Terminal 2**에서 실행됩니다.
> 자세한 내용은 [3. VLA 데이터 녹화](#3-vla-데이터-녹화) 섹션을 참고하세요.

### 키보드 조작

| 키 | 동작 |
|:--:|------|
| `q` | 왼쪽 그리퍼 열기 |
| `w` | 왼쪽 그리퍼 닫기 |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| `ESC` | 종료 |

---

## 5. 데이터 재생

녹화된 데이터를 로봇에서 재생합니다. **3개의 터미널이 필요합니다.**

### Terminal 1: 로봇 인프라 실행

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup sbopenarm.launch.py \
  use_mock_hardware:=false \
  disable_torque:=false \
  active_mode:=fpc \
  rviz:=true
```

> [!NOTE]
> `active_mode:=fpc`를 사용하면 `forward_position_controller`가 기본 활성화됩니다.

### Terminal 2: 컨트롤러 설정

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

# 1. 현재 컨트롤러 상태 확인
ros2 control list_controllers

# 2. 그리퍼 컨트롤러 활성화
ros2 control switch_controllers \
  --activate left_gripper_controller right_gripper_controller

# 3. 최종 상태 확인 (아래 상태여야 함)
ros2 control list_controllers
```

**올바른 컨트롤러 상태:**

| 컨트롤러 | 상태 |
|----------|------|
| `joint_state_broadcaster` | active |
| `left_forward_position_controller` | active |
| `right_forward_position_controller` | active |
| `left_gripper_controller` | active |
| `right_gripper_controller` | active |
| 나머지 | inactive |

### Terminal 3: 재생 실행

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args -p dataset_path:=~/lerobot_datasets/openarm_bimanual \
  -p use_action:=true
```

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `dataset_path` | (필수) | 데이터셋 경로 |
| `episode_index` | `0` | 재생할 에피소드 |
| `playback_speed` | `1.0` | 재생 속도 |
| `loop` | `false` | 반복 재생 |
| `use_action` | `false` | action 컬럼 사용 |

### 예시

```bash
# 에피소드 3을 2배속 반복 재생
ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args \
  -p dataset_path:=~/lerobot_datasets/openarm_bimanual \
  -p episode_index:=3 \
  -p playback_speed:=2.0 \
  -p loop:=true \
  -p use_action:=true
```

### 문제 해결: 컨트롤러가 활성화되지 않을 때

`forward_position_controller`가 inactive 상태라면:

```bash
# 1. 충돌하는 컨트롤러들 비활성화
ros2 control switch_controllers \
  --deactivate left_effort_controller \
  --deactivate right_effort_controller \
  --deactivate left_joint_trajectory_controller \
  --deactivate right_joint_trajectory_controller \
  --deactivate left_teleop_stream_controller \
  --deactivate right_teleop_stream_controller

# 2. forward_position_controller 활성화
ros2 control switch_controllers \
  --activate left_forward_position_controller \
  --activate right_forward_position_controller

# 3. gripper_controller 활성화
ros2 control switch_controllers \
  --activate left_gripper_controller \
  --activate right_gripper_controller
```

---

## 6. 문제 해결

### rev4 조인트 오실레이션

```python
# gravity_comp_teaching.launch.py에서 조정
'gravity_scale_joints': [0.5, 2.0, 1.1, 1.8, 1.5, 1.85, 1.65]
#                                      ^^^^ rev4: 1.0 → 1.8
```

### 카메라 누락 오류

**증상**: "Cannot start: Missing cameras" 메시지

**해결**: 카메라 토픽 확인
```bash
ros2 topic list | grep camera1
```

### 재생 시 미반응

**해결**: `forward_position_controller`가 활성화되어 있는지 확인

```bash
ros2 control list_controllers
```

`left_forward_position_controller`와 `right_forward_position_controller`가 `active` 상태여야 합니다.

**launch 시 `active_mode:=fpc` 옵션 사용 권장:**
```bash
ros2 launch openarm_static_bimanual_bringup sbopenarm.launch.py \
  use_mock_hardware:=false disable_torque:=false active_mode:=fpc rviz:=true
```
