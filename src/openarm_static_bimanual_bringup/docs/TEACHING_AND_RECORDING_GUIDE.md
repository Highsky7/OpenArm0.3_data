# OpenArm Bimanual VLA 2단계 데이터 수집 가이드

이 문서는 OpenArm Static Bimanual 로봇의 **2단계 VLA (Vision-Language-Action) 데이터 수집 워크플로우**를 설명합니다.

---

## 목차

1. [워크플로우 개요](#1-워크플로우-개요)
2. [Phase 1: Trajectory 녹화](#2-phase-1-trajectory-녹화)
3. [Phase 2: VLA 데이터셋 생성](#3-phase-2-vla-데이터셋-생성)
4. [데이터셋 구조](#4-데이터셋-구조)
5. [파라미터 레퍼런스](#5-파라미터-레퍼런스)
6. [문제 해결](#6-문제-해결)

---

## 1. 워크플로우 개요

### 2단계 데이터 수집의 장점

```
┌─────────────────────────────────────────────────────────────────────┐
│  [Phase 1] 수동 티칭 → Trajectory 데이터셋                           │
│     • 중력보상 모드로 로봇 팔을 손으로 조작                            │
│     • Joint state + action 만 녹화 (카메라 없음)                      │
│     • 경량 + 고속 녹화 (30Hz)                                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│  [Phase 2] Trajectory 재생 → VLA 데이터셋                            │
│     • Phase 1 데이터를 로봇에서 재생                                  │
│     • 카메라 observation 동시 녹화                                   │
│     • 완전한 VLA 데이터셋 생성                                       │
└─────────────────────────────────────────────────────────────────────┘
```

| 장점 | 설명 |
|------|------|
| **반복 가능** | 동일 trajectory를 여러 환경/조명에서 녹화 |
| **품질 향상** | 카메라 안정화 시간 확보 |
| **효율성** | Phase 1은 빠르게 수집, Phase 2는 자동화 |

### 데이터셋 구성 (LeRobot v3.0)

| Feature | Shape | 설명 |
|---------|-------|------|
| `observation.state` | (16,) | 16-DOF 조인트 위치 |
| `action` | (16,) | 다음 프레임 조인트 위치 (Absolute) |
| `observation.images.top` | (256, 256, 3) | 상단 카메라 |
| `observation.images.wrist_left` | (256, 256, 3) | 왼쪽 손목 카메라 |
| `observation.images.wrist_right` | (256, 256, 3) | 오른쪽 손목 카메라 |
| `task` | string | 작업 설명 (Multi-task 지원) |

---

## 2. Phase 1: Trajectory 녹화

**목표**: 카메라 없이 조인트 데이터만 경량 녹화

### 필요 터미널: 3개

#### Terminal 1: 로봇 환경 + 중력보상

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py
```

#### Terminal 2: 그리퍼 제어

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

| 키 | 동작 |
|:--:|------|
| `q` | 왼쪽 그리퍼 열기 |
| `w` | 왼쪽 그리퍼 닫기 |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| `ESC` | 종료 |

#### Terminal 3: Trajectory 녹화

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args \
    -p dataset_name:=my_trajectory \
    -p task_description:="pick and place red cube"
```

| 키 | 동작 |
|:--:|------|
| `r` | 에피소드 녹화 시작 |
| `s` | 에피소드 저장 |
| `q` | 데이터셋 저장 후 종료 |

### 녹화 워크플로우

1. 모든 터미널 실행 후 로봇 안정화 대기 (~5초)
2. **Terminal 3**에서 `r` 키로 녹화 시작
3. 로봇 팔을 손으로 조작 + **Terminal 2**에서 그리퍼 제어
4. 작업 완료 시 **Terminal 3**에서 `s` 키로 에피소드 저장
5. 2~4 반복하여 여러 에피소드 녹화
6. 모든 녹화 완료 후 **Terminal 3**에서 `q` 키로 종료

> [!TIP]
> **Phase 1은 30Hz**로 녹화됩니다 (카메라 없이 고속 수집).

---

## 3. Phase 2: VLA 데이터셋 생성

**목표**: Phase 1 trajectory를 재생하며 카메라 observation 녹화

### 필요 터미널: 2개

#### Terminal 1: 카메라 실행

```bash
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
  camera_name1:=cam_1 camera_name2:=cam_2 camera_name3:=cam_3 \
  camera_namespace1:=camera camera_namespace2:=camera camera_namespace3:=camera \
  serial_no1:='_346222072155' serial_no2:='_247122072494' serial_no3:='_247122074423'
```

> [!NOTE]
> 카메라 시리얼은 `rs-enumerate-devices -s`로 확인 가능

#### Terminal 2: VLA 데이터셋 생성 (자동)

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    trajectory_dataset:=~/lerobot_datasets/my_trajectory \
    vla_dataset:=~/lerobot_datasets/my_vla \
    task_description:="pick and place red cube"
```

### 처리 흐름

1. 로봇 초기화 (~5초)
2. 카메라 초기화 대기 (~10초)
3. **자동 재생 시작** - trajectory를 따라 로봇 이동
4. **자동 녹화** - 카메라 observation + 현재 state 저장
5. 모든 에피소드 완료 후 **자동 저장**

> [!IMPORTANT]
> Phase 2는 **완전 자동**입니다. 실행 후 완료될 때까지 대기하세요.

---

## 4. 데이터셋 구조

### Phase 1: Trajectory 데이터셋

```
~/lerobot_datasets/my_trajectory/
├── meta/
│   ├── info.json
│   ├── tasks.parquet
│   └── episodes.parquet
└── data/
    └── chunk-000/
        └── episode_000000.parquet
```

- **특징**: 카메라 없음, `observation.state` + `action` 만 포함
- **FPS**: 30Hz

### Phase 2: VLA 데이터셋

```
~/lerobot_datasets/my_vla/
├── meta/
│   ├── info.json
│   ├── tasks.parquet
│   └── episodes.parquet
├── data/
│   └── chunk-000/
│       └── episode_000000.parquet
└── videos/
    └── chunk-000/
        ├── observation.images.top/
        │   └── episode_000000.mp4
        ├── observation.images.wrist_left/
        │   └── episode_000000.mp4
        └── observation.images.wrist_right/
            └── episode_000000.mp4
```

- **특징**: 카메라 포함, 완전한 VLA 데이터
- **FPS**: 20Hz

---

## 5. 파라미터 레퍼런스

### Phase 1: `lerobot_trajectory_recorder.py`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `record_rate` | `30.0` | 녹화 Hz (카메라 없이 고속) |
| `dataset_name` | `openarm_trajectory` | 데이터셋 이름 |
| `save_dir` | `~/lerobot_datasets` | 저장 경로 |
| `task_description` | `bimanual manipulation task` | VLA 태스크 설명 |
| `resume` | `true` | 기존 데이터셋에 이어서 녹화 |

### Phase 2: `lerobot_vla_collection.launch.py`

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `trajectory_dataset` | (필수) | Phase 1 데이터셋 경로 |
| `vla_dataset` | trajectory + "_vla" | 출력 VLA 데이터셋 경로 |
| `episode_index` | `-1` | 특정 에피소드 (-1 = 전체) |
| `playback_speed` | `1.0` | 재생 속도 배율 |
| `record_rate` | `20.0` | 녹화 Hz |
| `task_description` | `bimanual manipulation task` | VLA 태스크 설명 |

---

## 6. 문제 해결

### Phase 1 문제

**"Cannot get joint states"**
```bash
# /joint_states 토픽 확인
ros2 topic hz /joint_states
```

**로봇 팔이 움직이지 않음**
- 중력보상 노드가 실행 중인지 확인

### Phase 2 문제

**"Waiting for cameras"**
```bash
# 카메라 토픽 확인
ros2 topic list | grep camera
```

**"No parquet files found"**
- `trajectory_dataset` 경로에 유효한 데이터셋이 있는지 확인

### 공통 문제

**LeRobot Import 오류**
```bash
pip install lerobot
# 또는
cd ~/lerobot_FMVLA && pip install -e .
```

**numpy/pandas 호환성**
```bash
pip install numpy>=1.22.4
```

---

## 빠른 참조: 실행 명령어

### Phase 1 (수동 티칭)

```bash
# Terminal 1
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py

# Terminal 2
ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

# Terminal 3
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py
```

### Phase 2 (자동 VLA 생성)

```bash
# Terminal 1: 카메라 실행
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py ...

# Terminal 2: VLA 데이터셋 생성
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    trajectory_dataset:=~/lerobot_datasets/my_trajectory
```
