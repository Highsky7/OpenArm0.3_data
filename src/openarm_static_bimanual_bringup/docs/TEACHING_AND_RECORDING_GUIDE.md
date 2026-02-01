# OpenArm Bimanual VLA 2단계 데이터 수집 가이드

이 문서는 OpenArm Static Bimanual 로봇의 **2단계 VLA (Vision-Language-Action) 데이터 수집 워크플로우**를 설명합니다. 실제 하드웨어 및 Mock 하드웨어(시뮬레이션) 모드 모두 지원합니다.

---

## 목차

1. [워크플로우 개요](#1-워크플로우-개요)
2. [환경 설정](#2-환경-설정)
3. [데이터셋 관리 및 전략](#3-데이터셋-관리-및-전략)
4. [Phase 1: Trajectory 녹화](#4-phase-1-trajectory-녹화)
5. [Phase 2: VLA 데이터셋 생성](#5-phase-2-vla-데이터셋-생성)
6. [Mock Hardware 테스트 가이드](#6-mock-hardware-테스트-가이드)
7. [데이터셋 구조](#7-데이터셋-구조)
8. [파라미터 레퍼런스](#8-파라미터-레퍼런스)
9. [문제 해결](#9-문제-해결)

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

| 장점                | 설명                                      |
| ------------------- | ----------------------------------------- |
| **반복 가능** | 동일 trajectory를 여러 환경/조명에서 녹화 |
| **품질 향상** | 카메라 안정화 시간 확보                   |
| **효율성**    | Phase 1은 빠르게 수집, Phase 2는 자동화   |

### 데이터셋 구성 (LeRobot v3.0)

| Feature                            | Shape         | 설명                               |
| ---------------------------------- | ------------- | ---------------------------------- |
| `observation.state`              | (16,)         | 16-DOF 조인트 위치                 |
| `action`                         | (16,)         | 다음 프레임 조인트 위치 (Absolute) |
| `observation.images.top`         | (256, 256, 3) | 상단 카메라                        |
| `observation.images.wrist_left`  | (256, 256, 3) | 왼쪽 손목 카메라                   |
| `observation.images.wrist_right` | (256, 256, 3) | 오른쪽 손목 카메라                 |
| `task`                           | string        | 작업 설명 (Multi-task 지원)        |

---

## 2. 환경 설정

### 필수 조건

```bash
# ROS 2 Humble 환경
source /opt/ros/humble/setup.bash

# OpenArm 패키지
source ~/OpenArm0.3_data/install/setup.bash

# LeRobot (editable mode 권장)
cd ~/lerobot_FMVLA && pip install -e .
```

### 모드 선택

| 모드                    | 파라미터                              | 용도           |
| ----------------------- | ------------------------------------- | -------------- |
| **실제 하드웨어** | `use_mock_hardware:=false` (기본값) | 실제 로봇 운용 |
| **Mock 하드웨어** | `use_mock_hardware:=true`           | 개발/테스트/CI |

---

## 3. 데이터셋 관리 및 전략

효률적인 학습을 위해 작업(Task)별로 데이터셋을 분리하거나, 기존 데이터셋에 데이터를 추가하는 방법을 이해해야 합니다.

### Resume 기능 (기본 활성화)

`lerobot_trajectory_recorder.py`는 기본적으로 **Resume(이어쓰기)** 모드가 활성화되어 있습니다(`resume:=true`).

- **동일한 `dataset_name`**을 사용하면:
  - 기존 데이터셋 폴더를 감지합니다.
  - 기존 에피소드 뒤에 **새로운 에피소드를 추가(Append)**합니다.
  - 예: 어제 50개 녹화 → 오늘 동일 이름으로 실행 → 51번째 에피소드부터 녹화됨.

### Task 별 데이터셋 분리 전략

서로 다른 작업(Task)은 별도의 데이터셋 이름(`dataset_name`)으로 저장하는 것을 권장합니다.

**예시 시나리오:**

1. **"Pick and Place Red Cube" 작업**

   - `dataset_name:=pick_red_cube`
   - `task_description:="pick up the red cube and place it on the tray"`
   - 여러 세션에 걸쳐 50회 녹화 (Resume 활용)
2. **"Stack Blue Box" 작업**

   - `dataset_name:=stack_blue_box`
   - `task_description:="stack the blue box on top of the green box"`
   - 별도 데이터셋으로 50회 녹화

> [!TIP]
> Phase 1에서 설정한 `task_description`은 Phase 2 실행 시 다시 입력해야 최종 VLA 데이터셋에 올바르게 반영됩니다.

---

## 4. Phase 1: Trajectory 녹화

**목표**: 카메라 없이 조인트 데이터만 경량 녹화 (고속, 30Hz)

### 실제 하드웨어 모드

#### 필요 터미널: 3개

**Terminal 1: 로봇 환경 + 중력보상**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py
```

**Terminal 2: 그리퍼 제어**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

|   키   | 동작               |
| :-----: | ------------------ |
|  `q`  | 왼쪽 그리퍼 열기   |
|  `w`  | 왼쪽 그리퍼 닫기   |
|  `o`  | 오른쪽 그리퍼 열기 |
|  `p`  | 오른쪽 그리퍼 닫기 |
| `ESC` | 종료               |

**Terminal 3: Trajectory 녹화**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

# 예시: 'pick_red_cube' 데이터셋에 저장/이어쓰기
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args \
    -p dataset_name:=pick_red_cube \
    -p task_description:="pick up the red cube and place it on the tray" \
    -p resume:=true
```

|  키  | 동작                  |
| :---: | --------------------- |
| `r` | 에피소드 녹화 시작    |
| `s` | 에피소드 저장         |
| `q` | 데이터셋 저장 후 종료 |

### 녹화 워크플로우

1. 모든 터미널 실행 후 로봇 안정화 대기 (~5초)
2. **Terminal 3**에서 `r` 키로 녹화 시작
3. 로봇 팔을 손으로 조작 + **Terminal 2**에서 그리퍼 제어
4. 작업 완료 시 **Terminal 3**에서 `s` 키로 에피소드 저장
5. 2~4 반복하여 여러 에피소드 녹화 (Resume 기능으로 누적됨)
6. 모든 녹화 완료 후 **Terminal 3**에서 `q` 키로 종료

> [!IMPORTANT]
> **Task Description (Short/Long Horizon)**
>
> - "Move the paper box at the front to the back" (dataset name: Moving box)
> - "Move the cube on the right to the left" (dataset name: Moving cube)
> - "Put the items on the desk into the basket" (dataset name: Putting items)
> - "Put the umbrellas into the basket" (dataset name: Putting umbrellas)
> - "Stack the paper boxes on the desk" (dataset name: Stacking boxes)
> - "Put the toilet paper rolls into the basket" (dataset name: Putting toilet paper rolls)
> - Phase 1에서 입력한 설명은 참고용이며, **Phase 2에서 최종 확정**됩니다.

---

## 5. Phase 2: VLA 데이터셋 생성

**목표**: Phase 1 trajectory를 재생하며 카메라 observation 녹화

### 실제 하드웨어 모드

#### 필요 터미널: 2개

**Terminal 1: 카메라 실행**

```bash
source /opt/ros/humble/setup.bash
source ~/realsense_ws/install/setup.bash

ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
  camera_name1:=cam_1 camera_name2:=cam_2 camera_name3:=cam_3 \
  camera_namespace1:=camera camera_namespace2:=camera camera_namespace3:=camera \
  serial_no1:='_346222072155' serial_no2:='_247122072494' serial_no3:='_247122074423'
```

**Terminal 2: VLA 데이터셋 생성 (자동)**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

# Phase 1 데이터셋(trajectory_dataset)을 읽어서 -> VLA 데이터셋(vla_dataset) 생성
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    trajectory_dataset:=~/lerobot_datasets/pick_red_cube \
    vla_dataset:=~/lerobot_datasets/pick_red_cube_vla \
    vla_dataset:=~/lerobot_datasets/pick_red_cube_vla \
    task_description:="pick up the red cube and place it on the tray" \
    resume:=true
```

### 처리 흐름

1. 로봇 초기화 + 카메라 연결 확인 (~10초 대기)
2. **자동 재생 시작**: 로봇이 Phase 1의 경로를 그대로 따라 움직임
3. **자동 녹화**: 카메라 영상 + Joint State를 동기화하여 저장
4. 모든 에피소드 처리 후 **자동 종료**

> [!WARNING]
> **안전 주의**: 로봇이 Phase 1에서 녹화된 움직임을 1.0배속으로 그대로 재현합니다. 로봇 주변에 장애물이 없는지, 사람이 너무 가까이 있지 않은지 확인하세요.

---

## 6. Mock Hardware 테스트 가이드

실제 로봇/카메라 없이 워크플로우를 검증할 때 사용합니다.

### Phase 1 (Mock): Trajectory 녹화

```bash
# T1: Mock 로봇
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    use_mock_hardware:=true

# T2: 그리퍼 제어
ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

# T3: 녹화 (mock_test_dataset)
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args -p dataset_name:=mock_test_dataset -p resume:=true
```

### Phase 2 (Mock): VLA 데이터셋 생성

```bash
# T1: Fake Camera (필수)
ros2 run openarm_static_bimanual_bringup fake_camera_publisher.py

# T2: VLA 생성
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    use_mock_hardware:=true \
    trajectory_dataset:=~/lerobot_datasets/mock_test_dataset \
    vla_dataset:=~/lerobot_datasets/mock_test_dataset_vla \
    task_description:="mock task"
```

---

## 7. 데이터셋 구조

### Phase 1: Trajectory

경로: `~/lerobot_datasets/<dataset_name>`

- `observation.state`, `action` 포함
- 영상 없음
- 용량이 작음

### Phase 2: VLA

경로: `~/lerobot_datasets/<dataset_name>_vla` (권장)

- `observation.images.*` 포함 (MP4 비디오)
- `observation.state`, `action` 포함
- **LeRobot 학습에 바로 사용 가능**

---

## 8. 파라미터 레퍼런스

### `lerobot_trajectory_recorder.py` (Phase 1)

| 파라미터             | 기본값                 | 설명                                                       |
| :------------------- | :--------------------- | :--------------------------------------------------------- |
| `dataset_name`     | `openarm_trajectory` | 저장할 데이터셋 폴더명                                     |
| `task_description` | `bimanual task`      | 작업에 대한 자연어 설명                                    |
| `resume`           | `true`               | `true`: 기존 데이터셋에 추가 / `false`: 덮어쓰기(주의) |
| `record_rate`      | `30.0`               | 녹화 주기 (Hz). 카메라가 없으므로 높게 설정 가능           |

### `lerobot_vla_collection.launch.py` (Phase 2)

| 파라미터               | 필수          | 설명                                                      |
| :--------------------- | :------------ | :-------------------------------------------------------- |
| `trajectory_dataset` | **Yes** | 입력: Phase 1에서 만든 데이터셋 경로                      |
| `vla_dataset`        | No            | 출력: 생성할 VLA 데이터셋 경로 (기본: 입력경로 +`_vla`) |
| `task_description`   | No            | 최종 데이터셋에 저장될 작업 설명                          |
| `episode_index`      | No            | `-1`: 전체 변환, `0`: 0번 에피소드만 변환 (테스트용)  |
| `playback_speed`     | No            | 재생 속도 배율 (기본 1.0)                                 |
| `resume`             | No            | `true`: 기존 VLA 데이터셋에 추가 (기본값) / `false`: 덮어쓰기 |

---

## 9. 문제 해결

### Resume가 작동하지 않음 (새로 덮어씌워짐)

- **원인**: `dataset_name`이 변경되었거나, `save_dir`(기본: `~/lerobot_datasets`) 경로가 다를 수 있습니다.
- **해결**: 터미널에서 `ls ~/lerobot_datasets`로 기존 폴더명을 확인하고 정확히 동일한 이름을 입력하세요.

### Phase 2에서 "No parquet files found" 오류

- **원인**: `trajectory_dataset` 경로가 잘못되었거나, Phase 1 데이터가 비어있습니다.
- **해결**: 입력한 경로 안에 `data/chunk-000` 등의 폴더와 `.parquet` 파일이 있는지 확인하세요.

### Task Description이 불일치함

- **원인**: Phase 1과 Phase 2에서 서로 다른 설명을 입력함.
- **해결**: 최종적으로 Phase 2(`lerobot_vla_collection.launch.py`) 실행 시 입력한 `task_description`이 VLA 데이터셋의 `meta/tasks.parquet`에 저장되므로, **Phase 2 실행 시 정확한 설명을 입력**하세요.

### 로봇이 너무 빠르게/느리게 움직임

- **해결**: Phase 2 실행 시 `playback_speed` 파라미터를 조절하세요.
  - 예: `playback_speed:=0.5` (0.5배속, 느리게)
  - 예: `playback_speed:=2.0` (2배속, 빠르게)
