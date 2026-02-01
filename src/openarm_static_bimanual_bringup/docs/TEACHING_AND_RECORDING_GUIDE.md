# OpenArm Bimanual VLA 2단계 데이터 수집 가이드

이 문서는 OpenArm Static Bimanual 로봇의 **2단계 VLA (Vision-Language-Action) 데이터 수집 워크플로우**를 설명합니다. 실제 하드웨어 및 Mock 하드웨어(시뮬레이션) 모드 모두 지원합니다.

---

## 목차

1. [워크플로우 개요](#1-워크플로우-개요)
2. [환경 설정](#2-환경-설정)
3. [Phase 1: Trajectory 녹화](#3-phase-1-trajectory-녹화)
4. [Phase 2: VLA 데이터셋 생성](#4-phase-2-vla-데이터셋-생성)
5. [Mock Hardware 테스트 가이드](#5-mock-hardware-테스트-가이드)
6. [데이터셋 구조](#6-데이터셋-구조)
7. [파라미터 레퍼런스](#7-파라미터-레퍼런스)
8. [문제 해결](#8-문제-해결)

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

## 3. Phase 1: Trajectory 녹화

**목표**: 카메라 없이 조인트 데이터만 경량 녹화

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

ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args \
    -p dataset_name:=my_trajectory \
    -p task_description:="pick and place red cube"
```

|  키  | 동작                  |
| :---: | --------------------- |
| `r` | 에피소드 녹화 시작    |
| `s` | 에피소드 저장         |
| `q` | 데이터셋 저장 후 종료 |

### Task_description

#### Short Horizon:

"Move the paper box at the front to the back"
(앞쪽에 있는 종이 박스를 뒤쪽으로 옮겨라)
"Move the cube on the right to the left
"(오른쪽에 있는 큐브를 왼쪽으로 옮겨라)

#### Long Horizon:

"Put the items on the desk into the basket"
(책상에 있는 물건들을 바구니에 넣어라)
"Put the umbrellas into the basket"
(우산들을 바구니에 넣어라)
"Stack the paper boxes on the desk"
(책상에 있는 종이 박스들을 쌓아라)
"Put the toilet paper rolls into the basket"
(휴지들을 바구니에 넣어라)

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

## 4. Phase 2: VLA 데이터셋 생성

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

> [!NOTE]
> 카메라 시리얼은 `rs-enumerate-devices -s`로 확인 가능

**Terminal 2: VLA 데이터셋 생성 (자동)**

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

## 5. Mock Hardware 테스트 가이드

실제 하드웨어 없이 전체 파이프라인을 테스트할 수 있습니다.

### Mock 하드웨어 개요

```
┌───────────────────────────────────────────────────────────────┐
│  Mock Hardware 모드                                           │
│  - 실제 CAN 통신 없이 가상 조인트 상태 생성                     │
│  - fake_camera_publisher.py로 가상 카메라 이미지 제공          │
│  - 실제 하드웨어와 동일한 데이터셋 구조 생성                    │
└───────────────────────────────────────────────────────────────┘
```

---

### Phase 1 (Mock): Trajectory 녹화

#### 필요 터미널: 3개

**Terminal 1: Mock 로봇 환경 + 중력보상**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    use_mock_hardware:=true
```

**Terminal 2: 그리퍼 제어**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

> [!TIP]
> Mock 모드에서도 그리퍼 키보드 제어는 동일하게 작동합니다.

**Terminal 3: Trajectory 녹화**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args \
    -p dataset_name:=mock_test_trajectory \
    -p task_description:="mock hardware test"
```

#### 녹화 후 종료 순서

1. **Terminal 3**: `q` 키 → 데이터셋 저장 후 종료
2. **Terminal 2**: `ESC` 키 → 키보드 컨트롤러 종료
3. **Terminal 1**: `Ctrl+C` → 로봇 환경 종료

---

### Phase 2 (Mock): VLA 데이터셋 생성

#### 필요 터미널: 2개

**Terminal 1: Fake Camera Publisher (먼저 실행!)**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 run openarm_static_bimanual_bringup fake_camera_publisher.py
```

> [!IMPORTANT]
> **반드시 Terminal 1을 먼저 실행**하세요! VLA 녹화기가 카메라 토픽을 기다립니다.

확인 메시지:

```
[INFO] Publishing to /camera/cam_1/color/image_raw
[INFO] Publishing to /camera/cam_2/color/image_raw
[INFO] Publishing to /camera/cam_3/color/image_raw
[INFO] Fake Camera Publisher Started!
[INFO] Publishing at 20.0 Hz
```

**Terminal 2: VLA 데이터셋 생성**

```bash
source /opt/ros/humble/setup.bash
source ~/OpenArm0.3_data/install/setup.bash

ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    use_mock_hardware:=true \
    trajectory_dataset:=~/lerobot_datasets/mock_test_trajectory \
    vla_dataset:=~/lerobot_datasets/mock_test_vla \
    task_description:="mock hardware test"
```

#### 완료 후 종료 순서

1. **Terminal 2**: 자동 완료 대기 → 완료 후 자동 종료
2. **Terminal 1**: `Ctrl+C` → Fake Camera Publisher 종료

---

### Mock 테스트 전체 실행 순서 요약

#### 📋 Phase 1 (3개 터미널)

| 순서 | 터미널 | 명령어                                                                             |    종료 방법    |
| :--: | :----: | ---------------------------------------------------------------------------------- | :-------------: |
|  1  |   T1   | `ros2 launch ... lerobot_trajectory_recording.launch.py use_mock_hardware:=true` | Ctrl+C (마지막) |
|  2  |   T2   | 그리퍼 컨트롤러 활성화 + 실행                                                      |       ESC       |
|  3  |   T3   | `ros2 run ... lerobot_trajectory_recorder.py`                                    |    q (먼저)    |

**종료 순서**: T3 → T2 → T1

#### 📋 Phase 2 (2개 터미널)

| 순서 | 터미널 | 명령어                                                                           |    종료 방법    |
| :--: | :----: | -------------------------------------------------------------------------------- | :-------------: |
|  1  |   T1   | `ros2 run ... fake_camera_publisher.py`                                        | Ctrl+C (마지막) |
|  2  |   T2   | `ros2 launch ... lerobot_vla_collection.launch.py use_mock_hardware:=true ...` |    자동 완료    |

**종료 순서**: T2 자동완료 대기 → T1

---

## 6. 데이터셋 구조

### Phase 1: Trajectory 데이터셋

```
~/lerobot_datasets/my_trajectory/
├── meta/
│   ├── info.json
│   ├── tasks.parquet
│   └── episodes/
│       └── episode_000000.parquet
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
│   ├── stats.json
│   ├── tasks.parquet
│   └── episodes/
│       └── episode_000000.parquet
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
- **FPS**: 30Hz
- **포맷**: MP4 비디오 (LeRobot v3.0 표준)

---

## 7. 파라미터 레퍼런스

### Phase 1: `lerobot_trajectory_recording.launch.py`

| 파라미터              | 기본값    | 설명                                    |
| --------------------- | --------- | --------------------------------------- |
| `use_mock_hardware` | `false` | Mock 하드웨어 사용 여부                 |
| `can_device`        | `can0`  | CAN 디바이스 이름                       |
| `active_arms`       | `both`  | 제어 팔 (`left`, `right`, `both`) |
| `record_rate`       | `30.0`  | 녹화 Hz                                 |

### Phase 1: `lerobot_trajectory_recorder.py`

| 파라미터             | 기본값                         | 설명                        |
| -------------------- | ------------------------------ | --------------------------- |
| `dataset_name`     | `openarm_trajectory`         | 데이터셋 이름               |
| `save_dir`         | `~/lerobot_datasets`         | 저장 경로                   |
| `task_description` | `bimanual manipulation task` | VLA 태스크 설명             |
| `resume`           | `true`                       | 기존 데이터셋에 이어서 녹화 |

### Phase 2: `lerobot_vla_collection.launch.py`

| 파라미터               | 기본값                         | 설명                      |
| ---------------------- | ------------------------------ | ------------------------- |
| `use_mock_hardware`  | `false`                      | Mock 하드웨어 사용 여부   |
| `trajectory_dataset` | (필수)                         | Phase 1 데이터셋 경로     |
| `vla_dataset`        | trajectory + "_vla"            | 출력 VLA 데이터셋 경로    |
| `episode_index`      | `-1`                         | 특정 에피소드 (-1 = 전체) |
| `playback_speed`     | `1.0`                        | 재생 속도 배율            |
| `record_rate`        | `30.0`                       | 녹화 Hz                   |
| `task_description`   | `bimanual manipulation task` | VLA 태스크 설명           |

---

## 8. 문제 해결

### Phase 1 문제

**"Cannot get joint states"**

```bash
# /joint_states 토픽 확인
ros2 topic hz /joint_states
```

**로봇 팔이 움직이지 않음**

- 중력보상 노드가 실행 중인지 확인
- Mock 모드에서는 가상 조인트 상태만 업데이트됨

### Phase 2 문제

**"Waiting for cameras"**

```bash
# 카메라 토픽 확인
ros2 topic list | grep camera

# Mock 모드: fake_camera_publisher가 실행 중인지 확인
ros2 topic hz /camera/cam_1/color/image_raw
```

**"No parquet files found"**

- `trajectory_dataset` 경로에 유효한 데이터셋이 있는지 확인
- `~/lerobot_datasets/<dataset_name>/data/chunk-*/episode_*.parquet` 파일 존재 확인

**"RepositoryNotFoundError" (VLA 데이터셋 초기화 오류)**

```bash
# 불완전한 VLA 데이터셋 삭제 후 재시도
rm -rf ~/lerobot_datasets/mock_test_vla/
```

### Mock 모드 특정 문제

**Fake Camera 이미지가 수신되지 않음**

```bash
# fake_camera_publisher 로그 확인
# "Publishing at 20.0 Hz" 메시지가 보여야 함
```

### 공통 문제

**LeRobot Import 오류**

```bash
pip install lerobot
# 또는 (권장)
cd ~/lerobot_FMVLA && pip install -e .
```

**numpy/pandas 호환성**

```bash
pip install numpy>=1.22.4 pandas>=2.0.0
```

**Float64MultiArray 데이터 타입 오류**

- 이미 패치됨: NumPy 타입이 Python float로 자동 변환됩니다

---

## 빠른 참조: 전체 명령어

### 🔧 실제 하드웨어 모드

#### Phase 1 (수동 티칭)

```bash
# Terminal 1: 로봇 환경
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py

# Terminal 2: 그리퍼 제어
ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

# Terminal 3: 녹화
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py
```

#### Phase 2 (자동 VLA 생성)

```bash
# Terminal 1: 카메라 실행
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py ...

# Terminal 2: VLA 데이터셋 생성
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    trajectory_dataset:=~/lerobot_datasets/my_trajectory
```

---

### 🧪 Mock 하드웨어 모드

#### Phase 1 (Mock 티칭)

```bash
# Terminal 1: Mock 로봇 환경
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    use_mock_hardware:=true

# Terminal 2: 그리퍼 제어
ros2 control switch_controllers --activate left_gripper_controller right_gripper_controller
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

# Terminal 3: 녹화
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args -p dataset_name:=mock_test_trajectory
```

#### Phase 2 (Mock VLA 생성)

```bash
# Terminal 1: Fake 카메라 (먼저 실행!)
ros2 run openarm_static_bimanual_bringup fake_camera_publisher.py

# Terminal 2: VLA 데이터셋 생성
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    use_mock_hardware:=true \
    trajectory_dataset:=~/lerobot_datasets/mock_test_trajectory \
    vla_dataset:=~/lerobot_datasets/mock_test_vla \
    task_description:="mock hardware test"
```

---

## VLA 학습으로 진행

생성된 VLA 데이터셋을 사용하여 모델 학습:

```bash
# LeRobot 학습 명령어 예시
cd ~/lerobot_FMVLA

python lerobot/scripts/train.py \
    --dataset.repo_id=local/mock_test_vla \
    --dataset.root=~/lerobot_datasets/mock_test_vla \
    --dataset.local_files_only=true \
    --policy.type=act \
    --output_dir=outputs/train/openarm_act
```

> [!NOTE]
> `use_videos=True`로 생성된 데이터셋은 LeRobot의 자동 비디오 디코딩 기능을 통해
> 학습 시 실시간으로 프레임을 로드합니다.
