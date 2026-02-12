# 🤖 OpenArm Bimanual VLA 프로젝트 — 전체 작업 내역 종합

> 📅 작성일: 2026-02-11
> 👤 작성자: Highsky
> 🎯 목적: 프로젝트에서 수행한 모든 작업 단계를 체계적으로 정리

---

## 📋 목차

1. [프로젝트 개요](#1-프로젝트-개요)
2. [시스템 구성](#2-시스템-구성)
3. [Phase 0: 중력보상 (Gravity Compensation)](#3-phase-0-중력보상-gravity-compensation)
4. [Phase 1: 2Phase VLA 모델 학습용 데이터 녹화](#4-phase-1-2phase-vla-모델-학습용-데이터-녹화)
5. [Phase 2: VLA 모델 학습](#5-phase-2-vla-모델-학습)
6. [Phase 3: 추론 및 배포](#6-phase-3-추론-및-배포)
7. [개발된 코드 파일 목록](#7-개발된-코드-파일-목록)
8. [작성된 가이드 문서 목록](#8-작성된-가이드-문서-목록)
9. [향후 계획](#9-향후-계획)

---

## 1. 프로젝트 개요

### 목표

OpenArm v0.3 양팔(Bimanual) 로봇에 **VLA (Vision-Language-Action)** 모델을 적용하여, 자연어 지시를 이해하고 양팔로 조작 태스크를 수행하는 자율 로봇 시스템 구축.

### 전체 파이프라인

```
┌──────────────────────────────────────────────────────────────────────────────────────────┐
│                          OpenArm VLA 전체 파이프라인                                       │
├──────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                          │
│  ① 중력보상 적용                                                                          │
│  ├─ Pinocchio RNEA 기반 중력 토크 계산                                                    │
│  └─ 사용자가 로봇 팔을 자유롭게 조작 가능                                                 │
│          │                                                                               │
│          ▼                                                                               │
│  ② 2Phase 데이터 수집                                                                     │
│  ├─ Phase 1: 수동 티칭 → Joint Trajectory 녹화 (카메라 없이 경량 고속 녹화)               │
│  └─ Phase 2: Trajectory 재생 + 카메라 Observation 동시 녹화 → VLA 데이터셋 생성           │
│          │                                                                               │
│          ▼                                                                               │
│  ③ VLA 모델 학습                                                                          │
│  ├─ SmolVLA (SmolVLM2-500M 기반, 완료)                                                    │
│  └─ Pi-0 (PaliGemma 기반, 진행 중)                                                        │
│          │                                                                               │
│          ▼                                                                               │
│  ④ 추론 및 배포                                                                           │
│  ├─ 로컬 추론: 로봇 PC에서 직접 SmolVLA 추론 실행                                         │
│  └─ 원격 추론: SSH 터널 + ZeroMQ를 통해 GPU 서버에서 추론                                 │
│                                                                                          │
└──────────────────────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 시스템 구성

### 하드웨어

| 구성요소            | 사양                                              |
| ------------------- | ------------------------------------------------- |
| **로봇**      | OpenArm v0.3 Static Bimanual (양팔)               |
| **각 팔**     | 7 DOF + 1 Gripper = 8 관절 (총 16 관절)           |
| **카메라**    | Intel RealSense × 3 (상단, 좌측 손목, 우측 손목) |
| **로봇 PC**   | GPU 탑재 노트북 (ROS2 Humble)                     |
| **학습 서버** | NVIDIA H100 80GB / RTX 4090 24GB                  |

### 소프트웨어 스택

| 항목                     | 기술                                         |
| ------------------------ | -------------------------------------------- |
| **로봇 미들웨어**  | ROS2 Humble                                  |
| **중력보상**       | Pinocchio (RNEA 알고리즘)                    |
| **데이터 포맷**    | LeRobot v3.0 (HuggingFace)                   |
| **VLA 프레임워크** | lerobot 0.4.3                                |
| **VLA 모델**       | SmolVLA (SmolVLM2-500M), Pi-0 (PaliGemma-3B) |
| **원격 통신**      | ZeroMQ (REQ/REP 패턴) + SSH 터널             |

### 관절 배치 (16차원 State/Action)

```
State/Action Index Layout:
┌────────────────────────────────────────────────────────────┐
│          Left Arm                    Right Arm             │
├────────────────────────────────────────────────────────────┤
│ [0] left_rev1   (joint 1)    [8]  right_rev1  (joint 1)   │
│ [1] left_rev2   (joint 2)    [9]  right_rev2  (joint 2)   │
│ [2] left_rev3   (joint 3)    [10] right_rev3  (joint 3)   │
│ [3] left_rev4   (joint 4)    [11] right_rev4  (joint 4)   │
│ [4] left_rev5   (joint 5)    [12] right_rev5  (joint 5)   │
│ [5] left_rev6   (joint 6)    [13] right_rev6  (joint 6)   │
│ [6] left_rev7   (joint 7)    [14] right_rev7  (joint 7)   │
│ [7] left_rev8   (GRIPPER)    [15] right_rev8  (GRIPPER)   │
└────────────────────────────────────────────────────────────┘
```

### LeRobot v3.0 데이터셋 구조

| Feature                            | Shape         | 설명                               |
| ---------------------------------- | ------------- | ---------------------------------- |
| `observation.state`              | (16,)         | 16-DOF 조인트 위치                 |
| `action`                         | (16,)         | 다음 프레임 조인트 위치 (Absolute) |
| `observation.images.top`         | (256, 256, 3) | 상단 카메라                        |
| `observation.images.wrist_left`  | (256, 256, 3) | 왼쪽 손목 카메라                   |
| `observation.images.wrist_right` | (256, 256, 3) | 오른쪽 손목 카메라                 |
| `task`                           | string        | 자연어 작업 설명                   |

---

## 3. Phase 0: 중력보상 (Gravity Compensation)

### 개요

로봇 팔이 자중에 의해 쓰러지지 않도록 **중력에 대항하는 토크**를 실시간으로 계산하여 모터에 인가하는 기능. 이를 통해 사용자가 로봇 팔을 손으로 자유롭게 움직이면서 작업 경로(Trajectory)를 교시(Teaching)할 수 있음.

### 핵심 구현

| 항목                | 내용                                                                                         |
| ------------------- | -------------------------------------------------------------------------------------------- |
| **알고리즘**  | Pinocchio RNEA (Recursive Newton-Euler Algorithm)                                            |
| **입력**      | `/joint_states` (현재 관절 각도)                                                           |
| **출력**      | `/left_effort_controller/commands`, `/right_effort_controller/commands` (중력 보상 토크) |
| **제어 주기** | 100Hz (10ms)                                                                                 |
| **안전 기능** | 관절 한계 보호 토크, Rate Limiting, 외부 명령 타임아웃 저널                                  |

### 주요 코드

**파일**: `scripts/gravity_comp_node.py` (690줄)

```python
class GravityCompNode(Node):
    """중력보상 ROS2 노드"""
  
    # 주요 기능:
    # - compute_gravity_torque_pinocchio(): RNEA 기반 중력 토크 계산
    # - compute_limit_protection_torque(): 관절 한계 보호 토크
    # - rate_limit_position(): 위치 명령 Rate Limiting
    # - check_external_cmd_timeout(): 외부 명령 타임아웃 체크
    # - _handle_initial_move(): 초기 위치 이동 상태 머신
    # - control_loop(): 100Hz 메인 제어 루프
```

### 동작 모드

| 모드                       | 설명                                         | 파라미터                      |
| -------------------------- | -------------------------------------------- | ----------------------------- |
| **티칭 모드** (기본) | 중력보상만 적용, 사용자가 자유롭게 로봇 조작 | `enable_replay_mode:=false` |
| **리플레이 모드**    | 외부 위치 명령(VLA 추론 등) 수신 가능        | `enable_replay_mode:=true`  |
| **초기 이동**        | 시작 시 지정 위치로 자동 이동(보간)          | `enable_initial_move:=true` |

### Launch 파일

- `gravity_comp_teaching.launch.py`: 중력보상 티칭 전용
- `lerobot_trajectory_recording.launch.py`: 중력보상 + 데이터 녹화 통합

---

## 4. Phase 1: 2Phase VLA 모델 학습용 데이터 녹화

### 개요

VLA 모델 학습 데이터를 효율적으로 수집하기 위한 **2단계 데이터 수집 워크플로우**를 설계 및 구현.

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
│     • repeat_count로 데이터 증강 (동일 경로, 다른 조명/환경)          │
│     • 완전한 VLA 데이터셋 생성                                       │
└─────────────────────────────────────────────────────────────────────┘
```

### 이 방식의 장점

| 장점                  | 설명                                                       |
| --------------------- | ---------------------------------------------------------- |
| **반복 가능**   | 동일 trajectory를 여러 환경/조명에서 반복 녹화             |
| **품질 향상**   | 카메라 안정화 시간 확보, 데이터 일관성 보장                |
| **효율성**      | Phase 1은 빠르게 수집, Phase 2는 자동화                    |
| **데이터 증강** | `repeat_count`로 동일 경로를 반복 재생하여 데이터량 확대 |

### Phase 1: Trajectory 녹화

**핵심 코드**: `scripts/lerobot_trajectory_recorder.py`

- 중력보상 모드에서 사람이 로봇 팔을 직접 조작
- 키보드로 에피소드 녹화 시작(`r`), 저장(`s`), 종료(`q`)
- 키보드로 그리퍼 제어 (`keyboard_gripper_controller.py`)
- Resume 기능: 기존 데이터셋에 에피소드 추가 가능
- LeRobot v3.0 포맷으로 `observation.state` + `action` 저장

**실행 방법**: 3개 터미널 필요

```bash
# T1: 로봇 환경 + 중력보상
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py

# T2: 그리퍼 제어
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py

# T3: Trajectory 녹화
ros2 run openarm_static_bimanual_bringup lerobot_trajectory_recorder.py \
    --ros-args -p dataset_name:=pick_red_cube \
    -p task_description:="pick up the red cube and place it on the tray"
```

### Phase 2: VLA 데이터셋 생성

**핵심 코드**: `scripts/lerobot_vla_replay_recorder.py`, `launch/lerobot_vla_collection.launch.py`

- Phase 1 trajectory를 로봇에서 자동 재생
- 재생 중 3대의 RealSense 카메라로 영상 동시 녹화
- `repeat_count`로 데이터 증강 (예: 50 에피소드 × 10회 반복 = 500 데이터)
- `task_description`이 메타데이터에 저장되어 Multi-task 학습 지원

**실행 방법**: 2개 터미널 필요

```bash
# T1: 카메라 실행
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
    camera_name1:=cam_1 camera_name2:=cam_2 camera_name3:=cam_3 \
    serial_no1:='_346222072155' serial_no2:='_247122072494' serial_no3:='_247122074423'

# T2: VLA 데이터셋 생성 (자동)
ros2 launch openarm_static_bimanual_bringup lerobot_vla_collection.launch.py \
    trajectory_dataset:=~/lerobot_datasets/putting_umbrellas1 \
    vla_dataset:=~/lerobot_datasets/openarm_vla \
    task_description:="Put the umbrellas into the basket" \
    repeat_count:=10
```

### 수집된 태스크 목록

| Task Description                                                                        | Dataset Name      |
| --------------------------------------------------------------------------------------- | ----------------- |
| Move the basket to the right side and put the paper roll in the basket                  | moving_basket     |
| Put the umbrellas into the basket                                                       | putting_umbrellas |
| Move the Rubik's Cube on the right end to the left end using both arms                  | moving_cube       |
| Put the brown cup into the basket and put the green cup on the plate                    | putting_cups      |
| Open the first floor of the drawer and put the steel cup into the opened floor          | opening_drawer    |
| Pass the orange wine glass to the left arm to put the orange wine glass into the basket | passing_wine      |

### 기타 유틸리티

| 스크립트                     | 용도                                  |
| ---------------------------- | ------------------------------------- |
| `simple_state_replay.py`   | Phase 1 trajectory 단순 재생 (데모용) |
| `fake_camera_publisher.py` | Mock 카메라 (개발/테스트용)           |
| `parquet_to_csv.py`        | Parquet → CSV 변환                   |
| `test_image_to_server.py`  | 서버 추론 테스트용 이미지 전송        |

---

## 5. Phase 2: VLA 모델 학습

### 5.1 SmolVLA 학습 ✅ (완료)

#### 모델 정보

| 항목                  | 값                             |
| --------------------- | ------------------------------ |
| **기반 모델**   | SmolVLA (lerobot/smolvla_base) |
| **VLM 백본**    | SmolVLM2-500M-Video-Instruct   |
| **파라미터 수** | ~906MB                         |
| **State 차원**  | 16 (양팔 8 × 2)               |
| **Action 차원** | 16 (양팔 8 × 2)               |
| **이미지 입력** | 3개 카메라, 256×256           |

#### 학습 명령어

```bash
cd /home/highsky/lerobot_FMVLA

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=openarm_vla \
  --dataset.root=/home/highsky/openarm_vla \
  "--policy.input_features={...state: [16], images: [3,256,256]...}" \
  "--policy.output_features={action: [16]}" \
  --rename_map="{top→camera1, wrist_left→camera2, wrist_right→camera3}" \
  --batch_size=16 \
  --gradient_accumulation_steps=8 \
  --steps=45000 \
  --output_dir=outputs/train/smolvla_openarm600 \
  --wandb.enable=true \
  --save_freq=5000
```

#### 핵심 포인트

- 카메라 키 리네이밍: `observation.images.top` → `camera1`, `wrist_left` → `camera2`, `wrist_right` → `camera3`
- **16차원 State/Action 설정이 핵심** (기본 SmolVLA는 6차원)
- 체크포인트 저장 위치: `outputs/train/smolvla_openarm*/checkpoints/XXXXX/pretrained_model/`

---

### 5.2 Pi-0 학습 🔄 (진행 중)

#### 모델 정보

| 항목                    | 값                      |
| ----------------------- | ----------------------- |
| **기반 모델**     | Pi-0 (lerobot/pi0_base) |
| **VLM 백본**      | PaliGemma-3B            |
| **전체 파라미터** | 3.6B                    |
| **LoRA 파라미터** | 127M (3.5%)             |
| **State 차원**    | 16 (max_state_dim)      |
| **Action 차원**   | 16 (max_action_dim)     |

#### 환경 설정 특이사항

- **OpenPI 패치 필수**: Transformers의 `modeling_gemma.py`를 OpenPI 버전으로 교체해야 함
  - `GemmaRMSNorm.forward()`에 `cond` 파라미터 추가
  - `_gated_residual` 함수 추가
- **비디오 백엔드**: OpenCV 사용 (torchcodec/pyav 호환성 문제 회피)
- **HuggingFace**: PaliGemma 모델 접근 승인 필요

#### 학습 옵션 비교

| 옵션              | 학습 파라미터 | VRAM  | 예상 시간 | 권장          |
| ----------------- | ------------- | ----- | --------- | ------------- |
| Full Fine-Tuning  | 3.6B (100%)   | ~40GB | 6-7일     | H100 이상     |
| **LoRA** ⭐ | 127M (3.5%)   | ~12GB | 10-15시간 | RTX 4090 가능 |

#### LoRA 학습 명령어 (권장)

```bash
lerobot-train \
  --policy.path=lerobot/pi0_base \
  --dataset.repo_id=openarm_vla \
  --dataset.root=/data/khdw/openarm_vla \
  --rename_map='{top→base_0_rgb, wrist_left→left_wrist_0_rgb, wrist_right→right_wrist_0_rgb}' \
  --policy.use_lora=true \
  --policy.lora_rank=64 \
  --policy.lora_alpha=128 \
  --policy.merge_lora_on_save=true \
  --policy.freeze_vision_tower=true \
  --batch_size=16 \
  --gradient_accumulation_steps=4 \
  --steps=84755 \
  --policy.gradient_checkpointing=true \
  --policy.dtype=bfloat16 \
  --wandb.enable=true
```

#### 해결된 이슈들

| 이슈                                    | 원인                         | 해결                        |
| --------------------------------------- | ---------------------------- | --------------------------- |
| `TypeError: cond` unexpected argument | OpenPI 패치 미적용           | `modeling_gemma.py` 교체  |
| CUDA OOM (RTX 4090)                     | Full Fine-Tuning 메모리 부족 | LoRA 적용                   |
| 비디오 로딩 오류                        | torchcodec 비호환            | OpenCV 백엔드               |
| State dict 크기 불일치                  | LoRA 병합 문제               | `merge_lora_on_save=true` |

---

## 6. Phase 3: 추론 및 배포

### 6.1 로컬 추론 (로봇 PC에서 직접 추론) ✅

**핵심 코드**: `scripts/smolvla_inference_node.py` (ROS2 노드)

로봇 PC의 GPU에서 SmolVLA 모델을 직접 로드하여 실시간 추론 수행.

```
┌─────────────────────────────────────────────────────────────────────┐
│  로봇 PC (ROS2 Humble + CUDA GPU)                                    │
│                                                                      │
│  [카메라 × 3] → [smolvla_inference_node] → [gravity_comp_node]       │
│  [joint_states] ↗                          ↓                         │
│                                     [effort_controllers]             │
│                                            ↓                         │
│                                       [로봇 모터]                     │
└─────────────────────────────────────────────────────────────────────┘
```

#### 실행 방법

```bash
# T1: 로봇 + 중력보상 (Replay 모드)
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    active_arms:=both enable_replay_mode:=true

# T2: SmolVLA 추론
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/path/to/pretrained_model \
    task_description:="pick up the red block" \
    enable_control:=true
```

#### 주요 파라미터

| 파라미터                  | 기본값               | 설명               |
| ------------------------- | -------------------- | ------------------ |
| `policy_path`           | (필수)               | 체크포인트 경로    |
| `task_description`      | "pick up the object" | 자연어 태스크 설명 |
| `enable_control`        | false                | 로봇 제어 활성화   |
| `inference_rate`        | 10.0 Hz              | 추론 주파수        |
| `safety_velocity_limit` | 0.5 rad/s            | 안전 속도 제한     |

---

### 6.2 원격 추론 (서버 통신 기반) ✅

GPU가 부족한 로봇 PC 대신, **원격 GPU 서버**에서 VLA 추론을 수행하고 결과를 로봇으로 전달하는 시스템.

```
┌─────────────────┐    SSH 터널 (ZeroMQ)    ┌─────────────────┐
│   로봇 Laptop   │ ──────────────────────▶ │   GPU 서버      │
│                 │   이미지 + 상태 전송    │                 │
│   ROS2 노드     │ ◀────────────────────── │   VLA 추론      │
│   로봇 제어     │   16-dim 액션 수신      │   (SmolVLA/Pi0) │
└─────────────────┘                         └─────────────────┘
```

#### 아키텍처

**서버 측 (GPU 서버)**:

- `vla_server_inference/vla_inference_server.py`: ZeroMQ REP 소켓 기반 추론 서버
- SmolVLA 또는 Pi-0 모델 지원 (`--model_type` 선택)
- `start_server.sh`: 서버 시작 스크립트

**클라이언트 측 (로봇 PC)**:

- `scripts/vla_remote_client_node.py`: ROS2 노드, ZeroMQ REQ 소켓
- 카메라 이미지 + 관절 상태를 msgpack으로 직렬화하여 전송
- 서버로부터 16-dim 액션을 수신하여 중력보상 노드에 전달

#### 통신 프로토콜

```
[Client → Server]  msgpack 직렬화
{
    "images": {
        "camera1": bytes (JPEG 압축),
        "camera2": bytes,
        "camera3": bytes
    },
    "state": [16-dim float list],
    "task": "natural language instruction"
}

[Server → Client]  msgpack 직렬화
{
    "action": [16-dim float list],
    "inference_time": float,
    "status": "ok"
}
```

#### 실행 절차

```bash
# 1. 서버: VLA 추론 서버 실행
ssh user@서버IP
cd ~/OpenArm0.3_data/src/vla_server_inference
./start_server.sh /path/to/checkpoint

# 2. 로봇 PC: SSH 터널 생성
ssh -L 5555:localhost:5555 user@서버IP

# 3. 로봇 PC: 로봇 하드웨어 + 카메라 실행
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    enable_replay_mode:=true

# 4. 로봇 PC: 원격 추론 클라이언트 실행
ros2 launch openarm_static_bimanual_bringup vla_remote_inference.launch.py \
    task_description:="Move the basket to the right side" \
    enable_control:=true
```

#### 해결된 이슈들

| 이슈                            | 원인                            | 해결                            |
| ------------------------------- | ------------------------------- | ------------------------------- |
| Action 차원 불일치 (32D vs 16D) | 모델 출력과 통계 차원 불일치    | 액션 텐서를 16차원으로 슬라이싱 |
| ZeroMQ + ROS2 충돌              | libzmq와 ROS2 라이브러리 충돌   | 서버에서 pyzmq 독립 설치        |
| 키 이름 불일치                  | Client/Server 간 이미지 키 다름 | 키 매핑 통일                    |

---

## 7. 개발된 코드 파일 목록

### OpenArm0.3_data 워크스페이스

#### 📂 `src/openarm_static_bimanual_bringup/scripts/`

| 파일                               | 줄 수  | 설명                                                      |
| ---------------------------------- | ------ | --------------------------------------------------------- |
| `gravity_comp_node.py`           | 690    | Pinocchio 기반 중력보상 ROS2 노드                         |
| `lerobot_trajectory_recorder.py` | ~470   | Phase 1 Trajectory 녹화 스크립트                          |
| `lerobot_vla_replay_recorder.py` | ~1,050 | Phase 2 VLA 데이터셋 생성 (Trajectory 재생 + 카메라 녹화) |
| `smolvla_inference_node.py`      | ~730   | SmolVLA 로컬 추론 ROS2 노드                               |
| `vla_remote_client_node.py`      | ~500   | VLA 원격 추론 클라이언트 ROS2 노드                        |
| `simple_state_replay.py`         | ~460   | Trajectory 단순 재생 (데모용)                             |
| `keyboard_gripper_controller.py` | ~310   | 키보드-그리퍼 제어 노드                                   |
| `fake_camera_publisher.py`       | ~110   | Mock 카메라 퍼블리셔 (테스트용)                           |
| `trajectory_replay_node.py`      | ~200   | Trajectory 재생 보조 노드                                 |
| `test_image_to_server.py`        | ~300   | 서버 추론 테스트 클라이언트                               |

#### 📂 `src/openarm_static_bimanual_bringup/launch/`

| 파일                                       | 설명                                |
| ------------------------------------------ | ----------------------------------- |
| `sbopenarm.launch.py`                    | 로봇 기본 실행 (ros2_control)       |
| `lerobot_trajectory_recording.launch.py` | 중력보상 + Phase 1 데이터 녹화 통합 |
| `lerobot_vla_collection.launch.py`       | Phase 2 VLA 데이터셋 생성 자동화    |
| `smolvla_inference.launch.py`            | SmolVLA 로컬 추론 실행              |
| `vla_remote_inference.launch.py`         | VLA 원격 추론 클라이언트 실행       |
| `gravity_comp_teaching.launch.py`        | 중력보상 티칭 모드                  |

#### 📂 `src/vla_server_inference/`

| 파일                        | 설명                                          |
| --------------------------- | --------------------------------------------- |
| `vla_inference_server.py` | ZeroMQ 기반 VLA 추론 서버 (SmolVLA/Pi-0 지원) |
| `start_server.sh`         | 서버 시작 자동화 스크립트                     |
| `requirements.txt`        | Python 의존성 목록                            |

### lerobot_FMVLA 워크스페이스

lerobot 프레임워크를 수정하여 OpenArm 양팔 로봇에 맞게 커스터마이즈한 버전.

#### 주요 수정 사항

- SmolVLA / Pi-0 학습 파이프라인 커스터마이즈
- 16차원 State/Action 지원
- 카메라 키 리네이밍 로직
- OpenCV 비디오 백엔드 기본 설정
- LoRA 학습 지원 (Pi-0)

---

## 8. 작성된 가이드 문서 목록

| 문서                          | 경로                                             | 내용                                         |
| ----------------------------- | ------------------------------------------------ | -------------------------------------------- |
| **데이터 수집 가이드**  | `docs/TEACHING_AND_RECORDING_GUIDE.md`         | 2Phase 데이터 수집 워크플로우 전체           |
| **SmolVLA 배포 가이드** | `docs/SMOLVLA_DEPLOYMENT_GUIDE.md`             | SmolVLA 학습 → 체크포인트 배포 → 로컬 추론 |
| **원격 추론 가이드**    | `docs/VLA_REMOTE_INFERENCE_GUIDE.md`           | SSH 터널 + ZeroMQ 원격 추론 시스템           |
| **Pi-0 학습 가이드**    | `PI0_SERVER_TRAINING_GUIDE.md` (lerobot_FMVLA) | Pi-0 서버 환경 설정 → 학습 전체 과정        |
| **전체 작업 내역 종합** | `docs/PROJECT_WORK_SUMMARY.md`                 | (본 문서)                                    |

---

## 9. 향후 계획

### 단기 (진행 중)

- [ ] Pi-0 LoRA 학습 완료 및 체크포인트 검증
- [ ] Pi-0 추론 배포 테스트 (로컬 + 원격)

### 중기

- [ ] Pi-0, SmolVLA 성능 비교 평가

---

> 📝 **참고**: 각 단계의 상세한 사용법은 개별 가이드 문서를 참조하세요.
> 문의사항이 있으면 관련 가이드 문서의 트러블슈팅 섹션을 먼저 확인해주세요.
