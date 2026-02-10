# 🚀 SmolVLA 추론 배포 가이드

> 📅 작성일: 2026-02-05
> 🎯 목표: 학습된 SmolVLA 체크포인트를 양팔로봇 제어 컴퓨터에 배포하여 실시간 추론 실행

---

## 📚 참조

- 학습 체크포인트: `lerobot_FMVLA/outputs/train/smolvla_openarm_16dim/checkpoints/XXXXX/pretrained_model/`
- lerobot 0.4.3 (pip 설치 버전)
- OpenArm 양팔 로봇: 각 팔 7 DOF + 1 Gripper = 16개 관절

---

## 🔌 체크포인트 구성 (재학습 후)

| 항목                  | 값                           | 설명                                    |
| --------------------- | ---------------------------- | --------------------------------------- |
| **State 차원**  | 16                           | 양팔 각 8개 관절 (7 arm + 1 gripper)    |
| **Action 차원** | 16                           | 양팔 각 8개 관절 명령                   |
| **이미지 크기** | 256×256                     | 3개 카메라                              |
| **카메라 키**   | camera1, camera2, camera3    | top, wrist_left, wrist_right에서 rename |
| **VLM 모델**    | SmolVLM2-500M-Video-Instruct | 언어 이해용                             |

---

## 📋 관절 배치 (16차원)

```
State/Action Index Layout:
┌────────────────────────────────────────────────────────────┐
│          Left Arm                    Right Arm            │
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

---

## 🔧 배포 단계

### Step 1: 16차원 재학습 실행

먼저 16차원 state/action 설정으로 재학습을 진행합니다:

```bash
cd /home/highsky/lerobot_FMVLA

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --dataset.repo_id=openarm_vla \
  --dataset.root=/home/highsky/openarm_vla \
  "--policy.input_features={\"observation.state\": {\"type\": \"STATE\", \"shape\": [16]}, \"observation.images.camera1\": {\"type\": \"VISUAL\", \"shape\": [3, 256, 256]}, \"observation.images.camera2\": {\"type\": \"VISUAL\", \"shape\": [3, 256, 256]}, \"observation.images.camera3\": {\"type\": \"VISUAL\", \"shape\": [3, 256, 256]}}" \
  "--policy.output_features={\"action\": {\"type\": \"ACTION\", \"shape\": [16]}}" \
  --rename_map="{\"observation.images.top\": \"observation.images.camera1\", \"observation.images.wrist_left\": \"observation.images.camera2\", \"observation.images.wrist_right\": \"observation.images.camera3\"}" \
  --batch_size=16 \
  --gradient_accumulation_steps=8 \
  --steps=45000 \
  --output_dir=outputs/train/smolvla_openarm600 \
  --job_name=smolvla_openarm_16dim_training \
  --policy.device=cuda \
  --wandb.enable=true \
  --save_freq=5000 \
  --policy.push_to_hub=false
```

> [!IMPORTANT]
> `--policy.input_features.observation.state.shape='[16]'`와 `--policy.output_features.action.shape='[16]'` 옵션이 핵심입니다!

---

### Step 2: 체크포인트 복사

학습 완료 후 로봇 제어 컴퓨터로 체크포인트를 복사합니다:

```bash
# 학습 컴퓨터에서 (source)
scp -r /home/highsky/lerobot_FMVLA/outputs/train/smolvla_openarm_16dim/checkpoints/XXXXX/pretrained_model \
    robot@robot-pc:/home/robot/smolvla_checkpoint/

# 또는 rsync 사용 (권장)
rsync -avz --progress \
    /home/highsky/lerobot_FMVLA/outputs/train/smolvla_openarm_16dim/checkpoints/XXXXX/pretrained_model/ \
    robot@robot-pc:/home/robot/smolvla_checkpoint/
```

복사해야 할 파일 목록:

```text
pretrained_model/
├── config.json
├── model.safetensors           (~906MB)
├── policy_postprocessor.json
├── policy_postprocessor_step_0_unnormalizer_processor.safetensors
├── policy_preprocessor.json
├── policy_preprocessor_step_5_normalizer_processor.safetensors
└── train_config.json
```

---

### Step 3: 로봇 컴퓨터 환경 확인

```bash
# lerobot 버전 확인 (0.4.3 필요)
pip show lerobot

# 필요한 의존성 확인
pip show transformers torch safetensors

# GPU 사용 가능 여부
python -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

> [!WARNING]
> lerobot 0.4.3 이하 버전에서는 SmolVLA 지원이 제한적일 수 있습니다.
> 호환성 문제 발생 시 lerobot 업그레이드를 권장합니다:
> `pip install --upgrade lerobot`

---

### Step 4: ROS2 패키지 빌드

```bash
# OpenArm 워크스페이스로 이동
cd /path/to/OpenArm0.3_data

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 패키지 빌드
colcon build --packages-select openarm_static_bimanual_bringup --symlink-install

# 설치 환경 로드
source install/setup.bash
```

---

### Step 5: 추론 실행

#### 5.1 Dry-run 테스트 (필수!)

먼저 제어 명령 없이 추론만 테스트:

```bash
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/home/robot/smolvla_checkpoint \
    task_description:="pick up the object" \
    enable_control:=false
```

예상 출력:

```text
[smolvla_inference_node]: ✅ SmolVLA Inference Node initialized
[smolvla_inference_node]:    Policy: /home/robot/smolvla_checkpoint
[smolvla_inference_node]:    Device: cuda
[smolvla_inference_node]: ⏳ Waiting for: cameras: [...], joint states
[smolvla_inference_node]: 🚀 All sensors ready! Starting inference...
[smolvla_inference_node]: 📊 Inference #50: 45.2ms, action: [0.1, -0.2, ..., 0.05] (16-dim)
```

#### 5.2 양팔 실제 로봇 제어

Gravity compensation 모드 활성화 후:

```bash
# 터미널 1: 로봇 하드웨어 및 중력 보상 실행 (Replay Mode)
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    active_arms:=both \
    enable_replay_mode:=true

# 터미널 2: SmolVLA 추론 실행 (양팔 제어)
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/home/robot/smolvla_checkpoint \
    task_description:="pick up the red block" \
    enable_control:=true \
    control_arm:=both
```

---

## ⚙️ 파라미터 설명

| 파라미터                  | 기본값               | 설명                        |
| ------------------------- | -------------------- | --------------------------- |
| `policy_path`           | (필수)               | 체크포인트 경로             |
| `task_description`      | "pick up the object" | 태스크 설명 (자연어)        |
| `device`                | "cuda"               | 추론 장치                   |
| `inference_rate`        | 10.0                 | 추론 주파수 (Hz)            |
| `enable_control`        | **false**      | 로봇 제어 활성화            |
| `safety_velocity_limit` | 0.5                  | 최대 관절 속도 (rad/s)      |
| `control_arm`           | "both"               | 제어할 팔 (left/right/both) |

---

## 🔄 실행 중 태스크 변경

```bash
ros2 topic pub /vla/task_description std_msgs/msg/String "data: 'place the object on the table'"
```

---

## 🐛 트러블슈팅

### 문제: 정책 로드 실패

```text
❌ Failed to load policy: No such file or directory
```

**해결:**

- `policy_path`가 `pretrained_model` 폴더를 가리키는지 확인
- `config.json` 파일이 존재하는지 확인

### 문제: 16차원 Action 오류

```text
❌ Expected 16-dim action, got 6-dim. Check checkpoint config!
```

**해결:**

- 재학습 시 `--policy.input_features.observation.state.shape='[16]'` 옵션 확인
- 기존 6-dim 체크포인트를 사용 중인지 확인

### 문제: 카메라 토픽 대기 중

```text
⏳ Waiting for: cameras: ['observation.images.top', ...]
```

**해결:**

- 카메라 노드가 실행 중인지 확인
- 토픽 이름 확인: `ros2 topic list | grep camera`

### 문제: CUDA 메모리 부족

```text
CUDA out of memory
```

**해결:**

- 다른 GPU 프로세스 종료
- `device:=cpu`로 CPU 모드 시도 (느림)

---

## ⚠️ 안전 주의사항

> [!CAUTION]
>
> 1. **항상 dry-run 테스트 먼저!** `enable_control:=false`
> 2. E-STOP 버튼을 항상 손에 닿는 곳에 배치
> 3. 처음에는 `safety_velocity_limit:=0.3`으로 낮게 시작
> 4. 작업 영역 내 장애물 제거
> 5. 양팔 동시 제어 시 충돌 주의 (`control_arm:=both`)

---

## 💡 핵심 요약

```bash
# 1. 16차원 재학습 (중요!)
lerobot-train   --policy.path=lerobot/smolvla_base   --dataset.repo_id=openarm_vla   --dataset.root=/home/highsky/openarm_vla   --policy.input_features='{"observation.state": {"type": "STATE", "shape": [16]}, "observation.images.camera1": {"type": "VISUAL", "shape": [3, 256, 256]}, "observation.images.camera2": {"type": "VISUAL", "shape": [3, 256, 256]}, "observation.images.camera3": {"type": "VISUAL", "shape": [3, 256, 256]}}'   --policy.output_features='{"action": {"type": "ACTION", "shape": [16]}}'   --rename_map='{"observation.images.top": "observation.images.camera1", "observation.images.wrist_left": "observation.images.camera2", "observation.images.wrist_right": "observation.images.camera3"}'   --batch_size=4   --steps=20000   --output_dir=outputs/train/smolvla_openarm_16dim   --job_name=smolvla_openarm_16dim_training   --policy.device=cuda   --wandb.enable=true   --policy.push_to_hub=false

# 2. 체크포인트 복사
scp -r pretrained_model/ robot@robot-pc:/home/robot/smolvla_checkpoint/

# 3. 로봇 컴퓨터에서 빌드
colcon build --packages-select openarm_static_bimanual_bringup

## 3. 추론 실행 (Inference)

SmolVLA 추론을 실행하려면 **두 개의 터미널**이 필요합니다. 하나는 로봇 하드웨어 제어 및 중력 보상을 실행하고, 다른 하나는 SmolVLA 모델 추론을 실행합니다.

### 터미널 1: 로봇 하드웨어 및 중력 보상 실행 (Replay Mode)

먼저 로봇 드라이버와 중력 보상 노드를 실행합니다. **`enable_replay_mode:=true`**로 설정하여 외부(SmolVLA)로부터 제어 명령을 받을 수 있도록 해야 합니다.

> [!IMPORTANT]
> **`lerobot_trajectory_recording.launch.py`를 사용하세요!** 이 파일에는 `enable_replay_mode` 옵션이 **이미 포함**되어 있으며, 초기 위치 이동(`enable_initial_move`) 기능도 지원합니다.

```bash
# 로봇 하드웨어 실행 (Replay Mode)
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py \
    active_arms:=both \
    enable_replay_mode:=true
```

| 주요 파라미터             | 기본값 | 설명                                                    |
| ------------------------- | ------ | ------------------------------------------------------- |
| `enable_replay_mode`    | false  | 외부 제어 명령 수신 활성화 (추론 시**true 필수**) |
| `enable_initial_move`   | true   | 시작 시 초기 위치로 자동 이동                           |
| `initial_move_duration` | 3.0    | 초기 위치 이동 시간 (초)                                |
| `active_arms`           | both   | 제어할 팔 선택                                          |

### 터미널 2: SmolVLA 추론 실행

새로운 터미널에서 SmolVLA 추론 노드를 실행합니다.

**1단계: Dry-Run (안전 테스트)**
먼저 로봇을 실제로 움직이지 않고 추론 루프가 정상적으로 작동하는지 확인합니다.

```bash
# Dry-run execution
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/path/to/16dim_checkpoint \
    task_description:="pick up the object" \
    enable_control:=false
```

**2단계: 실제 제어 (Real Control)**
Dry-run이 성공하면 `enable_control:=true`로 설정하여 실제로 로봇을 제어합니다.

```bash
# Real robot control
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/path/to/16dim_checkpoint \
    task_description:="pick up the object" \
    enable_control:=true \
    safety_velocity_limit:=0.5
```

### 문제 해결 (Troubleshooting)

- **로봇이 움직이지 않음:**

  - 터미널 1에서 `enable_replay_mode:=true`가 설정되었는지 확인하세요.
  - 터미널 2에서 `enable_control:=true`인지 확인하세요.
  - 별도의 터미널에서 `ros2 topic echo /gravity_comp/left_external_position_cmd`를 실행하여 추론 노드가 명령을 발행하고 있는지 확인하세요.
- **"Dimension mismatch" 오류:**

  - `policy_path`가 올바른 16차원 모델을 가리키는지 확인하세요.

## 📐 체크포인트 검증 스크립트

재학습 완료 후 체크포인트의 차원을 확인하는 스크립트:

```python
import json
from safetensors import safe_open

checkpoint_path = "/path/to/pretrained_model"

# config.json 확인
with open(f"{checkpoint_path}/config.json") as f:
    config = json.load(f)
  
print("=== Config 분석 ===")
print(f"State shape: {config['input_features']['observation.state']['shape']}")
print(f"Action shape: {config['output_features']['action']['shape']}")

# Normalizer 통계 확인
normalizer_path = f"{checkpoint_path}/policy_preprocessor_step_5_normalizer_processor.safetensors"
with safe_open(normalizer_path, framework="pt") as f:
    state_mean = f.get_tensor("observation.state.mean")
    action_mean = f.get_tensor("action.mean")
    print(f"\n=== Normalizer 실제 차원 ===")
    print(f"State dim: {state_mean.shape[0]} (expected: 16)")
    print(f"Action dim: {action_mean.shape[0]} (expected: 16)")
```
