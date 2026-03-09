# VLA 원격 추론 가이드 (VLA Remote Inference Guide)

> 📅 작성일: 2026-02-09
> 🎯 목적: 로봇 laptop과 원격 GPU 서버 간 VLA 추론 통신 설정 및 실행

---

## 📋 개요

이 가이드는 SSH 터널을 통해 원격 GPU 서버에서 VLA(SmolVLA, Pi0, GROOT N1.5, FMVLA) 모델 추론을 수행하고,
그 결과를 로봇 laptop으로 전달하여 실시간 로봇 제어를 가능하게 하는 방법을 설명합니다.

```
┌─────────────────┐    SSH 터널 (ZeroMQ)    ┌──────────────────────┐
│   로봇 Laptop   │ ──────────────────────▶ │   GPU 서버           │
│                 │   이미지 + 상태 전송    │                      │
│   ROS2 노드     │ ◀────────────────────── │   VLA 추론           │
│   로봇 제어     │   16-dim 액션 수신      │ (SmolVLA/Pi0/GROOT/  │
│                 │                         │          FMVLA)      │
└─────────────────┘                         └──────────────────────┘
```

---

## 🚀 실행 단계

### Step 1: 서버 파일 전송 (로봇 laptop에서 실행)

```bash
# OpenArm0.3_data 폴더를 서버로 전송
scp -r ~/OpenArm0.3_data user@서버IP:~/

# 또는 SSH config 설정된 경우
scp -r ~/OpenArm0.3_data gpu-server:~/
```

---

### Step 2: 서버 환경 설정 (서버에서 실행, 최초 1회)

```bash
# 1) 서버 SSH 접속
ssh user@서버IP

# 2) 작업 디렉토리 이동
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
```

#### 2-1) SmolVLA / Pi0용 환경 (`vla_server`)

```bash
conda create -n vla_server python=3.10 -y
conda activate vla_server
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
pip install -r requirements.txt
```

#### 2-2) GROOT N1.5용 환경 (`vla_server_groot`)

```bash
conda create -n vla_server_groot python=3.10 -y
conda activate vla_server_groot
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
pip install -r requirements.txt

# 필요 시 lerobot groot extra 설치
# cd /path/to/lerobot_0211_0.4.3_openarm
# pip install -e ".[groot]"
```

#### 2-3) FMVLA용 환경 (`vla_server_fmvla`)

```bash
conda create -n vla_server_fmvla python=3.10 -y
conda activate vla_server_fmvla

# FMVLA 지원 lerobot 코드 설치 (예: 커스텀 lerobot_0211_0.4.3_openarm)
cd /path/to/lerobot_0211_0.4.3_openarm
pip install -e .

# 서버 통신 의존성 설치
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
pip install pyzmq msgpack numpy opencv-python pillow
```

#### 2-4) FMVLA 체크포인트 확인

```bash
ls /datastore/khdw/OpenArm0.3_data/checkpoints/fmvla_openarm_0213_1614/checkpoints/022500/pretrained_model
ls /datastore/khdw/OpenArm0.3_data/checkpoints/V_P_OpenARM_0209_2332/checkpoint-27110/pytorch_lora_weights.safetensors
```

---

### Step 3: VLA 서버 실행 (서버에서 실행)

#### 3-1) SmolVLA / Pi0: `start_server.sh` 사용 (`vla_server`)

```bash
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
conda activate vla_server

# SmolVLA 실행 (기본값)
./start_server.sh /datastore/khdw/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/020000/pretrained_model

# Pi0 실행
MODEL_TYPE=pi0 ./start_server.sh /datastore/khdw/OpenArm0.3_data/checkpoints/pi0_lora_20260209_080912/checkpoints/last/pretrained_model --debug
```

#### 3-2) GROOT N1.5: Python 직접 실행 (`vla_server_groot`)

```bash
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
conda activate vla_server_groot

python vla_inference_server.py \
    --policy_path /datastore/khdw/OpenArm0.3_data/checkpoints/groot_run_full_finetune_v1/checkpoints/last/pretrained_model \
    --port 5555 \
    --model_type groot \
    --debug
```

#### 3-3) FMVLA: Python 직접 실행 (`vla_server_fmvla`)

```bash
cd /datastore/khdw/OpenArm0.3_data/src/vla_server_inference
conda activate vla_server_fmvla

python vla_inference_server.py \
    --policy_path /datastore/khdw/OpenArm0.3_data/checkpoints/fmvla_openarm_0213_1614/checkpoints/022500/pretrained_model \
    --model_type fmvla \
    --port 5555 \
    --fmvla_lora_weights_path /datastore/khdw/OpenArm0.3_data/checkpoints/V_P_OpenARM_0209_2332/checkpoint-27110/pytorch_lora_weights.safetensors \
    --fmvla_hold_on_chunk_boundary \
    --fmvla_hold_max_sec 0 \
    --fmvla_lora_scale 1.0 \
    --debug

# 필요 시 추가 옵션:
#   --fmvla_precomputed_dir /path/to/precomputed_dir
#   --fmvla_precomputed_only
#   --no-fmvla_enable_sd3_cpu_offload
#   --no-fmvla_hold_on_chunk_boundary   # 기존 즉시 재예측 방식
```

> ⚠️ **중요**: `start_server.sh`는 `smolvla/pi0`만 지원합니다. `groot/fmvla`는 반드시 각 전용 환경에서 `python vla_inference_server.py`로 직접 실행하세요.
>
> ✅ **환경 분리 권장**:
>
> - `vla_server`: SmolVLA/Pi0
> - `vla_server_groot`: GROOT N1.5
> - `vla_server_fmvla`: FMVLA

서버가 정상 실행되면 다음과 같은 메시지가 표시됩니다:

```
============================================================
  🤖 VLA Inference Server
============================================================
  ✅ 정책 로드 완료!
  ⏳ 클라이언트 요청 대기 중...
```

> ⚠️ **GROOT N1.5 참고사항**:
>
> - 첫 실행 시 `nvidia/GR00T-N1.5-3B` base 모델을 HuggingFace에서 다운로드합니다 (인터넷 필요)
> - 추론에 약 6-8GB VRAM이 필요합니다
> - `flash-attn` 패키지는 CUDA 환경에서만 설치 가능합니다
>
> ⚠️ **FMVLA 참고사항**:
>
> - 첫 실행 시 SD3/관련 가중치 로딩으로 초기 지연이 발생할 수 있습니다.
> - `--fmvla_lora_weights_path`를 지정하면 config 내부 경로를 덮어써서 서버 로컬 경로에 맞출 수 있습니다.
> - 기본값으로 `--fmvla_hold_on_chunk_boundary`가 활성화되어, action queue 경계에서 SD3 chunk 생성 중 마지막 action을 유지합니다.
> - `--debug`로 실행하면 생성된 모든 subgoal image가 서버의 `src/vla_server_inference/fmvla_subgoals/<세션시각>/subgoal_000001.png` 형태로 추가 저장됩니다.
> - 위 archive 저장은 **새 action chunk가 생성될 때마다** 1장씩 누적됩니다. 요청 수와 1:1은 아닙니다.

---

### Step 4: SSH 터널 생성 (로봇 laptop에서 실행)

```bash
# SSH 포트 포워딩 (새 터미널에서 실행)
# user@서버IP: 원격 GPU 서버의 계정과 IP를 입력해야 합니다. (로봇 노트북 계정 아님!)
# 예: ssh -L 5555:localhost:5555 dongwoo@163.152.193.246(0번 server)
ssh -L 5555:localhost:5555 dongwoo@163.152.193.246

# 또는 백그라운드 모드
ssh -fN -L 5555:localhost:5555 dongwoo@163.152.193.246

### ✅ 터널링 성공 확인
1. **일반 모드**: 서버에 로그인되어 프롬프트(예: `(base) dongwoo@server:~$`)가 뜨면 성공입니다. 창을 닫으면 연결이 끊깁니다.
2. **백그라운드 모드**: 아무 메시지 없이 바로 명령 프롬프트가 떨어지면 성공입니다. 에러가 날 경우에만 메시지가 뜹니다.
```

> ⚠️ **중요**: SSH 터널은 VLA 추론 동안 유지되어야 합니다. `Ctrl+C`로 종료하면 연결이 끊어집니다.

---

### Step 5: 로봇 hardware 실행 (로봇 laptop에서 실행)

**⚠️ 주의: `vla_remote_inference.launch.py`에는 `initial_move` 로직이 없습니다!**

```bash
# 터미널 1: 로봇 하드웨어 및 초기화 (Step 3 trajectory recording 환경)
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py enable_replay_mode:=true

# 터미널 2: 카메라 실행
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
    serial_no1:="'_317322073024'" \
    serial_no2:="'_326522073051'" \
    serial_no3:="'_327322071339'"
```

```bash
# 권장: 640x480@30 명시 (수집/추론 해상도 일관성 확보)
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
    serial_no1:="'_317322073024'" \
    serial_no2:="'_326522073051'" \
    serial_no3:="'_327322071339'" \
    rgb_camera.color_profile1:=640,480,30 \
    rgb_camera.color_profile2:=640,480,30 \
    rgb_camera.color_profile3:=640,480,30
```

### Step 5-1: FMVLA용 `top view`를 512x512 정방형 `.mp4`로 저장 (선택)

원격 VLA 추론과 별개로, FMVLA가 사용하는 `top view`와 같은 전처리 방식으로 영상을 남기고 싶다면 아래 standalone recorder를 사용합니다.

- 대상 토픽: `/camera/cam_1/color/image_raw/compressed`
- 메시지 타입: `sensor_msgs/msg/CompressedImage`
- 전처리 방식: 원본 `cam_1` 프레임을 **정방형으로 direct resize**
- 추론 입력 크기: `256x256` (`vla_remote_inference.launch.py` 의 `image_size:=256`)
- 저장 영상 크기: `512x512` (`cam1_mp4_recorder.py --record-size 512`)
- 출력 형식: H.264 `.mp4`
- 파일명 규칙: `fmvla_top512_YYYYMMDD_HHMMSS_ffffff_pidPID.mp4`
- 파일 저장 시작 시점: **첫 정상 프레임 수신 후**

```bash
# 새 터미널에서 실행
source /opt/ros/humble/setup.bash

python3 /home/highsky/cam1_mp4_recorder.py \
    --topic /camera/cam_1/color/image_raw/compressed \
    --output-dir ~/ros2_recordings/fmvla_top \
    --prefix fmvla_top512 \
    --record-size 512 \
    --fps 30.0
```

> ✅ 권장 실행 순서:
>
> 1. Step 5의 하드웨어/카메라를 먼저 실행
> 2. 위 recorder를 실행
> 3. Step 6의 VLA 추론을 `image_size:=256` 으로 실행

#### recorder 동작 상세

- `CompressedImage`를 decode 한 뒤, FMVLA와 같은 방식으로 **정방형 resize** 해서 `ffmpeg`로 바로 인코딩합니다.
- 현재 FMVLA 추론은 `256x256` 입력을 사용하지만, 저장 영상은 시인성을 위해 `512x512`로 남깁니다.
- 즉, **전처리 정책은 같고 저장 해상도만 더 큽니다.**
- 같은 초에 여러 번 실행해도 마이크로초(`ffffff`)와 PID가 붙어 파일명이 겹치지 않습니다.
- 녹화는 지정 FPS(`--fps`) 기준의 고정 FPS MP4로 저장됩니다.
- 원본 카메라 해상도가 바뀌어도 저장 파일은 계속 `512x512` 정방형입니다.
- 토픽이 안 오면 파일이 만들어지지 않습니다. 첫 프레임이 와야 인코더가 시작됩니다.

#### 안전 종료

- 반드시 `Ctrl+C`로 종료하세요.
- 정상 종료 시 `ffmpeg`가 마무리되면서 MP4 메타데이터가 flush 됩니다.
- `kill -9` 같은 강제 종료는 깨진 MP4를 남길 수 있습니다.

#### 빠른 검증 명령

```bash
# 토픽과 타입 확인
source /opt/ros/humble/setup.bash
ros2 topic info /camera/cam_1/color/image_raw/compressed

# FMVLA 추론은 256x256 입력 유지
ros2 launch openarm_static_bimanual_bringup vla_remote_inference.launch.py \
    image_size:=256 \
    enable_control:=false \
    debug:=true

# 녹화 파일 확인
ls -lh ~/ros2_recordings/fmvla_top

# 저장된 mp4 메타데이터 확인 (해상도 512x512 확인)
ffprobe -hide_banner ~/ros2_recordings/fmvla_top/<생성된파일명>.mp4
```

---

### Step 6: VLA 추론 실행 (로봇 laptop에서 실행)

**Dry-run 테스트 (로봇 제어 비활성화):**

```bash
ros2 launch openarm_static_bimanual_bringup vla_remote_inference.launch.py \
    task_description:="Move the basket to the right side and put the paper roll in the basket" \
    enable_control:=false \
    debug:=true
```

**실제 로봇 제어:**

```bash
ros2 launch openarm_static_bimanual_bringup vla_remote_inference.launch.py \
    task_description:="Move the basket to the right side and put the paper roll in the basket" \
    enable_control:=true \
    debug:=true
```

**taks_description list**

> - "Move the basket to the right side and put the paper roll into the basket" (dataset name: moving_basket)
> - "Put the umbrellas into the basket" (dataset name: putting_umbrellas)
> - "Move the Rubik's Cube on the right end to the left end using both arms" (dataset name: moving_cube)
> - "Open the first floor of the drawer and put the steel cup into the opened floor" (dataset name: opening_drawer)
> - "Put the brown cup into the basket and put the green cup on the plate" (dataset name: putting_cups)
> - "Pass the orange wine glass to the left arm to put the orange wine glass into the basket" (dataset name: passing_wine)

---

## 📊 파라미터 설명

| 파라미터             | 기본값              | 설명                                  |
| -------------------- | ------------------- | ------------------------------------- |
| `server_port`      | 5555                | ZeroMQ 서버 포트 (SSH 터널 로컬 포트) |
| `inference_rate`   | 10.0                | 추론 요청 주기 (Hz)                   |
| `enable_control`   | false               | 로봇 제어 활성화 여부                 |
| `task_description` | "manipulation task" | VLA 모델에 전달할 태스크 설명         |
| `debug`            | true                | 디버그 로그 출력                      |
| `timeout_ms`       | 5000                | 서버 응답 타임아웃 (ms)               |

---

## 🔧 트러블슈팅

### 1. ZeroMQ 연결 실패

```bash
# SSH 터널 확인
lsof -i :5555  # 로컬 포트 사용 확인

# 서버에서 포트 확인
netstat -tlnp | grep 5555
```

### 2. 서버 응답 타임아웃

- GPU 메모리 부족: `nvidia-smi`로 확인
- 모델 로딩 지연: 첫 요청에서 웜업 필요
- 네트워크 지연: SSH 터널 상태 확인

### 3. 이미지/상태 누락

```bash
# ROS2 토픽 확인
ros2 topic list
ros2 topic hz /camera/cam_1/color/image_raw/compressed
ros2 topic hz /joint_states
```

### 4. `cam1_mp4_recorder.py`가 파일을 만들지 않음

```bash
# 1) ROS 환경 source 확인
source /opt/ros/humble/setup.bash

# 2) 토픽 실제 수신 확인
ros2 topic info /camera/cam_1/color/image_raw/compressed
ros2 topic hz /camera/cam_1/color/image_raw/compressed

# 3) ffmpeg 설치 확인
ffmpeg -version
```

- `cam1_mp4_recorder.py`는 **첫 프레임을 받은 뒤에만** `.mp4` 파일을 생성합니다.
- 토픽 타입은 반드시 `sensor_msgs/msg/CompressedImage` 여야 합니다.
- `--record-size 512` 로 실행하면 저장 영상은 항상 `512x512` 여야 합니다.
- FMVLA 추론 입력 크기는 별도이며, `vla_remote_inference.launch.py image_size:=256` 와 혼동하지 마세요.
- 비정상 종료한 경우 `.mp4`가 재생되지 않을 수 있으니 `Ctrl+C` 정상 종료를 사용하세요.

---

## 📁 파일 구조

```
OpenArm0.3_data/
├── src/
│   ├── vla_server_inference/          # 서버 측 코드 (scp로 전송)
│   │   ├── vla_inference_server.py    # ZeroMQ 서버 + SmolVLA/Pi0/GROOT/FMVLA 추론
│   │   ├── requirements.txt           # Python 의존성
│   │   └── start_server.sh            # smolvla/pi0 전용 시작 스크립트
│   │
│   └── openarm_static_bimanual_bringup/
│       ├── scripts/
│       │   └── vla_remote_client_node.py  # ROS2 클라이언트 노드
│       └── launch/
│           └── vla_remote_inference.launch.py
│
├── checkpoints/
│   ├── smolvla_openarm_16dim/...      # SmolVLA 체크포인트
│   ├── pi0_lora_20260209_080912/...   # Pi0 체크포인트
│   ├── groot_run_full_finetune_v1/... # GROOT 체크포인트
│   ├── fmvla_openarm_0213_1614/...    # FMVLA 체크포인트
│   └── V_P_OpenARM_0209_2332/
│       └── checkpoint-27110/pytorch_lora_weights.safetensors  # FMVLA Vision Planner LoRA
│
└── docs/
    └── VLA_REMOTE_INFERENCE_GUIDE.md  # 이 문서
```

---

## ✅ 실행 체크리스트

- [ ] 서버에 OpenArm0.3_data 폴더 복사 완료
- [ ] `vla_server`/`vla_server_groot`/`vla_server_fmvla` 환경 구성 완료
- [ ] 모델별 서버 실행 방식 확인 (`start_server.sh`: smolvla/pi0, python 직접 실행: groot/fmvla)
- [ ] VLA 서버 정상 실행 확인 (모델별 최소 1회)
- [ ] SSH 터널 연결 확인
- [ ] 로봇 하드웨어 및 카메라 실행
- [ ] 필요 시 `python3 /home/highsky/cam1_mp4_recorder.py --record-size 512 ...` 로 FMVLA top-view mp4 녹화 시작
- [ ] Dry-run 테스트 성공 (enable_control:=false)
- [ ] 실제 로봇 제어 테스트 (enable_control:=true)
