# VLA 원격 추론 가이드 (VLA Remote Inference Guide)

> 📅 작성일: 2026-02-09
> 🎯 목적: 로봇 laptop과 원격 GPU 서버 간 VLA 추론 통신 설정 및 실행

---

## 📋 개요

이 가이드는 SSH 터널을 통해 원격 GPU 서버에서 VLA(SmolVLA 또는 Pi0) 모델 추론을 수행하고,
그 결과를 로봇 laptop으로 전달하여 실시간 로봇 제어를 가능하게 하는 방법을 설명합니다.

```
┌─────────────────┐    SSH 터널 (ZeroMQ)    ┌─────────────────┐
│   로봇 Laptop   │ ──────────────────────▶ │   GPU 서버      │
│                 │   이미지 + 상태 전송    │                 │
│   ROS2 노드     │ ◀────────────────────── │   VLA 추론      │
│   로봇 제어     │   16-dim 액션 수신      │   (SmolVLA/Pi0) │
└─────────────────┘                         └─────────────────┘
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

### Step 2: 서버 환경 설정 (서버에서 실행)

```bash
# 1. 서버 SSH 접속
ssh user@서버IP

# 2. Conda 환경 생성 (최초 1회)
conda create -n vla_server python=3.10 -y
conda activate vla_server

# 3. 의존성 설치
cd ~/OpenArm0.3_data/src/vla_server_inference
pip install -r requirements.txt

# 4. 체크포인트 확인
ls ~/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/pretrained_model/
```

---

### Step 3: VLA 서버 실행 (서버에서 실행)

```bash
# 방법 1: start_server.sh 스크립트 사용 (권장)
cd ~/OpenArm0.3_data/src/vla_server_inference

# SmolVLA 실행 (기본값)
./start_server.sh /path/to/smolvla_checkpoint

# Pi0 실행 (MODEL_TYPE 환경변수 설정)
MODEL_TYPE=pi0 ./start_server.sh /path/to/pi0_checkpoint --debug

# 방법 2: Python 직접 실행
conda activate vla_server
python vla_inference_server.py \
    --policy_path /path/to/checkpoint \
    --port 5555 \
    --model_type pi0 \
    --debug
```

서버가 정상 실행되면 다음과 같은 메시지가 표시됩니다:

```
============================================================
  🤖 VLA Inference Server - smolvla (or pi0)
============================================================
  ✅ 서버 준비 완료! 클라이언트 연결 대기 중...
```

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
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py enable_replay:=true

# 터미널 2: 카메라 실행
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
    cam_1_serial_no:=317322073024 \
    cam_2_serial_no:=326522073051 \
    cam_3_serial_no:=327322071339
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
> - "Put the brown cup into the basket and put the green cup on the plate" (dataset name: putting_cups)
> - "Open the first floor of the drawer and put the steel cup into the opened floor" (dataset name: opening_drawer)
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

---

## 📁 파일 구조

```
OpenArm0.3_data/
├── src/
│   ├── vla_server_inference/          # 서버 측 코드 (scp로 전송)
│   │   ├── vla_inference_server.py    # ZeroMQ 서버 + SmolVLA 추론
│   │   ├── requirements.txt           # Python 의존성
│   │   └── start_server.sh            # 서버 시작 스크립트
│   │
│   └── openarm_static_bimanual_bringup/
│       ├── scripts/
│       │   └── vla_remote_client_node.py  # ROS2 클라이언트 노드
│       └── launch/
│           └── vla_remote_inference.launch.py
│
├── checkpoints/
│   └── smolvla_openarm_16dim/
│       └── pretrained_model/          # SmolVLA 체크포인트
│
└── docs/
    └── VLA_REMOTE_INFERENCE_GUIDE.md  # 이 문서
```

---

## ✅ 실행 체크리스트

- [ ] 서버에 OpenArm0.3_data 폴더 복사 완료
- [ ] 서버 Conda 환경 및 의존성 설치 완료
- [ ] VLA 서버 정상 실행 확인
- [ ] SSH 터널 연결 확인
- [ ] 로봇 하드웨어 및 카메라 실행
- [ ] Dry-run 테스트 성공 (enable_control:=false)
- [ ] 실제 로봇 제어 테스트 (enable_control:=true)
