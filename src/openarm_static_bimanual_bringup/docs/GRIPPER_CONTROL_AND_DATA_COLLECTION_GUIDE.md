# OpenArm 양팔 로봇 그리퍼 제어 및 데이터 수집 통합 가이드

**최종 업데이트**: 2026-01-08

이 문서는 OpenArm 양팔 로봇의 그리퍼 키보드 제어, 트래젝토리 데이터 녹화, VLA 모델 학습용 데이터 수집 기능에 대한 총정리 가이드입니다.

---

## 목차

1. [개요](#개요)
2. [하드웨어 구성](#하드웨어-구성)
3. [파일 구조](#파일-구조)
4. [빌드 방법](#빌드-방법)
5. [실행 방법 (단계별)](#실행-방법-단계별)
6. [각 노드 상세 설명](#각-노드-상세-설명)
7. [주요 토픽 구조](#주요-토픽-구조)
8. [데이터 형식](#데이터-형식)
9. [문제 해결](#문제-해결)

---

## 개요

이 시스템은 다음 5가지 핵심 기능을 제공합니다:

| 기능                     | 스크립트                           | 설명                                          |
| ------------------------ | ---------------------------------- | --------------------------------------------- |
| **중력보상 티칭**        | `gravity_comp_node.py`             | 양팔 로봇의 중력보상 모드로 수동 티칭 가능    |
| **데이터 녹화**          | `continuous_recorder_node.py`      | 조인트 상태를 JSON 형식으로 녹화              |
| **그리퍼 키보드 제어**   | `keyboard_gripper_controller.py`   | 키보드(q/w/o/p)로 양팔 그리퍼 제어            |
| **VLA 학습 데이터 수집** | `fmvla_data_record.py`             | LeRobot Parquet 형식으로 VLA 학습 데이터 수집 |
| **트래젝토리 재생**      | `trajectory_replay_node.py`        | 녹화된 JSON 트래젝토리 재생                   |

---

## 하드웨어 구성

### 그리퍼 서보모터

| 항목           | 스펙                          |
| -------------- | ----------------------------- |
| **모델명**     | DOMAN DM-CLS400MD             |
| **타입**       | 디지털 코어리스 서보          |
| **회전 각도**  | 180° (PWM 500-2500µs)         |
| **토크**       | 38.8 kg.cm @ 6V / 40.6 kg.cm @ 7.4V |
| **속도**       | 0.15 sec/60° @ 6V             |
| **인터페이스** | JR (25T 출력축)               |

### 통신 구조

```
Arduino (Serial 115200bps)
    ↕ /dev/ttyACM0
bimanual_bridge_node.py
    ↕ ROS2 Topics
keyboard_gripper_controller.py / gravity_comp_node.py
```

---

## 파일 구조

```
~/OpenArm0.3_data/src/
├── openarm_static_bimanual_bringup/
│   ├── scripts/
│   │   ├── gravity_comp_node.py            # 중력보상 노드 (관절별 스케일 지원)
│   │   ├── continuous_recorder_node.py     # 트래젝토리 녹화 (JSON)
│   │   ├── keyboard_gripper_controller.py  # 그리퍼 키보드 제어 (초기 동기화 지원)
│   │   ├── fmvla_data_record.py            # VLA 학습 데이터 녹화 (Parquet)
│   │   └── trajectory_replay_node.py       # 트래젝토리 재생
│   ├── launch/
│   │   └── gravity_comp_teaching.launch.py # 메인 Launch 파일
│   └── docs/
│       └── GRIPPER_CONTROL_AND_DATA_COLLECTION_GUIDE.md
│
└── openarm_arduino_bridge/
    └── openarm_arduino_bridge/
        └── bimanual_bridge_node.py         # Arduino 그리퍼 브릿지
```

---

## 빌드 방법

### 1단계: 워크스페이스 이동

```bash
cd ~/OpenArm0.3_data
```

### 2단계: 패키지 빌드

```bash
colcon build --packages-select openarm_static_bimanual_bringup openarm_arduino_bridge openarm_static_bimanual_hardware --symlink-install
```

### 3단계: 환경 설정 (매 터미널마다 실행)

```bash
source ~/OpenArm0.3_data/install/setup.bash
```

> **Tip**: `.bashrc`에 추가하면 자동으로 적용됩니다:
>
> ```bash
> echo "source ~/OpenArm0.3_data/install/setup.bash" >> ~/.bashrc
> ```

---

## 실행 방법 (단계별)

### 🚀 전체 실행 타임라인

```
[0초]   터미널 1: Launch 파일 실행
           ↓
[3초]   effort_controller 스폰 완료
           ↓
[5초]   gravity_comp_node 시작
           ↓
[6초]   gripper_bridge_node 시작
           ↓
[8초]   터미널 2: keyboard_gripper_controller 실행
           ↓
[~13초] 그리퍼 상태 동기화 완료 (또는 5초 타임아웃)
           ↓
[준비완료] 키보드로 그리퍼 제어 가능!
```

---

### 📌 STEP 1: Launch 파일 실행 (터미널 1)

#### 실제 하드웨어 + 그리퍼 브릿지

```bash
# 터미널 1
source ~/OpenArm0.3_data/install/setup.bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    enable_gripper_bridge:=true \
    servo_port:=/dev/ttyACM0
```

> ⏱️ **대기**: 이 명령 실행 후 **최소 8초** 대기 (gripper_bridge 초기화 완료까지)

#### Mock 하드웨어 (시뮬레이션)

```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    use_mock_hardware:=true
```

#### Launch 인자 설명

| 인자                    | 기본값  | 설명                         |
| ----------------------- | ------- | ---------------------------- |
| `use_mock_hardware`     | `false` | Mock 하드웨어 사용 여부      |
| `enable_recorder`       | `true`  | Recorder 노드 포함 여부      |
| `enable_gripper_bridge` | `false` | Arduino 그리퍼 브릿지 활성화 |
| `servo_port`            | `auto`  | Arduino 시리얼 포트          |
| `active_arms`           | `both`  | 활성 팔 (left/right/both)    |

---

### 📌 STEP 2: 그리퍼 키보드 제어 시작 (터미널 2)

> ⚠️ **중요**: Launch 후 **반드시 8초 이상 대기** 후 실행하세요!

```bash
# 터미널 2 (새 터미널 열기)
source ~/OpenArm0.3_data/install/setup.bash
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

#### 시작 시 출력 메시지

```
[INFO] === Keyboard Gripper Controller ===
[INFO]   Waiting for gripper state sync from Arduino bridge...
[INFO]   'q' = Left open,  'w' = Left close
[INFO]   'o' = Right open, 'p' = Right close
[INFO]   ESC or Ctrl+C to quit
```

#### 동기화 완료 시 (약 1-5초 후)

```
[INFO] ✅ Synced with Arduino bridge: L=0.000, R=0.000
[INFO]   Ready for keyboard control!
```

> 만약 5초 내 동기화 실패 시:
> ```
> [WARN] Sync timeout (5.0s). Using default position 0.0 (open)
> ```

#### 키보드 조작

| 키  | 동작               |
| --- | ------------------ |
| `q` | 왼쪽 그리퍼 열기   |
| `w` | 왼쪽 그리퍼 닫기   |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| ESC | 종료               |

---

### 📌 STEP 3 (선택): 데이터 녹화 (터미널 3)

```bash
# 터미널 3 (새 터미널 열기)
source ~/OpenArm0.3_data/install/setup.bash
ros2 run openarm_static_bimanual_bringup continuous_recorder_node.py
```

#### 키보드 조작

| 키  | 동작              |
| --- | ----------------- |
| `r` | 녹화 시작         |
| `s` | 녹화 중지 및 저장 |
| `q` | 종료              |

**저장 위치**: `~/OpenArm0.3_data/joint_trajectory_YYYYMMDD_HHMMSS.json`

---

### 📌 STEP 4 (선택): VLA 학습 데이터 녹화 (터미널 4)

```bash
# 터미널 4 (새 터미널 열기)
source ~/OpenArm0.3_data/install/setup.bash
ros2 run openarm_static_bimanual_bringup fmvla_data_record.py
```

#### 키보드 조작

| 키  | 동작                         |
| --- | ---------------------------- |
| `r` | 에피소드 녹화 시작           |
| `s` | 현재 에피소드 저장           |
| `f` | 데이터셋 완료 (Parquet 저장) |
| `q` | 종료                         |

**저장 위치**: `~/OpenArm0.3_data/vla_data/dataset_YYYYMMDD_HHMMSS.parquet`

---

## 각 노드 상세 설명

### 1. keyboard_gripper_controller.py

키보드 입력으로 양팔 그리퍼를 제어하는 노드입니다.

**주요 기능:**
- Arduino 브릿지와 **초기 위치 자동 동기화** (점프 현상 방지)
- 동기화 타임아웃 시 기본값(0.0) 사용

**발행 토픽:**
- `/left_gripper_cmd` (std_msgs/Float64)
- `/right_gripper_cmd` (std_msgs/Float64)

**구독 토픽:**
- `/gripper_states` (sensor_msgs/JointState) - 초기 동기화용

**파라미터:**

| 파라미터       | 기본값 | 설명                    |
| -------------- | ------ | ----------------------- |
| `gripper_speed`| 0.5    | 초당 위치 변화량        |
| `publish_rate` | 20.0   | 발행 주기 Hz            |
| `min_gripper`  | 0.0    | 최소 그리퍼 위치 (열림) |
| `max_gripper`  | 1.0    | 최대 그리퍼 위치 (닫힘) |
| `sync_timeout` | 5.0    | 동기화 대기 시간 (초)   |

---

### 2. gravity_comp_node.py

Pinocchio 기반 중력보상을 수행하는 노드입니다.

**주요 기능:**
- **관절별 개별 중력 스케일** 지원 (rev5~7에 더 강한 보상)
- 관절 한계 보호 (가상 스프링)

**파라미터:**

| 파라미터               | 기본값                              | 설명                        |
| ---------------------- | ----------------------------------- | --------------------------- |
| `gravity_scale_joints` | [1.5, 1.5, 1.5, 1.5, 1.8, 1.8, 1.8] | 관절별 중력 스케일 [rev1~7] |
| `publish_rate`         | 100.0                               | 제어 주기 Hz                |
| `enable_limit_protection` | true                             | 관절 한계 보호 활성화       |

---

### 3. continuous_recorder_node.py

조인트 상태를 JSON 형식으로 녹화하는 노드입니다.

**구독 토픽:**
- `/joint_states` (sensor_msgs/JointState)
- `/gripper_states` (sensor_msgs/JointState)

**파라미터:**
- `record_rate`: 녹화 주기 Hz (기본: 50.0)
- `save_dir`: 저장 디렉토리 (기본: ~/OpenArm0.3_data)
- `file_format`: 저장 형식 json/npy (기본: json)

---

### 4. fmvla_data_record.py

VLA 모델 학습용 데이터를 LeRobot 형식으로 수집하는 노드입니다.

**데이터 구조 (16 DOF):**
```
left_arm[7] + left_gripper[1] + right_arm[7] + right_gripper[1]
```

---

## 주요 토픽 구조

```
/joint_states                     ← 14 DOF 양팔 조인트 상태
/gripper_states                   ← 2 DOF 그리퍼 상태 (Arduino 브릿지)
/left_gripper_cmd                 → 왼쪽 그리퍼 명령 (Float64: 0.0~1.0)
/right_gripper_cmd                → 오른쪽 그리퍼 명령 (Float64: 0.0~1.0)
/left_effort_controller/commands  → 왼팔 토크 명령
/right_effort_controller/commands → 오른팔 토크 명령
```

### 그리퍼 값 범위 매핑

| 레이어                    | 값 범위   | 의미         |
| ------------------------- | --------- | ------------ |
| keyboard_controller       | 0.0 ~ 1.0 | 열림 ~ 닫힘  |
| Arduino 브릿지 (시리얼)   | 0 ~ 60    | 정수 변환    |
| 서보 PWM                  | 500 ~ 2500µs | 180° 범위 |

---

## 데이터 형식

### JSON (continuous_recorder_node.py)

```json
{
  "metadata": {
    "record_rate": 50.0,
    "total_samples": 1000,
    "duration_sec": 20.0,
    "recorded_at": "20260107_133000"
  },
  "data": [
    {
      "timestamp": 0.02,
      "joint_names": ["left_rev1", ..., "left_gripper_joint", ..., "right_gripper_joint"],
      "positions": [0.0, 0.1, ..., 0.5],
      "velocities": [...],
      "efforts": [...]
    }
  ]
}
```

### Parquet (fmvla_data_record.py)

LeRobot Dataset v3.0 호환 형식:

| 컬럼                | 타입        | 설명               |
| ------------------- | ----------- | ------------------ |
| `timestamp`         | float64     | 시간 (초)          |
| `frame_index`       | int32       | 프레임 인덱스      |
| `episode_index`     | int32       | 에피소드 인덱스    |
| `observation.state` | float32[16] | 현재 조인트 상태   |
| `action`            | float32[16] | 액션 (상태 변화량) |

---

## 문제 해결

### 키보드 입력이 안 됨

**원인**: Launch 터미널에서 실행하면 키보드 입력이 비활성화됩니다.

**해결**: 키보드 입력이 필요한 노드는 반드시 **별도 터미널**에서 `ros2 run`으로 실행하세요.

---

### 그리퍼가 시작 시 갑자기 움직임 (점프 현상)

**원인**: 이전 버전에서는 초기값이 0.5로 고정되어 있었습니다.

**해결**: 최신 버전에서는 `/gripper_states` 토픽을 구독하여 Arduino 브릿지의 현재 상태와 **자동 동기화**됩니다. 최신 코드로 업데이트하세요.

---

### 'package not found' 에러

**해결**:

```bash
cd ~/OpenArm0.3_data
colcon build --packages-select openarm_static_bimanual_bringup openarm_arduino_bridge --symlink-install
source install/setup.bash
```

---

### Arduino 브릿지 연결 실패

**확인**:

```bash
ls /dev/ttyACM*
# 또는
ls /dev/ttyUSB*
```

**권한 문제 해결**:

```bash
sudo usermod -a -G dialout $USER
# 로그아웃 후 재로그인 필요
```

**해결**: `servo_port` 파라미터에 올바른 포트 지정

```bash
ros2 launch ... enable_gripper_bridge:=true servo_port:=/dev/ttyACM0
```

---

### 그리퍼 동기화 타임아웃

**원인**: keyboard_gripper_controller를 너무 일찍 실행했거나 Arduino 브릿지가 시작되지 않음

**해결**:
1. Launch 실행 후 **최소 8초** 대기
2. Arduino 연결 상태 확인: `ls /dev/ttyACM*`
3. 브릿지 로그 확인: `ros2 topic echo /gripper_states`

---

### 로봇팔이 굳어버리거나 Ctrl+C 후 토크가 유지됨

**원인 1 (Limit 도달)**: 관절이 소프트웨어 Limit에 도달하면 가상 스프링 토크가 발생합니다.

**해결**: `limit_spring_k` 값을 낮추거나 (기본 5.0 → 3.0), Limit 범위를 확장하세요.

**원인 2 (종료 처리)**: 이전 버전에서는 노드 종료 시 0 토크를 보내지 않았습니다.

**해결**: 최신 버전에서는 `gravity_comp_node`가 종료 시 자동으로 0 토크를 발행합니다. 최신 코드로 업데이트하세요.

---

### CAN 버퍼 오버플로우 (모터 굳음/초록불)

**증상**: 수동 조작 중 일부 모터가 갑자기 굳어버리고 초록 LED가 켜짐. 토픽 발행이 중단됨.

**원인**: CAN 수신 버퍼가 가득 차서 새 패킷이 드롭되고 모터가 Fault 모드에 진입.

**해결 (시스템 버퍼 확대)**:

```bash
# 일시 적용 (재부팅 시 초기화)
sudo sysctl -w net.core.rmem_default=1048576
sudo sysctl -w net.core.rmem_max=2097152

# 적용 확인
sysctl net.core.rmem_default
```

**해결 (영구 적용)**:

```bash
echo "net.core.rmem_default=1048576" | sudo tee -a /etc/sysctl.conf
echo "net.core.rmem_max=2097152" | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

> **참고**: 하드웨어 패키지(`openarm_static_bimanual_hardware`)에는 이미 소켓 버퍼 512KB 설정과 recv 횟수 증가(25회) 최적화가 적용되어 있습니다. 반드시 빌드 후 사용하세요.

#### 🔧 문제가 지속될 경우: 고급 디버깅

위 조치 후에도 문제가 발생하면 다음 파라미터를 순차적으로 조정하세요.

**1. CAN 버퍼 상태 실시간 모니터링**:

```bash
# CAN 인터페이스 통계 확인 (RX-ERR, TX-ERR 증가 시 버퍼 문제)
watch -n 1 'ip -s link show can0'

# 드롭된 패킷 수 확인
cat /proc/net/can/stats
```

**2. 조정 가능한 파라미터**:

| 파라미터 | 파일 위치 | 현재값 | 권장 조정 | 효과 |
|----------|-----------|--------|----------|------|
| recv 루프 횟수 | `openarm_hw_flex.cpp:174` | 25 | → 35~50 | 버퍼 더 빠르게 비움 |
| 소켓 버퍼 크기 | `canbus.cpp:20` | 512KB | → 1MB | 더 많은 패킷 수용 |
| recv 타임아웃 | `canbus.cpp:23` | 1ms | → 0.5ms | 루프 속도 향상 |
| 시스템 버퍼 | sysctl | 1MB | → 2~4MB | 커널 수준 확대 |

**3. 코드 수정 예시**:

```cpp
// openarm_hw_flex.cpp:174 - recv 횟수 증가
for (int i = 0; i < 40; ++i) {  // 25 → 40

// canbus.cpp:20 - 버퍼 1MB로 확대
int rcvbuf = 1048576;  // 512KB → 1MB

// canbus.cpp:23 - 타임아웃 0.5ms로 축소
::timeval tv{0, 500};  // 1ms → 0.5ms
```

**4. 수정 후 반드시 재빌드**:

```bash
cd ~/OpenArm0.3_data
colcon build --packages-select openarm_static_bimanual_hardware
source install/setup.bash
```

> ⚠️ **주의**: recv 횟수를 50 이상으로 늘리면 제어 루프 타이밍에 영향을 줄 수 있습니다. 점진적으로 조정하세요.

---

## 참고 자료

- [OpenArm 공식 GitHub](https://github.com/openarm)
- [LeRobot Dataset 형식](https://huggingface.co/docs/lerobot)
- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)
- [DOMAN Servo Datasheet](https://domanrchobby.com)
