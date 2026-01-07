# OpenArm 양팔 로봇 그리퍼 제어 및 데이터 수집 통합 가이드

**최종 업데이트**: 2026-01-03

이 문서는 OpenArm 양팔 로봇의 그리퍼 키보드 제어, 트래젝토리 데이터 녹화, VLA 모델 학습용 데이터 수집 기능에 대한 총정리 가이드입니다.

---

## 목차

1. [개요](#개요)
2. [파일 구조](#파일-구조)
3. [빌드 방법](#빌드-방법)
4. [실행 방법](#실행-방법)
5. [각 노드 상세 설명](#각-노드-상세-설명)
6. [주요 토픽 구조](#주요-토픽-구조)
7. [데이터 형식](#데이터-형식)
8. [문제 해결](#문제-해결)

---

## 개요

이 시스템은 다음 4가지 핵심 기능을 제공합니다:

| 기능                           | 스크립트                           | 설명                                          |
| ------------------------------ | ---------------------------------- | --------------------------------------------- |
| **중력보상 티칭**        | `gravity_comp_node.py`           | 양팔 로봇의 중력보상 모드로 수동 티칭 가능    |
| **데이터 녹화**          | `continuous_recorder_node.py`    | 조인트 상태를 JSON 형식으로 녹화              |
| **그리퍼 키보드 제어**   | `keyboard_gripper_controller.py` | 키보드(q/w/o/p)로 양팔 그리퍼 제어            |
| **VLA 학습 데이터 수집** | `fmvla_data_record.py`           | LeRobot Parquet 형식으로 VLA 학습 데이터 수집 |
| **트래젝토리 재생**      | `trajectory_replay_node.py`      | 녹화된 JSON 트래젝토리 재생                   |

---

## 파일 구조

```
/home/highsky/openarm_official_ws/src/
├── openarm_static_bimanual_bringup/
│   ├── scripts/
│   │   ├── gravity_comp_node.py         # 중력보상 노드
│   │   ├── continuous_recorder_node.py  # 트래젝토리 녹화 (JSON)
│   │   ├── keyboard_gripper_controller.py  # 그리퍼 키보드 제어
│   │   ├── fmvla_data_record.py         # VLA 학습 데이터 녹화 (Parquet)
│   │   └── trajectory_replay_node.py    # 트래젝토리 재생
│   ├── launch/
│   │   └── gravity_comp_teaching.launch.py  # 메인 Launch 파일
│   └── CMakeLists.txt                   # 빌드 설정
│
└── openarm_arduino_bridge/
    └── openarm_arduino_bridge/
        └── bimanual_bridge_node.py      # Arduino 그리퍼 브릿지
```

---

## 빌드 방법

### 1단계: 워크스페이스 이동

```bash
cd ~/openarm_official_ws
```

### 2단계: 패키지 빌드

```bash
colcon build --packages-select openarm_static_bimanual_bringup openarm_arduino_bridge --symlink-install
```

### 3단계: 환경 설정 (매 터미널마다 실행)

```bash
source ~/openarm_official_ws/install/setup.bash
```

> **Tip**: `.bashrc`에 추가하면 자동으로 적용됩니다:
>
> ```bash
> echo "source ~/openarm_official_ws/install/setup.bash" >> ~/.bashrc
> ```

---

## 실행 방법

### 전체 실행 흐름

```
┌─────────────────────────────────────────────────────────────────────┐
│                          터미널 1 (Launch)                           │
│  ros2 launch openarm_static_bimanual_bringup                        │
│      gravity_comp_teaching.launch.py use_mock_hardware:=true        │
├─────────────────────────────────────────────────────────────────────┤
│                          터미널 2 (녹화)                             │
│  ros2 run openarm_static_bimanual_bringup continuous_recorder_node.py│
│  [키: r=녹화시작, s=저장, q=종료]                                     │
├─────────────────────────────────────────────────────────────────────┤
│                          터미널 3 (그리퍼)                           │
│  ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py│
│  [키: q/w=왼쪽 열기/닫기, o/p=오른쪽 열기/닫기]                       │
├─────────────────────────────────────────────────────────────────────┤
│                          터미널 4 (VLA 녹화)                         │
│  ros2 run openarm_static_bimanual_bringup fmvla_data_record.py       │
│  [키: r=녹화, s=에피소드저장, f=완료, q=종료]                         │
└─────────────────────────────────────────────────────────────────────┘
```

---

### 터미널 1: Launch 파일 실행 (중력보상 + 기본 구성)

#### Mock 하드웨어 (시뮬레이션)

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    use_mock_hardware:=true
```

#### 실제 하드웨어

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py
```

#### 그리퍼 브릿지 포함 (실제 Arduino 연결 시)

```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    enable_gripper_bridge:=true \
    servo_port:=/dev/ttyACM0
```

**Launch 인자:**

| 인자                      | 기본값    | 설명                             |
| ------------------------- | --------- | -------------------------------- |
| `use_mock_hardware`     | `false` | Mock 하드웨어 사용 여부          |
| `enable_recorder`       | `true`  | Recorder 노드 launch에 포함 여부 |
| `enable_gripper_bridge` | `false` | Arduino 그리퍼 브릿지 활성화     |
| `servo_port`            | `auto`  | Arduino 시리얼 포트              |
| `active_arms`           | `both`  | 활성 팔 (left/right/both)        |

---

### 터미널 2: 데이터 녹화 (키보드 입력 필요)

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 run openarm_static_bimanual_bringup continuous_recorder_node.py
```

**키보드 조작:**

| 키    | 동작              |
| ----- | ----------------- |
| `r` | 녹화 시작         |
| `s` | 녹화 중지 및 저장 |
| `q` | 종료              |

**저장 위치**: `~/openarm_official_ws/joint_trajectory_YYYYMMDD_HHMMSS.json`

---

### 터미널 3: 그리퍼 키보드 제어 (키보드 입력 필요)

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

**키보드 조작:**

| 키    | 동작               |
| ----- | ------------------ |
| `q` | 왼쪽 그리퍼 열기   |
| `w` | 왼쪽 그리퍼 닫기   |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| ESC   | 종료               |

> **중요**: 이 노드는 반드시 **별도 터미널**에서 실행해야 키보드 입력이 정상 작동합니다.

---

### 터미널 4: VLA 학습 데이터 녹화 (키보드 입력 필요)

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 run openarm_static_bimanual_bringup fmvla_data_record.py
```

**키보드 조작:**

| 키    | 동작                         |
| ----- | ---------------------------- |
| `r` | 에피소드 녹화 시작           |
| `s` | 현재 에피소드 저장           |
| `f` | 데이터셋 완료 (Parquet 저장) |
| `q` | 종료                         |

**저장 위치**: `~/openarm_official_ws/vla_data/dataset_YYYYMMDD_HHMMSS.parquet`

---

### 트래젝토리 재생

녹화된 JSON 파일을 재생합니다:

```bash
source ~/openarm_official_ws/install/setup.bash
ros2 run openarm_static_bimanual_bringup trajectory_replay_node.py \
    --ros-args -p trajectory_file:=/path/to/joint_trajectory.json
```

**파라미터:**

| 파라미터            | 기본값    | 설명           |
| ------------------- | --------- | -------------- |
| `trajectory_file` | (필수)    | JSON 파일 경로 |
| `playback_speed`  | `1.0`   | 재생 속도 배율 |
| `loop`            | `false` | 반복 재생 여부 |

---

## 각 노드 상세 설명

### 1. keyboard_gripper_controller.py

키보드 입력으로 양팔 그리퍼를 제어하는 노드입니다.

**발행 토픽:**

- `/left_gripper_cmd` (std_msgs/Float64)
- `/right_gripper_cmd` (std_msgs/Float64)

**파라미터:**

- `gripper_speed`: 초당 위치 변화량 (기본: 0.5)
- `publish_rate`: 발행 주기 Hz (기본: 20.0)
- `min_gripper`: 최소 그리퍼 위치 (기본: 0.0)
- `max_gripper`: 최대 그리퍼 위치 (기본: 1.0)

---

### 2. continuous_recorder_node.py

조인트 상태를 JSON 형식으로 녹화하는 노드입니다.

**구독 토픽:**

- `/joint_states` (sensor_msgs/JointState)
- `/gripper_states` (sensor_msgs/JointState)

**파라미터:**

- `record_rate`: 녹화 주기 Hz (기본: 50.0)
- `save_dir`: 저장 디렉토리 (기본: ~/openarm_official_ws)
- `file_format`: 저장 형식 json/npy (기본: json)

---

### 3. fmvla_data_record.py

VLA 모델 학습용 데이터를 LeRobot 형식으로 수집하는 노드입니다.

**구독 토픽:**

- `/joint_states` (sensor_msgs/JointState)
- `/gripper_states` (sensor_msgs/JointState)

**데이터 구조 (16 DOF):**

```
left_arm[7] + left_gripper[1] + right_arm[7] + right_gripper[1]
```

**파라미터:**

- `repo_id`: 데이터셋 ID (기본: openarm/bimanual_dataset)
- `root_dir`: 저장 디렉토리 (기본: ~/openarm_official_ws/vla_data)
- `fps`: 녹화 FPS (기본: 50.0)
- `push_to_hub`: HuggingFace Hub 업로드 여부 (기본: false)

---

### 4. trajectory_replay_node.py

녹화된 JSON 트래젝토리를 재생하는 노드입니다.

**발행 토픽:**

- `/left_joint_trajectory_controller/joint_trajectory`
- `/right_joint_trajectory_controller/joint_trajectory`
- `/left_gripper_cmd`
- `/right_gripper_cmd`

---

## 주요 토픽 구조

```
/joint_states                    ← 14 DOF 양팔 조인트 상태
/gripper_states                  ← 2 DOF 그리퍼 상태 (Arduino 브릿지)
/left_gripper_cmd                → 왼쪽 그리퍼 명령 (Float64)
/right_gripper_cmd               → 오른쪽 그리퍼 명령 (Float64)
/left_effort_controller/commands → 왼팔 토크 명령
/right_effort_controller/commands → 오른팔 토크 명령
```

---

## 데이터 형식

### JSON (continuous_recorder_node.py)

```json
{
  "metadata": {
    "record_rate": 50.0,
    "total_samples": 1000,
    "duration_sec": 20.0,
    "recorded_at": "20260103_165000"
  },
  "data": [
    {
      "timestamp": 0.02,
      "joint_names": ["left_rev1", "left_rev2", ..., "right_gripper_joint"],
      "positions": [0.0, 0.1, ..., 0.5],
      "velocities": [0.0, 0.0, ...],
      "efforts": [0.0, 0.0, ...]
    }
  ]
}
```

### Parquet (fmvla_data_record.py)

LeRobot Dataset v3.0 호환 형식:

| 컬럼                  | 타입        | 설명               |
| --------------------- | ----------- | ------------------ |
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

### 'package not found' 에러

**해결**:

```bash
cd ~/openarm_official_ws
colcon build --packages-select openarm_static_bimanual_bringup openarm_arduino_bridge
source install/setup.bash
```

---

### pyarrow 미설치 (VLA 녹화 시 JSON으로 저장됨)

**해결**:

```bash
pip install pyarrow
```

---

### Arduino 브릿지 연결 실패

**확인**:

```bash
ls /dev/ttyACM*
# 또는
ls /dev/ttyUSB*
```

**해결**: `servo_port` 파라미터에 올바른 포트 지정

```bash
ros2 launch ... enable_gripper_bridge:=true servo_port:=/dev/ttyACM0
```

---

## 참고 자료

- [OpenArm 공식 GitHub](https://github.com/openarm)
- [LeRobot Dataset 형식](https://huggingface.co/docs/lerobot)
- [ROS2 Humble 문서](https://docs.ros.org/en/humble/)
