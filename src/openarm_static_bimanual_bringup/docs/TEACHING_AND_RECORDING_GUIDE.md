# OpenArm Bimanual 중력 보상 및 데이터 수집 가이드

이 문서는 OpenArm Static Bimanual 로봇의 중력 보상 모드, LeRobot 데이터 녹화, 그리퍼 제어 사용법을 설명합니다.

---

## 목차

1. [중력 보상 모드](#1-중력-보상-모드)
2. [LeRobot VLA 데이터 녹화](#2-lerobot-vla-데이터-녹화)
3. [키보드 그리퍼 제어](#3-키보드-그리퍼-제어)
4. [데이터 재생](#4-데이터-재생)

---

## 1. 중력 보상 모드

로봇 팔을 손으로 자유롭게 조작할 수 있는 Teaching 모드입니다.

### 실행 방법

```bash
# 터미널 1: 중력 보상 + 로봇 구동
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py
```

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `use_mock_hardware` | `false` | 시뮬레이션 모드 |
| `active_arms` | `both` | 활성화할 팔 (`left`, `right`, `both`) |
| `rviz` | `true` | RViz 시각화 |

### 사용 예시

```bash
# 왼쪽 팔만 활성화
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py active_arms:=left

# 시뮬레이션 모드
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py use_mock_hardware:=true
```

---

## 2. LeRobot VLA 데이터 녹화

VLA 모델 학습용 데이터를 LeRobot parquet 형식으로 녹화합니다.

### 실행 방법

**2개의 터미널이 필요합니다:**

```bash
# 터미널 1: 중력 보상 + 녹화 시스템
ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py

# 터미널 2: 그리퍼 제어 (별도 창)
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

### 녹화 키보드 조작 (터미널 1)

| 키 | 동작 |
|:--:|------|
| `r` | 새 에피소드 녹화 시작 |
| `s` | 현재 에피소드 저장 |
| `q` | 데이터셋 저장 후 종료 |

### 녹화 워크플로우

1. 터미널 1에서 launch 파일 실행
2. 터미널 2에서 그리퍼 컨트롤러 실행
3. 터미널 1에서 `r` 키로 녹화 시작
4. 로봇 팔을 조작하며 동작 수행
5. 터미널 2에서 그리퍼 조작 (`q`/`w`/`o`/`p`)
6. 터미널 1에서 `s` 키로 에피소드 저장
7. 3~6번 반복하여 여러 에피소드 녹화
8. 터미널 1에서 `q` 키로 데이터셋 완료

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `record_rate` | `50.0` | 녹화 주파수 (Hz) |
| `dataset_name` | `openarm_bimanual` | 데이터셋 이름 |
| `save_dir` | `~/lerobot_datasets` | 저장 경로 |

### 사용 예시

```bash
# 커스텀 데이터셋 이름
ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py \
  dataset_name:=my_task_001 record_rate:=30.0
```

### 저장 구조

```
~/lerobot_datasets/openarm_bimanual/
├── data/
│   └── train-00000.parquet
└── info.json
```

---

## 3. 키보드 그리퍼 제어

CAN 모터 기반 그리퍼를 키보드로 제어합니다.

### 실행 방법

```bash
# 반드시 별도 터미널에서 실행
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

### 키보드 조작

| 키 | 동작 |
|:--:|------|
| `q` | 왼쪽 그리퍼 열기 |
| `w` | 왼쪽 그리퍼 닫기 |
| `o` | 오른쪽 그리퍼 열기 |
| `p` | 오른쪽 그리퍼 닫기 |
| `ESC` | 종료 |

### 주요 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `gripper_speed` | `2.0` | 그리퍼 속도 (rad/s) |
| `min_gripper` | `-0.5` | 최소 위치 (rad) |
| `max_gripper` | `2.0` | 최대 위치 (rad) |

> [!NOTE]
> 그리퍼 컨트롤러는 `/joint_states`에서 현재 위치를 동기화한 후 동작합니다.
> "Both grippers synced!" 메시지가 나타난 후 조작하세요.

---

## 4. 데이터 재생

녹화된 데이터로 로봇을 재생합니다.

### 컨트롤러 전환 (필수)

```bash
ros2 control switch_controllers \
  --deactivate left_effort_controller right_effort_controller \
  --activate left_joint_trajectory_controller right_joint_trajectory_controller
```

### 재생 실행

```bash
ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args -p dataset_path:=~/lerobot_datasets/openarm_bimanual
```

### 재생 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `dataset_path` | (필수) | 데이터셋 경로 |
| `episode_index` | `0` | 재생할 에피소드 (-1: 전체) |
| `playback_speed` | `1.0` | 재생 속도 배수 |
| `loop` | `false` | 반복 재생 |

### 사용 예시

```bash
# 2배속으로 에피소드 3 반복 재생
ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args \
  -p dataset_path:=~/lerobot_datasets/openarm_bimanual \
  -p episode_index:=3 \
  -p playback_speed:=2.0 \
  -p loop:=true
```

---

## 문제 해결

### rev4 조인트 오실레이션

**증상**: rev4 조인트가 흔들리는 현상

**해결**: `gravity_comp_teaching.launch.py`에서 rev4 스케일 조정

```python
'gravity_scale_joints': [0.5, 2.0, 1.1, 1.8, 1.5, 1.85, 1.65]
#                                      ^^^^ rev4: 1.0 → 1.8
```

### 그리퍼 동기화 대기

**증상**: 그리퍼가 반응하지 않음

**해결**: "Both grippers synced!" 메시지 확인 후 조작

### 재생 시 로봇 미반응

**증상**: 재생 명령이 동작하지 않음

**해결**: trajectory 컨트롤러 활성화 확인
```bash
ros2 control list_controllers
```
