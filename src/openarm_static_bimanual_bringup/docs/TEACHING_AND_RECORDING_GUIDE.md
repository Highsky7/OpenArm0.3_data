# OpenArm Bimanual VLA 데이터 수집 및 재생 가이드

이 문서는 OpenArm Static Bimanual 로봇의 VLA (Vision-Language-Action) 모델 학습용 데이터 수집 시스템 사용법을 설명합니다.

---

## 목차

1. [시스템 개요](#1-시스템-개요)
2. [중력 보상 모드](#2-중력-보상-모드)
3. [VLA 데이터 녹화](#3-vla-데이터-녹화)
4. [키보드 그리퍼 제어](#4-키보드-그리퍼-제어)
5. [데이터 재생](#5-데이터-재생)
6. [문제 해결](#6-문제-해결)

---

## 1. 시스템 개요

### 데이터셋 구성

| Feature | Shape | 설명 |
|---------|-------|------|
| `observation.state` | (16,) | 16-DOF 조인트 위치 |
| `action` | (16,) | 다음 프레임 조인트 위치 |
| `observation.images.top` | (256, 256, 3) | 상단 카메라 |
| `observation.images.wrist_left` | (256, 256, 3) | 왼쪽 손목 카메라 |
| `observation.images.wrist_right` | (256, 256, 3) | 오른쪽 손목 카메라 |
| `language_instruction` | string | 작업 설명 |

### 카메라 토픽

| 토픽 | Feature Key |
|------|-------------|
| `camera1/cam1/color/image_raw` | `observation.images.top` |
| `camera1/cam2/color/image_raw` | `observation.images.wrist_left` |
| `camera1/cam3/color/image_raw` | `observation.images.wrist_right` |

> [!NOTE]
> 이미지는 원본 640×480에서 **256×256**으로 리사이즈되어 저장됩니다.

---

## 2. 중력 보상 모드

로봇 팔을 손으로 자유롭게 조작할 수 있는 Teaching 모드입니다.

### 실행

```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py
```

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `use_mock_hardware` | `false` | 시뮬레이션 모드 |
| `active_arms` | `both` | 활성화할 팔 |
| `rviz` | `true` | RViz 시각화 |

---

## 3. VLA 데이터 녹화

### 실행 (2개 터미널 필요)

```bash
# 터미널 1: 녹화 시스템
ros2 launch openarm_static_bimanual_bringup lerobot_vla_recording.launch.py \
  task_description:="pick and place red cube"

# 터미널 2: 그리퍼 제어
ros2 run openarm_static_bimanual_bringup keyboard_gripper_controller.py
```

### 녹화 키보드 (터미널 1)

| 키 | 동작 |
|:--:|------|
| `r` | 에피소드 녹화 시작 |
| `s` | 에피소드 저장 |
| `q` | 데이터셋 저장 후 종료 |

### 워크플로우

1. 터미널 1에서 launch 실행 (task_description 설정)
2. 터미널 2에서 그리퍼 컨트롤러 실행
3. `r` 키로 녹화 시작
4. 로봇 조작 + 그리퍼 제어
5. `s` 키로 에피소드 저장
6. 3~5 반복
7. `q` 키로 완료

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `record_rate` | `50.0` | 녹화 주파수 (Hz) |
| `dataset_name` | `openarm_bimanual` | 데이터셋 이름 |
| `save_dir` | `~/lerobot_datasets` | 저장 경로 |
| `task_description` | `bimanual manipulation task` | VLA 언어 지시 |
| `enable_cameras` | `true` | 카메라 녹화 활성화 |

### 저장 구조

```
~/lerobot_datasets/openarm_bimanual/
├── data/
│   └── train-00000.parquet
├── videos/
│   ├── observation.images.top/
│   │   └── episode_000000.mp4
│   ├── observation.images.wrist_left/
│   │   └── episode_000000.mp4
│   └── observation.images.wrist_right/
│       └── episode_000000.mp4
└── info.json
```

---

## 4. 키보드 그리퍼 제어

### 실행

```bash
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

> [!NOTE]
> "Both grippers synced!" 메시지 확인 후 조작하세요.

---

## 5. 데이터 재생

### 컨트롤러 전환 (필수)

```bash
ros2 control switch_controllers \
  --deactivate left_effort_controller right_effort_controller \
  --activate left_joint_trajectory_controller right_joint_trajectory_controller
```

### 재생 실행

```bash
ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args -p dataset_path:=~/lerobot_datasets/openarm_bimanual \
  -p use_action:=true
```

### 파라미터

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `dataset_path` | (필수) | 데이터셋 경로 |
| `episode_index` | `0` | 재생할 에피소드 |
| `playback_speed` | `1.0` | 재생 속도 |
| `loop` | `false` | 반복 재생 |
| `use_action` | `false` | action 컬럼 사용 |

### 예시

```bash
# 에피소드 3을 2배속 반복 재생
ros2 run openarm_static_bimanual_bringup lerobot_vla_replay.py \
  --ros-args \
  -p dataset_path:=~/lerobot_datasets/openarm_bimanual \
  -p episode_index:=3 \
  -p playback_speed:=2.0 \
  -p loop:=true \
  -p use_action:=true
```

---

## 6. 문제 해결

### rev4 조인트 오실레이션

```python
# gravity_comp_teaching.launch.py에서 조정
'gravity_scale_joints': [0.5, 2.0, 1.1, 1.8, 1.5, 1.85, 1.65]
#                                      ^^^^ rev4: 1.0 → 1.8
```

### 카메라 누락 오류

**증상**: "Cannot start: Missing cameras" 메시지

**해결**: 카메라 토픽 확인
```bash
ros2 topic list | grep camera1
```

### 재생 시 미반응

**해결**: trajectory 컨트롤러 활성화 확인
```bash
ros2 control list_controllers
```
