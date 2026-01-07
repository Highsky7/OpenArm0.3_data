# OpenArm v0.3 중력보상 및 데이터 녹화 가이드

## 개요

OpenArm v0.3 양팔 로봇팔에 **Pinocchio 기반 정밀 중력보상 모드**를 구현하여 수동 조작(Teaching)이 가능하도록 하고, **Joint Angle 데이터 녹화** 기능(Gripper 포함)을 추가했습니다.

---

## 주요 특징

### 🆕 Pinocchio 기반 3D 중력보상
- **Pinocchio 라이브러리**(v3.x)를 사용하여 정확한 3D 역동역학 계산
- URDF를 직접 파싱하여 조인트 간 커플링 및 회전축 변화를 완벽히 반영
- 기존 단순 모델 대비 복잡한 자세에서도 정확한 토크 계산

### 🆕 Gripper 데이터 녹화 지원
- **실제 하드웨어**: Arduino 브릿지가 `/gripper_states` 토픽으로 발행
- **Mock 하드웨어**: ros2_control에 gripper 조인트 등록 (`use_mock_hardware=true`일 때만)
- `continuous_recorder_node.py`가 `/joint_states` + `/gripper_states` 모두 구독하여 녹화

---

## 수정/생성된 파일

### 수정된 파일

| 파일                                                | 변경 내용                                            |
| --------------------------------------------------- | ---------------------------------------------------- |
| `urdf/openarm_static_bimanual.urdf.xacro`         | effort interface 추가, gripper는 mock에서만 ros2_control 등록 |
| `config/openarm_static_bimanual_controllers.yaml` | `left/right_effort_controller` 추가                |
| `launch/sbopenarm.launch.py`                      | `use_grippers` arg 추가 및 xacro 전달                |
| `launch/gravity_comp_teaching.launch.py`          | URDF 파일 생성(`xacro -o`), Pinocchio용 설정 추가   |
| `scripts/continuous_recorder_node.py`             | `/gripper_states` 구독 추가, arm+gripper 데이터 합쳐 녹화 |

### 신규 생성 파일

| 파일                                       | 용도                          |
| ------------------------------------------ | ----------------------------- |
| `scripts/gravity_comp_node.py`           | Pinocchio 기반 중력보상 노드 |
| `scripts/continuous_recorder_node.py`    | 연속 녹화 노드 (Gripper 포함) |
| `launch/gravity_comp_teaching.launch.py` | 통합 launch 파일              |

---

## 사전 요구사항

### Pinocchio 설치
```bash
# ROS2 Humble용 Pinocchio 설치
sudo apt install ros-humble-pinocchio

# NumPy 호환성 확인 (NumPy 2.x는 호환 안됨)
pip install "numpy<2"
```

### 설치 확인
```bash
python3 -c "import pinocchio as pin; print('Version:', pin.__version__)"
# 출력: Version: 3.x.x
```

---

## 안전 기능

### 1. 토크 클램핑 (정격 토크 기준)

```python
# DM4340 (rev1~4): 정격 9 Nm
# DM4310 (rev5~7): 정격 3 Nm
MAX_TORQUE = [9.0, 9.0, 9.0, 9.0, 3.0, 3.0, 3.0]
```

### 2. 조인트 리밋 보호

| 조인트 | 범위      | 안전 마진 |
| ------ | --------- | --------- |
| rev1   | ±120°   | ±5°     |
| rev2   | ±90°    | ±5°     |
| rev3   | ±120°   | ±5°     |
| rev4   | 0°~150° | ±5°     |
| rev5   | ±120°   | ±5°     |
| rev6   | ±90°    | ±5°     |
| rev7   | ±55°    | ±5°     |

리밋 접근 시 **가상 스프링 토크**가 적용되어 안전 범위로 밀어냅니다.

### 3. Position 동기화 (Kp 항 상쇄)

`active_mode='teleop'`으로 설정하여 현재 위치를 목표 위치로 발행, MIT 공식의 Kp 항을 상쇄합니다.

```text
τ = Kp*(q_des - q) + Kd*(dq_des - dq) + τ_ff
    ↓ q_des = q 설정 (teleop_stream_controller 사용)
τ = 0 + Kd*(-dq) + τ_ff  ← Kp 상쇄됨!
```

---

## 실행 방법

### Step 1: 빌드 확인

```bash
cd ~/openarm_official_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to openarm_static_bimanual_bringup
source install/setup.bash
```

### Step 2: CAN 인터페이스 설정

```bash
# CAN 활성화 (환경에 따라 다름)
sudo slcand -o -c -s8 -S 1000000 /dev/ttyACM0 can0
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000

# 연결 확인
candump can0
```

### Step 3: Mock Hardware 사전 테스트 (권장)

Mock 하드웨어에서는 gripper 조인트도 ros2_control에 등록되어 `/joint_states`에 발행됩니다.

```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    use_mock_hardware:=true \
    enable_recorder:=true
```

**별도 터미널에서 확인:**
```bash
# Pinocchio 모델 로드 확인
ros2 node list | grep gravity

# 조인트 상태 확인 (gripper 포함)
ros2 topic echo /joint_states --field name

# 컨트롤러 활성화 확인
ros2 control list_controllers
```

### Step 4: 중력보상 모드 실행 (실제 하드웨어)

실제 하드웨어에서는 Arm(CAN 모터)과 Gripper(Arduino 서보)가 분리되어 있습니다.

**터미널 1: 중력보상 노드 실행**
```bash
ros2 launch openarm_static_bimanual_bringup gravity_comp_teaching.launch.py \
    use_mock_hardware:=false \
    can_device:=can0 \
    enable_recorder:=true \
    active_arms:=both
```

**터미널 2: Arduino 브릿지 실행 (Gripper 데이터 발행)**
```bash
ros2 launch openarm_arduino_bridge arduino_servo.launch.py port:=/dev/ttyACM2
```

> ⚠️ **중요**: 
> - 실제 하드웨어에서 gripper는 ros2_control에 등록되지 않습니다.
> - Gripper 데이터 녹화를 위해 **Arduino 브릿지를 반드시 실행**해야 합니다.
> - Arduino 브릿지는 `/gripper_states` 토픽으로 발행합니다.

### Step 5: 데이터 녹화

`enable_recorder:=true`로 실행하면 recorder가 자동 시작됩니다.

**또는 별도 터미널에서 수동 실행:**
```bash
ros2 run openarm_static_bimanual_bringup continuous_recorder_node.py
```

**키보드 조작**:
- `r`: 녹화 시작
- `s`: 녹화 중지 및 저장
- `q`: 종료

저장 경로: `~/openarm_official_ws/joint_trajectory_YYYYMMDD_HHMMSS.json`

---

## 파라미터 설명

### active_arms (팔 선택)

| 값        | 설명                        |
| --------- | --------------------------- |
| `both`  | 양팔 모두 중력보상 (기본값) |
| `left`  | 왼쪽 팔만 중력보상          |
| `right` | 오른쪽 팔만 중력보상        |

### gravity_scale (중력보상 강도)

```bash
# 런타임 조정
ros2 param set /gravity_comp_node gravity_scale 0.5
```

| 값            | 효과                                       |
| ------------- | ------------------------------------------ |
| 0.0           | 중력보상 없음                              |
| **0.1** | **기본값 (10% 보상, 안전 테스트용)** |
| 0.5           | 중간 보상                                  |
| 1.0           | 100% 보상 (완전 중력 상쇄)                 |
| >1.0          | 과보상 (주의: 팔이 튀어오를 수 있음)       |

---

## 데이터 녹화 구조

### 토픽 구조

```
┌─────────────────────────────────────────────────────────────┐
│  OpenArmHWFlex + joint_state_broadcaster                   │
│    └── /joint_states (left_rev1~7, right_rev1~7)           │
└─────────────────────────────────────────────────────────────┘
                            ↓
                   continuous_recorder_node.py
                            ↑
┌─────────────────────────────────────────────────────────────┐
│  Arduino 브릿지 (bimanual_bridge_node.py)                  │
│    └── /gripper_states (left_gripper_joint, right_gripper_joint)
└─────────────────────────────────────────────────────────────┘
```

### 녹화 데이터 형식

```json
{
  "timestamp": 1.234,
  "joint_names": ["left_rev1", ..., "left_rev7", "right_rev1", ..., "right_rev7", "left_gripper_joint", "right_gripper_joint"],
  "positions": [...],
  "velocities": [...],
  "efforts": [...]
}
```

---

## 디버깅 명령어

### 토픽 모니터링

```bash
# Arm 조인트 상태 확인
ros2 topic echo /joint_states

# Gripper 상태 확인 (Arduino 브릿지)
ros2 topic echo /gripper_states

# 중력보상 토크 확인
ros2 topic echo /left_effort_controller/commands
```

### 컨트롤러 상태 확인

```bash
# 활성화된 컨트롤러 목록
ros2 control list_controllers

# 하드웨어 인터페이스 상태
ros2 control list_hardware_interfaces
```

### 파라미터 조회/변경

```bash
# 현재 파라미터 확인
ros2 param get /gravity_comp_node gravity_scale

# 파라미터 변경
ros2 param set /gravity_comp_node gravity_scale 0.8
```

---

## 문제 해결

| 증상                                  | 원인                       | 해결                                                         |
| ------------------------------------- | -------------------------- | ------------------------------------------------------------ |
| 팔이 떨어짐                           | gravity_scale 낮음         | `ros2 param set /gravity_comp_node gravity_scale 0.5`      |
| 팔이 튀어오름                         | gravity_scale 높음         | gravity_scale 감소                                           |
| **Param size mismatch**         | ros2_control에 8개 조인트  | 실제 HW에서는 gripper를 ros2_control에서 제외 (이미 적용됨) |
| **URDF file not found**         | xacro 실행 실패            | `/tmp/openarm_v03_bimanual.urdf` 파일 존재 확인            |
| **Pinocchio import error**      | 잘못된 pinocchio 패키지    | `pip uninstall pinocchio && sudo apt install ros-humble-pinocchio` |
| **NumPy 호환 오류**             | NumPy 2.x 설치됨           | `pip install "numpy<2"`                                    |
| **Gripper 데이터 없음**         | Arduino 브릿지 미실행      | `ros2 launch openarm_arduino_bridge arduino_servo.launch.py` |
| **gravity_comp_node not found** | 스크립트 미설치            | `colcon build` 및 `chmod +x` 실행                        |

---

## 기술 상세

### Pinocchio RNEA 알고리즘

`gravity_comp_node.py`는 Pinocchio의 `rnea()` 함수를 사용합니다:

```python
import pinocchio as pin

# URDF 로드
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# 중력 토크 계산 (속도=0, 가속도=0)
pin.rnea(model, data, q, np.zeros(model.nv), np.zeros(model.nv))
tau_gravity = data.tau
```

### Gripper ros2_control 등록 (Mock 전용)

`openarm_static_bimanual.urdf.xacro`에서 gripper 조인트는 **mock 하드웨어에서만** ros2_control에 등록됩니다:

```xml
<!-- Gripper joint (prismatic) - only for mock hardware -->
<xacro:if value="${use_mock_hw_eff}">
  <joint name="${left_prefix_eff}left_pris1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</xacro:if>
```

실제 하드웨어에서는 `OpenArmHWFlex`가 7개 모터만 지원하므로, gripper는 Arduino 브릿지를 통해 별도로 제어/모니터링됩니다.

---

## 긴급 정지

```bash
# Ctrl+C로 launch 종료
# 또는 별도 터미널에서:
ros2 topic pub --once /left_effort_controller/commands \
    std_msgs/Float64MultiArray "data: [0,0,0,0,0,0,0]"
ros2 topic pub --once /right_effort_controller/commands \
    std_msgs/Float64MultiArray "data: [0,0,0,0,0,0,0]"
```
