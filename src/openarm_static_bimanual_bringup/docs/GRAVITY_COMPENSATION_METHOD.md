# Gravity Compensation Method for OpenArm v0.3 Bimanual / OpenArm v0.3 양팔 중력보상 방식

> This document explains the **currently applied** gravity compensation method in `openarm_static_bimanual_bringup`, based on implementation truth in code and launch files.
> 이 문서는 `openarm_static_bimanual_bringup`의 **현재 적용 구현** 기준으로 중력보상 방식을 정리합니다.

---

## 1. Scope and Source of Truth / 범위와 근거

### Primary source files / 1차 근거 파일
- `scripts/gravity_comp_node.py`
- `launch/gravity_comp_teaching.launch.py`
- `launch/lerobot_trajectory_recording.launch.py`
- `launch/lerobot_vla_replay.launch.py`
- `launch/lerobot_vla_collection.launch.py`
- `config/openarm_static_bimanual_controllers.yaml`

### What this document covers / 문서 포함 범위
- Gravity torque computation pipeline (Pinocchio RNEA based)
- Safety terms (joint-limit virtual spring + torque clamp + zero-torque shutdown)
- Teaching / Replay / Initial-move state behavior
- Runtime integration with ros2_control controllers and topic dataflow
- Launch variant parameter differences (teaching / recording / replay / collection)

---

## 2. Runtime Architecture and Dataflow / 런타임 구조 및 데이터흐름

## 2.1 Core loop overview / 메인 루프 개요
`gravity_comp_node.py` runs a timer loop (`publish_rate`, default 100 Hz) and does the following each cycle:
1. Read current arm joint positions (`/joint_states`).
2. Compute gravity torques using Pinocchio RNEA.
3. Add joint-limit protection torques.
4. Clamp torques to configured motor-safe max values.
5. Publish:
- Effort command: `/left_effort_controller/commands`, `/right_effort_controller/commands`
- Position sync command: `/left_teleop_stream_controller/commands`, `/right_teleop_stream_controller/commands`

`gravity_comp_node.py`는 타이머 루프(기본 100 Hz)에서 매 주기 아래 순서로 동작합니다:
1. `/joint_states`로 현재 관절 상태 수집
2. Pinocchio RNEA로 중력 토크 계산
3. 관절 한계 보호 토크 추가
4. 모터 안전 한계로 토크 포화(clamp)
5. effort 명령 + position 동기화 명령 발행

## 2.2 Why position sync is published / position 동기화 발행 이유
The node intentionally publishes current (or rate-limited replay target) joint position to teleop stream controllers so that position-controller error is neutralized in teaching mode (`q_des ~= q`).

티칭 모드에서는 position controller 오차항(`q_des - q`)을 0에 가깝게 유지하기 위해 현재 관절값을 teleop stream controller로 재발행합니다.

## 2.3 Replay external command path / 리플레이 외부 명령 경로
When replay mode is enabled (`enable_replay_mode=true`), external position commands are accepted from:
- `/gravity_comp/left_external_position_cmd`
- `/gravity_comp/right_external_position_cmd`

These are consumed by gravity node, rate-limited, and converted into synchronized position command outputs while gravity effort remains active.

---

## 3. Core Control Law / 핵심 제어식

## 3.1 Gravity term / 중력 항
Current implementation uses Pinocchio model loaded from URDF (`urdf_path`) and computes:

```text
tau_g_full = RNEA(model, q, v=0, a=0)
```

Then per-arm 7 joints are extracted by mapped indices (`left_indices`, `right_indices`), and per-joint scale is applied:

```text
tau_scaled[i] = gravity_scale_joints[i] * tau_g_arm[i],  i=0..6
```

## 3.2 Safety augmentation / 안전항 추가
For each arm joint i:

```text
tau_total[i] = tau_scaled[i] + tau_limit[i]
```

where `tau_limit[i]` comes from joint-limit virtual spring logic (Section 4).

## 3.3 Torque saturation / 토크 포화
Final command is saturated by per-joint max torque:

```text
tau_cmd[i] = clamp(tau_total[i], -MAX_TORQUE[i], +MAX_TORQUE[i])
```

Current `MAX_TORQUE` in code:
- rev1~rev4: 9.0 Nm
- rev5~rev7: 3.0 Nm

---

## 4. Safety Mechanisms / 안전 메커니즘

## 4.1 Joint-limit virtual spring / 관절한계 가상 스프링
Code constants:
- `JOINT_LIMITS` (per `rev1..rev7` lower/upper rad)
- `safety_margin` (default 0.087 rad, ~5°)
- `limit_spring_k` (default 3.0 Nm/rad)

Behavior:
- Near lower bound (`position < lower + margin`): positive restoring torque.
- Near upper bound (`position > upper - margin`): negative restoring torque.
- If beyond hard limit: warning log is emitted (`warn` with throttle).

## 4.2 Timeout fallback in replay / 리플레이 타임아웃 폴백
Replay commands are accepted only if recent enough:
- `check_external_cmd_timeout()` compares now - last external command time.
- If timeout exceeded: automatically falls back to teaching-mode behavior.

Defaults:
- node default `external_cmd_timeout = 1.0 s`
- some replay-related launch files override to `0.5 s`

## 4.3 Graceful stop / 안전 종료
On shutdown (`destroy_node()`), `send_zero_torque()` publishes zero-effort command multiple times (3 iterations) to release motor load safely.

---

## 5. Mode Behavior / 모드별 동작

## 5.1 Teaching mode / 티칭 모드
Condition:
- replay mode disabled, or replay command timeout, or no valid external command

Behavior:
- Position command = current joint position (error neutralization)
- Effort command = gravity + limit protection + clamp

Use case:
- Human physically guides arm while robot compensates gravity.

## 5.2 Replay mode / 리플레이 모드
Condition:
- `enable_replay_mode=true`
- and external command not timed out

Behavior:
- External target is rate-limited via `max_position_step_rad`
- Limiting reference is **last published command**, not raw sensor position
- Gravity effort stays active while replay target drives position stream

Implication:
- Reduces command jerk and drift accumulation during replay.

## 5.3 Initial move mode / 초기자세 이동 모드
Condition:
- `enable_initial_move=true`

State machine:
- `waiting -> moving -> done`

Behavior in `moving`:
- Interpolates from current pose to configured initial pose using cosine easing
- During interpolation, gravity and safety effort are still computed and published

---

## 6. Controller Integration / 컨트롤러 연동

From `openarm_static_bimanual_controllers.yaml`:
- Gravity effort controllers:
  - `left_effort_controller` (`effort_controllers/JointGroupEffortController`)
  - `right_effort_controller` (`effort_controllers/JointGroupEffortController`)
- Position stream controllers:
  - `left_teleop_stream_controller`
  - `right_teleop_stream_controller`
- Both effort and teleop-stream controller joint sets are arm 7-DOF (`rev1..rev7`) only.
- Grippers (`left_rev8`, `right_rev8`) are separate position controllers.

Launch files in this package start base bringup with `active_mode='teleop'` and then spawn effort controllers, so gravity effort path and position-sync path coexist.

---

## 7. Launch Variant Comparison / 런치 변형 비교

All below variants generate URDF to `/tmp/openarm_v03_bimanual.urdf`, include base bringup, spawn effort controllers (typically after 3s), then start gravity node.

| Launch File | Main Purpose / 목적 | Replay Mode | `gravity_scale_joints` | Initial Move | `external_cmd_timeout` | Delayed Start Pattern |
|---|---|---|---|---|---|---|
| `gravity_comp_teaching.launch.py` | Manual teaching with gravity compensation / 수동 교시 | Default false (not explicitly enabled) | `[0.5, 2.0, 1.1, 1.8, 1.5, 1.85, 1.65]` | Default false (node default) | Node default (1.0s, replay off in practice) | effort @3s, gravity @5s |
| `lerobot_trajectory_recording.launch.py` | Phase 1 trajectory-only recording / 1단계 trajectory 녹화 | Launch arg `enable_replay_mode` (default false) | `[0.0, 2.5, 1.7, 1.7, 2.0, 2.0, 2.0]` | Launch arg enabled by default (`true`) | Node default 1.0s (unless replay enabled and overridden externally) | effort @3s, gravity @5s |
| `lerobot_vla_replay.launch.py` | Replay dataset on robot / trajectory 리플레이 | Always true when gravity node launched (`enable_replay_mode=true`) | `[0.0, 2.5, 1.7, 1.7, 2.0, 2.0, 2.0]` | Not explicitly enabled (default false) | Overridden to `0.5s` | effort @3s, gravity @5s, replay node @7s |
| `lerobot_vla_collection.launch.py` | Phase 2 replay + camera record / 2단계 VLA 수집 | Always true (`enable_replay_mode=true`) | `[0.0, 2.5, 1.7, 1.7, 2.0, 2.0, 2.0]` | Launch arg enabled by default (`true`) | Overridden to `0.5s` | effort @3s, gravity @5s, replay-recorder @10s |

---

## 8. Parameter Reference (`gravity_comp_node.py`) / 파라미터 레퍼런스

## 8.1 Core control / 핵심 제어
- `publish_rate` (default `100.0` Hz)
- `active_arms` (default `'both'`)
- `urdf_path` (default `'/tmp/openarm_v03_bimanual.urdf'`)

## 8.2 Gravity/safety / 중력/안전
- `gravity_scale_joints` (default `[1.5,1.5,1.5,1.5,2.5,2.5,2.5]`, often overridden by launch)
- `enable_limit_protection` (default `True`)
- `safety_margin` (default `0.087` rad)
- `limit_spring_k` (default `3.0` Nm/rad)

## 8.3 Replay mode / 리플레이
- `enable_replay_mode` (default `False`)
- `max_position_step_rad` (default `0.1` rad/cycle)
- `external_cmd_timeout` (default `1.0` sec)

## 8.4 Initial move / 초기 이동
- `enable_initial_move` (default `False`)
- `initial_left_position` (8 elements, first 7 used for arm interpolation)
- `initial_right_position` (8 elements, first 7 used for arm interpolation)
- `initial_move_duration` (default `3.0` sec)

---

## 9. Assumptions and Limitations / 가정 및 한계

1. **Gravity-only inverse dynamics**
- RNEA is evaluated with `v=0`, `a=0`, so dynamic inertial/coriolis effects are not compensated.

2. **Per-arm partial q update assumption**
- Node updates active arm indices in shared `self.q` and runs whole-model RNEA.
- Implementation comments assume left/right gravity decoupling through fixed base.

3. **Model/parameter consistency dependency**
- Compensation quality depends on URDF model fidelity and `gravity_scale_joints` tuning.
- Launch-specific scales differ by workflow.

4. **Embedded constants not in active torque path**
- `LINK_PARAMS`, `LINK_LENGTHS`, `JOINT_AXIS_SIGN`, `RIGHT_ARM_ORIGIN_FLIP` are defined in node but current torque computation path uses Pinocchio RNEA output directly.

---

## 10. Operational Checklist / 운용 체크리스트

1. Confirm gravity node and controllers are active.
- `ros2 control list_controllers`

2. Confirm external replay command flow (when replay enabled).
- `ros2 topic echo /gravity_comp/left_external_position_cmd`
- `ros2 topic echo /gravity_comp/right_external_position_cmd`

3. If arm sags (중력 보상 부족):
- Increase corresponding `gravity_scale_joints` entry.

4. If arm feels too stiff or resists manual teaching (과보상):
- Decrease corresponding `gravity_scale_joints` entry.

5. If near limit warnings appear frequently:
- Review working envelope, `safety_margin`, and motion scripts.

---

## 11. Quick Formula Summary / 수식 요약

```text
Input: q (from /joint_states)

1) tau_g = RNEA(q, 0, 0)
2) tau_scaled[i] = gravity_scale_joints[i] * tau_g_arm[i]
3) tau_limit[i] = virtual_spring(joint_limit, safety_margin, limit_spring_k)
4) tau_cmd[i] = clamp(tau_scaled[i] + tau_limit[i], -MAX_TORQUE[i], +MAX_TORQUE[i])
5) publish effort + position-sync command
```

This is the currently applied gravity compensation method in the package.
위 식이 본 패키지에서 현재 적용 중인 중력보상 방식입니다.
