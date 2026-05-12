# VLA 데모 가이드

## 0. 재부팅 시 CAN 설정

```bash
ls /dev/ttyACM*
sudo slcand -o -c -s8 -S 1000000 /dev/ttyACM0 can0
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
ip -details link show can0
candump can0
```

## 1. 통신 추론 실행 단계(순서대로 실행 필수!)

### Step 1: 서버 추론 실행(vla_server tmux session에서 실행)

```bash
cd ../..
cd datastore/khdw/OpenArm0.3_data/src/vla_server_inference

(smolvla, pi-0)
conda activate vla_server

# SmolVLA 실행 (기본값)
./start_server.sh /datastore/khdw/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/020000/pretrained_model

# Pi0 실행
MODEL_TYPE=pi0 ./start_server.sh /datastore/khdw/OpenArm0.3_data/checkpoints/pi0_lora_20260209_080912/checkpoints/last/pretrained_model --debug

(groot n 1.5)
conda activate vla_server_groot
python vla_inference_server.py \
    --policy_path /datastore/khdw/OpenArm0.3_data/checkpoints/groot_run_full_finetune_v1/checkpoints/last/pretrained_model \
    --port 5555 \
    --model_type groot \
    --debug

(tdsr-vla)
conda activate vla_server_fmvla
python vla_inference_server.py \
    --policy_path /datastore/khdw/OpenArm0.3_data/checkpoints/fmvla_openarm_0213_1614/checkpoints/022500/pretrained_model \
    --model_type fmvla \
    --port 5555 \
    --fmvla_lora_weights_path /datastore/khdw/OpenArm0.3_data/checkpoints/V_P_OpenARM_0209_2332/checkpoint-27110/pytorch_lora_weights.safetensors \
    --fmvla_hold_on_chunk_boundary \
    --fmvla_hold_max_sec 0 \
    --fmvla_lora_scale 1.0 \
    --no-fmvla_enable_sd3_cpu_offload \
    --fmvla_profile_timing \
    --fmvla_timing_warmup_chunks 1 \
    --fmvla_timing_report_every_chunks 1 \
    --debug

```

### Step 2: ssh 터널링(로봇 노트북에서 실행)

```bash
ssh -L 5555:localhost:5555 dongwoo@163.152.193.246
```

### Step 3: 카메라 실행(로봇 노트북에서 실행)

```bash
cd realsense_ws
humble
si
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py \
    serial_no1:="'_317322073024'" \
    serial_no2:="'_326522073051'" \
    serial_no3:="'_327322071339'"
```

### Step 4: 중력보상 및 초기 위치(로봇 노트북에서 실행)

```bash
cd OpenArm0.3_data
humble
si
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py enable_replay_mode:=true
```

### Step 5:  추론 client 노드 실행(로봇 노트북에서 항상 마지막에 실행!!)

```bash
cd OpenArm0.3_data
humble
si
ros2 launch openarm_static_bimanual_bringup vla_remote_inference.launch.py \
    task_description:="Move the basket to the right side and put the paper roll in the basket" \
    enable_control:=true \
    debug:=true
```

## 2. 로컬 추론 실행 단계(순서대로 실행 필수!, SmolVLA만 가능)

### Step 1: 중력보상 및 초기 위치(로봇 노트북에서 실행)

```bash
cd OpenArm0.3_data
humble
si
ros2 launch openarm_static_bimanual_bringup lerobot_trajectory_recording.launch.py enable_replay_mode:=true
```
### Step 2: 카메라 실행(로봇 노트북에서 실행)

```bash
cd realsense_ws
humble
si
ros2 launch realsense2_camera rs_multi_camera_launch_sync_3.py   camera_name1:=cam_1 camera_name2:=cam_2 camera_name3:=cam_3   camera_namespace1:=camera camera_namespace2:=camera camera_namespace3:=camera   serial_no1:='_346222072155' serial_no2:='_247122072494' serial_no3:='_247122074423'
```

### Step 3: 추론 실행(로봇 노트북에서 실행)

```bash
cd OpenArm0.3_data
humble
si
ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/home/mintlabskh/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/checkpoints/020000/pretrained_model \
    task_description:="Move the basket to the right side and put the paper roll in the basket" \
    enable_control:=true \
    control_arm:=both

ros2 launch openarm_static_bimanual_bringup smolvla_inference.launch.py \
    policy_path:=/home/mintlabskh/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/checkpoints/020000/pretrained_model \
    task_description:="Put the umbrellas into the basket" \
    enable_control:=true \
    control_arm:=both
```
