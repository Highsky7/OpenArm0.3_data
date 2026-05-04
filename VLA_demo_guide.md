# VLA 데모 가이드
## 실행 단계(순서대로 실행 필수!)

### Step 1: 서버 실행(vla_server tmux sessioin에서 실행)

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

(fmvla)
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

### Step 3: 중력보상 및 초기 위치(로봇 노트북에서 실행)

```bash
cd OpenArm0.3_data
humble
si

```
