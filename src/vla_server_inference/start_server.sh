#!/bin/bash
# VLA Inference Server 시작 스크립트 (smolvla/pi0 전용)
# 서버에서 실행: ./start_server.sh [checkpoint_path] [--debug]
#
# 사용 예시:
#   ./start_server.sh /path/to/smolvla_checkpoint                  # SmolVLA (기본값)
#   MODEL_TYPE=pi0 ./start_server.sh /path/to/pi0_checkpoint       # Pi0
#   # GROOT/FMVLA는 전용 가상환경에서 python 직접 실행

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}  VLA Inference Server Launcher${NC}"
echo -e "${GREEN}======================================${NC}"

# 기본 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECKPOINT_DIR="/datastore/khdw/OpenArm0.3_data/checkpoints"
DEFAULT_POLICY_PATH="${CHECKPOINT_DIR}/smolvla_openarm_16dim/020000/pretrained_model"
PORT="${PORT:-5555}"
DEBUG="${DEBUG:-false}"
MODEL_TYPE="${MODEL_TYPE:-smolvla}"

# start_server.sh는 smolvla/pi0만 지원
if [[ "$MODEL_TYPE" != "smolvla" && "$MODEL_TYPE" != "pi0" ]]; then
    echo -e "${RED}✗ start_server.sh는 smolvla/pi0만 지원합니다.${NC}"
    echo -e "${YELLOW}groot는 vla_server_groot 환경에서 python 직접 실행하세요:${NC}"
    echo -e "  conda activate vla_server_groot"
    echo -e "  python $SCRIPT_DIR/vla_inference_server.py --policy_path /path/to/groot_checkpoint --model_type groot --port $PORT --debug"
    echo -e "${YELLOW}fmvla는 vla_server_fmvla 환경에서 python 직접 실행하세요:${NC}"
    echo -e "  conda activate vla_server_fmvla"
    echo -e "  python $SCRIPT_DIR/vla_inference_server.py --policy_path /path/to/fmvla_pretrained_model --model_type fmvla --port $PORT --debug"
    exit 1
fi

# 체크포인트 경로 (인자로 전달 가능)
# 인자 파싱
while [[ $# -gt 0 ]]; do
  case $1 in
    --debug)
      DEBUG="true"
      shift
      ;;
    *)
      POLICY_PATH="$1"
      shift
      ;;
  esac
done

# 체크포인트 경로가 없으면 기본값 사용
if [ -z "$POLICY_PATH" ]; then
    if [ "$MODEL_TYPE" = "pi0" ]; then
        POLICY_PATH="${CHECKPOINT_DIR}/pi0_lora_20260209_080912/checkpoints/last/pretrained_model"
    else
        POLICY_PATH="$DEFAULT_POLICY_PATH"
    fi
fi

# Conda 환경 활성화
echo -e "\n${YELLOW}[1/3] Conda 환경 활성화 중...${NC}"
if command -v conda &> /dev/null; then
    eval "$(conda shell.bash hook)"
    conda activate vla_server
    echo -e "${GREEN}✓ vla_server 환경 활성화됨${NC}"
else
    echo -e "${RED}✗ Conda를 찾을 수 없습니다${NC}"
    exit 1
fi

# 체크포인트 확인
echo -e "\n${YELLOW}[2/3] 체크포인트 확인 중...${NC}"
if [ -d "$POLICY_PATH" ]; then
    echo -e "${GREEN}✓ 체크포인트 발견: $POLICY_PATH${NC}"
else
    echo -e "${RED}✗ 체크포인트를 찾을 수 없습니다: $POLICY_PATH${NC}"
    echo -e "${YELLOW}힌트: 체크포인트 경로를 인자로 전달하세요:${NC}"
    echo -e "  ./start_server.sh /path/to/checkpoint"
    exit 1
fi

# 포트 사용 확인
echo -e "\n${YELLOW}[3/3] 포트 $PORT 확인 중...${NC}"
if lsof -i :$PORT > /dev/null 2>&1; then
    echo -e "${RED}✗ 포트 $PORT가 이미 사용 중입니다${NC}"
    echo -e "${YELLOW}힌트: 다른 포트를 사용하세요:${NC}"
    echo -e "  PORT=5556 ./start_server.sh"
    exit 1
else
    echo -e "${GREEN}✓ 포트 $PORT 사용 가능${NC}"
fi

# 서버 시작
echo -e "\n${GREEN}======================================${NC}"
echo -e "${GREEN}  서버 시작! (모델: $MODEL_TYPE)${NC}"
echo -e "${GREEN}======================================${NC}"

if [ "$DEBUG" = "true" ]; then
    echo -e "${YELLOW}디버그 모드 활성화${NC}\n"
    python "$SCRIPT_DIR/vla_inference_server.py" \
        --policy_path "$POLICY_PATH" \
        --port $PORT \
        --debug \
        --model_type "$MODEL_TYPE"
else
    python "$SCRIPT_DIR/vla_inference_server.py" \
        --policy_path "$POLICY_PATH" \
        --port $PORT \
        --model_type "$MODEL_TYPE"
fi
