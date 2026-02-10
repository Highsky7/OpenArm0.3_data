#!/bin/bash
# VLA Inference Server 시작 스크립트
# 서버에서 실행: ./start_server.sh

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

# 체크포인트 경로 (인자로 전달 가능)
POLICY_PATH="${1:-$DEFAULT_POLICY_PATH}"

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
echo -e "${GREEN}  서버 시작!${NC}"
echo -e "${GREEN}======================================${NC}"

if [ "$DEBUG" = "true" ]; then
    echo -e "${YELLOW}디버그 모드 활성화${NC}\n"
    python "$SCRIPT_DIR/vla_inference_server.py" \
        --policy_path "$POLICY_PATH" \
        --port $PORT \
        --debug
else
    python "$SCRIPT_DIR/vla_inference_server.py" \
        --policy_path "$POLICY_PATH" \
        --port $PORT
fi
