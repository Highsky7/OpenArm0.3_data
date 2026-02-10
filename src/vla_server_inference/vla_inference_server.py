#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLA Inference Server - GPU 서버에서 실행

ZeroMQ REP 소켓을 통해 로봇 laptop으로부터 관측 데이터를 수신하고,
SmolVLA 추론 결과(action)를 반환합니다.

사용법:
    python vla_inference_server.py \
        --policy_path ~/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/pretrained_model \
        --port 5555 \
        --debug

Author: Antigravity Assistant
Date: 2026-02-09
"""

import argparse
import time
import sys
from typing import Dict, Any, Optional

import zmq
import msgpack
import numpy as np


class VLAInferenceServer:
    """VLA 모델 추론 서버 (ZeroMQ REP 패턴)"""
    
    def __init__(
        self,
        policy_path: str,
        port: int = 5555,
        device: str = 'cuda',
        image_size: int = 256
    ):
        """
        Args:
            policy_path: SmolVLA 체크포인트 경로
            port: ZeroMQ 서버 포트 (localhost에서만 바인딩)
            device: GPU 디바이스 ('cuda' 또는 'cuda:0' 등)
            image_size: 입력 이미지 크기 (기본값 256x256)
        """
        self.device = device
        self.port = port
        self.image_size = image_size
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None
        
        # ZeroMQ 설정
        self._setup_zmq()
        
        # SmolVLA 정책 로드
        self._load_policy(policy_path)
        
        # 통계
        self.inference_count = 0
        self.total_inference_time = 0.0
        
        self._print_banner()
    
    def _setup_zmq(self):
        """ZeroMQ REP 소켓 설정"""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REP)
        # localhost에서만 바인딩 (SSH 터널을 통해 접근)
        self.socket.bind(f"tcp://localhost:{self.port}")
        print(f"🔌 ZeroMQ REP 소켓 바인딩: tcp://localhost:{self.port}")
    
    def _load_policy(self, policy_path: str):
        """SmolVLA 정책 로드"""
        print(f"\n🔄 SmolVLA 정책 로딩 중...")
        print(f"   경로: {policy_path}")
        
        try:
            import torch
            from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
            from lerobot.policies.factory import make_pre_post_processors
            
            # 정책 로드
            self.policy = SmolVLAPolicy.from_pretrained(policy_path)
            self.policy.to(self.device)
            self.policy.eval()
            
            # 전처리/후처리기 생성
            self.preprocessor, self.postprocessor = make_pre_post_processors(
                self.policy.config,
                pretrained_path=policy_path
            )
            
            print(f"✅ 정책 로드 완료!")
            print(f"   디바이스: {self.device}")
            print(f"   State dim: {self.policy.config.input_shapes.get('observation.state', 'N/A')}")
            print(f"   Action dim: {self.policy.config.output_shapes.get('action', 'N/A')}")
            
        except ImportError as e:
            print(f"❌ LeRobot 패키지 임포트 실패: {e}")
            print("   pip install lerobot==0.4.3 를 실행하세요.")
            sys.exit(1)
        except Exception as e:
            print(f"❌ 정책 로드 실패: {e}")
            sys.exit(1)
    
    def _print_banner(self):
        """서버 시작 배너 출력"""
        print("\n" + "=" * 60)
        print("  🤖 VLA Inference Server")
        print("=" * 60)
        print(f"  📡 ZeroMQ 포트: localhost:{self.port}")
        print(f"  🎮 디바이스: {self.device}")
        print(f"  🖼️  이미지 크기: {self.image_size}x{self.image_size}")
        print("=" * 60)
        print("\n⏳ 클라이언트 요청 대기 중...\n")
    
    def _parse_request(self, raw_data: bytes) -> Dict[str, Any]:
        """요청 데이터 파싱"""
        data = msgpack.unpackb(raw_data, raw=False)
        
        # 이미지 복원 (bytes → numpy array)
        images = {}
        for key, img_bytes in data.get('images', {}).items():
            img_array = np.frombuffer(img_bytes, dtype=np.uint8)
            img_array = img_array.reshape(self.image_size, self.image_size, 3)
            images[key] = img_array
        
        # 상태 벡터
        state = np.array(data.get('state', []), dtype=np.float32)
        
        # 태스크 설명
        task = data.get('task', 'manipulation task')
        
        return {
            'images': images,
            'state': state,
            'task': task,
        }
    
    def _run_inference(self, parsed_data: Dict[str, Any]) -> np.ndarray:
        """SmolVLA 추론 실행"""
        import torch
        
        images = parsed_data['images']
        state = parsed_data['state']
        task = parsed_data['task']
        
        # 관측 구성 (SmolVLA 16-dim 형식)
        observation = {
            'observation.state': state,
            'task': task,
        }
        
        # 카메라 이미지 추가
        camera_mapping = {
            'top': 'observation.images.top',
            'wrist_left': 'observation.images.wrist_left',
            'wrist_right': 'observation.images.wrist_right',
        }
        
        for src_key, dst_key in camera_mapping.items():
            if src_key in images:
                observation[dst_key] = images[src_key]
        
        # 전처리
        batch = self.preprocessor(observation)
        
        # 추론
        with torch.no_grad():
            action = self.policy.select_action(batch)
        
        # 후처리
        action = self.postprocessor(action)
        
        # numpy 변환
        if isinstance(action, torch.Tensor):
            action_np = action.squeeze().cpu().numpy()
        else:
            action_np = np.array(action).squeeze()
        
        return action_np
    
    def _send_response(self, action: np.ndarray, inference_time: float, status: str = 'ok'):
        """응답 전송"""
        response = {
            'action': action.tolist() if action is not None else [],
            'inference_time_ms': inference_time * 1000,
            'status': status,
        }
        self.socket.send(msgpack.packb(response))
    
    def _send_error(self, message: str):
        """에러 응답 전송"""
        response = {
            'action': [],
            'inference_time_ms': 0,
            'status': 'error',
            'message': message,
        }
        self.socket.send(msgpack.packb(response))
    
    def run(self, debug: bool = False):
        """메인 서버 루프"""
        print("🚀 서버 루프 시작!")
        
        while True:
            try:
                # 1. 요청 수신
                raw_data = self.socket.recv()
                recv_time = time.time()
                
                # 2. 데이터 파싱
                parsed_data = self._parse_request(raw_data)
                
                if debug:
                    print(f"\n📥 요청 수신:")
                    print(f"   이미지: {list(parsed_data['images'].keys())}")
                    print(f"   상태: shape={parsed_data['state'].shape}")
                    print(f"   태스크: '{parsed_data['task'][:50]}...'")
                
                # 3. 추론 실행
                action = self._run_inference(parsed_data)
                inference_time = time.time() - recv_time
                
                # 4. 응답 전송
                self._send_response(action, inference_time)
                
                # 통계 업데이트
                self.inference_count += 1
                self.total_inference_time += inference_time
                
                if debug:
                    print(f"📤 응답 전송:")
                    print(f"   액션: {action[:4]}... (shape={action.shape})")
                    print(f"   추론 시간: {inference_time * 1000:.1f}ms")
                
                # 주기적 통계 출력 (50회마다)
                if self.inference_count % 50 == 0:
                    avg_time = self.total_inference_time / self.inference_count * 1000
                    print(f"\n📊 통계: {self.inference_count}회 추론 완료, "
                          f"평균 {avg_time:.1f}ms/추론")
                    
            except KeyboardInterrupt:
                print("\n\n🛑 서버 종료 요청 (Ctrl+C)")
                break
                
            except Exception as e:
                print(f"\n❌ 오류 발생: {e}")
                import traceback
                traceback.print_exc()
                self._send_error(str(e))
        
        # 정리
        self._cleanup()
    
    def _cleanup(self):
        """리소스 정리"""
        print("\n🧹 리소스 정리 중...")
        self.socket.close()
        self.context.term()
        print("✅ 서버 종료 완료")


def main():
    parser = argparse.ArgumentParser(
        description='VLA Inference Server - SmolVLA 원격 추론 서버',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  # 기본 실행
  python vla_inference_server.py --policy_path ~/checkpoints/smolvla

  # 디버그 모드
  python vla_inference_server.py --policy_path ~/checkpoints/smolvla --debug

  # 다른 포트 사용
  python vla_inference_server.py --policy_path ~/checkpoints/smolvla --port 5556
        """
    )
    
    parser.add_argument(
        '--policy_path',
        type=str,
        required=True,
        help='SmolVLA 체크포인트 경로 (필수)'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=5555,
        help='ZeroMQ 서버 포트 (기본값: 5555)'
    )
    parser.add_argument(
        '--device',
        type=str,
        default='cuda',
        help='GPU 디바이스 (기본값: cuda)'
    )
    parser.add_argument(
        '--image_size',
        type=int,
        default=256,
        help='입력 이미지 크기 (기본값: 256)'
    )
    parser.add_argument(
        '--debug',
        action='store_true',
        help='디버그 모드 활성화 (상세 로그 출력)'
    )
    
    args = parser.parse_args()
    
    # 서버 생성 및 실행
    server = VLAInferenceServer(
        policy_path=args.policy_path,
        port=args.port,
        device=args.device,
        image_size=args.image_size
    )
    
    server.run(debug=args.debug)


if __name__ == '__main__':
    main()
