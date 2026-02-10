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
            input_shapes = getattr(self.policy.config, 'input_shapes', {})
            output_shapes = getattr(self.policy.config, 'output_shapes', {})
            print(f"   State dim: {input_shapes.get('observation.state', 'N/A')}")
            print(f"   Action dim: {output_shapes.get('action', 'N/A')}")
            
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
        # flat key 구조 처리 (observation.images.camera1 등)
        for key, value in data.items():
            if key.startswith('observation.images.'):
                img_bytes = value
                img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                img_array = img_array.reshape(self.image_size, self.image_size, 3)
                images[key] = img_array
        
        # 이전 'images' 중첩 구조도 하위 호환성을 위해 유지 (선택사항)
        if 'images' in data:
            for key, img_bytes in data['images'].items():
                img_array = np.frombuffer(img_bytes, dtype=np.uint8)
                img_array = img_array.reshape(self.image_size, self.image_size, 3)
                # 만약 key가 observation.images.로 시작하지 않으면 붙여줌 (기존 로직 호환)
                full_key = key if key.startswith('observation.images.') else f'observation.images.{key}'
                images[full_key] = img_array
        
        # 상태 벡터
        # observation.state 키로 직접 들어올 수도 있고, 'state' 키로 들어올 수도 있음
        if 'observation.state' in data:
            state = np.array(data['observation.state'], dtype=np.float32)
        else:
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
        
        # 1. 상태(State) 처리: (dim,) -> (1, dim) Tensor
        state_tensor = torch.from_numpy(state).float().to(self.device).unsqueeze(0)
        
        # 관측값 구성
        observation = {
            'observation.state': state_tensor,
            'task': task,  # 문자열은 그대로 (배치 처리는 내부에서)
        }
        
        # 2. 이미지 처리: (H, W, C) -> (1, C, H, W) Tensor + 0-1 정규화
        # images 딕셔너리는 이미 'observation.images.cameraX' 형태의 키를 가짐
        for key, img_array in images.items():
            # numpy (H, W, C) -> torch (C, H, W)
            img_tensor = torch.from_numpy(img_array).permute(2, 0, 1).float() / 255.0
            # 배치 차원 추가: (1, C, H, W)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)
            # 관측값에 추가 (키 그대로 사용)
            observation[key] = img_tensor
        
        # 전처리 (이미 배치화된 Tensor가 들어옴)
        # 주의: LeRobot의 preprocessor는 데이터셋 아이템(dict)을 기대할 수도 있음
        # 하지만 이미 Tensor로 변환했으므로, 여기서는 필요한 추가 정규화만 수행하거나 패스
        # 만약 preprocessor가 None이거나 Tensor 입력을 처리한다면 그대로 사용
        if self.preprocessor:
             # preprocessor가 배치 입력을 처리하는지 확인 필요.
             # 보통은 데이터셋 아이템(Unbatched) -> Batched로 변환하는데,
             # 여기서는 직접 Batched Tensor를 만들었으므로 preprocessor 호출 방식에 주의
             # 일단은 policy에 직접 넣기 위해 preprocessor 통과 (필요 시)
             # 하지만 VLA 모델은 보통 normalize를 내부에서 하거나 전처리기가 함.
             # 수동으로 0-1 정규화했으니, preprocessor가 중복 정규화하지 않도록 주의.
             # *SmolVLA*의 경우 전처리기가 복잡할 수 있음.
             # 안전하게는: preprocessor 호출 없이 직접 포맷팅했으므로 바로 사용하거나,
             # preprocessor가 (C,H,W) 입력을 기대한다면 배치 차원 추가 전 호출해야 함.
             pass

        # SmolVLA의 경우, preprocessor가 토크나이징 등을 수행할 수 있으므로 호출 필요.
        # 단, state/image가 이미 Tensor라면?
        # -> SmolVLA preprocessor(normalization)는 보통 데이터셋 로더에서 수행됨.
        # -> 추론 시에는 모델이 기대하는 raw input(0-1 float)을 넣어주면 됨.
        
        # 3. 추론 실행
        # policy.select_action은 배치를 기대함
        with torch.no_grad():
            # 태스크가 문자열 리스트로 들어가야 함 (배치 크기 1)
            # observation['task'] = [task] 
            # (SmolVLA 내부 구현에 따라 다름, 보통 텍스트는 리스트)
            
            # 만약 preprocessor를 꼭 써야한다면:
            # batch = self.preprocessor(observation) 
            # 하지만 위 코드는 데이터셋용일 수 있음.
            
            # 직접 구성한 batch 사용
            batch = observation
            
            # Action 추론
            action = self.policy.select_action(batch)
        
        # 후처리
        if self.postprocessor:
            action = self.postprocessor(action)
        
        # numpy 변환
        if isinstance(action, torch.Tensor):
            action = action.squeeze().cpu().numpy()
        else:
            action = np.array(action).squeeze()
        
        return action
    
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
