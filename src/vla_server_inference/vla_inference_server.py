#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLA Inference Server - GPU 서버에서 실행

ZeroMQ REP 소켓을 통해 로봇 laptop으로부터 관측 데이터를 수신하고,
VLA 모델(SmolVLA, Pi0, GROOT N1.5, FMVLA) 추론 결과(action)를 반환합니다.

사용법:
    python vla_inference_server.py \
        --policy_path ~/OpenArm0.3_data/checkpoints/smolvla_openarm_16dim/pretrained_model \
        --port 5555 \
        --model_type smolvla \
        --debug

Author: Antigravity Assistant
Date: 2026-02-09
"""

import argparse
import time
import sys
import threading
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
        image_size: int = 256,
        model_type: str = 'smolvla',
        fmvla_sd3_model_path: str = "stabilityai/stable-diffusion-3-medium-diffusers",
        fmvla_lora_weights_path: Optional[str] = None,
        fmvla_lora_scale: float = 1.0,
        fmvla_enable_sd3_cpu_offload: bool = True,
        fmvla_precomputed_dir: Optional[str] = None,
        fmvla_precomputed_only: Optional[bool] = None,
        fmvla_chunk_size: Optional[int] = None,
        fmvla_n_action_steps: Optional[int] = None,
        fmvla_hold_on_chunk_boundary: bool = True,
        fmvla_hold_max_sec: float = 0.0,
    ):
        """
        Args:
            policy_path: 모델 체크포인트 경로
            port: ZeroMQ 서버 포트 (localhost에서만 바인딩)
            device: GPU 디바이스 ('cuda' 또는 'cuda:0' 등)
            image_size: 입력 이미지 크기 (기본값 256x256)
            model_type: 모델 타입 ('smolvla', 'pi0', 'groot', 또는 'fmvla')
        """
        self.device = device
        self.port = port
        self.image_size = image_size
        self.model_type = model_type
        self.fmvla_sd3_model_path = fmvla_sd3_model_path
        self.fmvla_lora_weights_path = fmvla_lora_weights_path
        self.fmvla_lora_scale = fmvla_lora_scale
        self.fmvla_enable_sd3_cpu_offload = fmvla_enable_sd3_cpu_offload
        self.fmvla_precomputed_dir = fmvla_precomputed_dir
        self.fmvla_precomputed_only = fmvla_precomputed_only
        self.fmvla_chunk_size = fmvla_chunk_size
        self.fmvla_n_action_steps = fmvla_n_action_steps
        self.fmvla_hold_on_chunk_boundary = fmvla_hold_on_chunk_boundary
        self.fmvla_hold_max_sec = fmvla_hold_max_sec
        self.debug = False
        self.policy = None
        self.preprocessor = None
        self.postprocessor = None

        # FMVLA hold-last-action state (used only when model_type='fmvla')
        self._fmvla_lock = threading.Lock()
        self._fmvla_worker_running = False
        self._fmvla_worker_thread = None
        self._fmvla_pending_first_action = None
        self._fmvla_pending_error = None
        self._fmvla_last_action = None
        self._fmvla_hold_start_ts = None
        self._fmvla_last_hold_log_ts = 0.0
        self._fmvla_last_hold_warn_ts = 0.0
        
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
        """모델 정책 로드"""
        print(f"\n🔄 {self.model_type} 정책 로딩 중...")
        print(f"   경로: {policy_path}")
        
        try:
            import torch
            from lerobot.policies.factory import make_pre_post_processors
            
            # 정책 로드
            if self.model_type == 'smolvla':
                from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
                self.policy = SmolVLAPolicy.from_pretrained(policy_path)
            elif self.model_type == 'pi0':
                from lerobot.policies.pi0.modeling_pi0 import PI0Policy
                self.policy = PI0Policy.from_pretrained(policy_path)
            elif self.model_type == 'groot':
                from lerobot.policies.groot.modeling_groot import GrootPolicy
                self.policy = GrootPolicy.from_pretrained(policy_path)
            elif self.model_type == 'fmvla':
                # Register FMVLA processor step (fmvla_task_processor) before loading
                import lerobot.policies.fmvla.processor_fmvla  # noqa: F401
                from lerobot.configs.policies import PreTrainedConfig
                from lerobot.policies.fmvla.modeling_fmvla import FMVLA_Policy

                cli_overrides = ["--random_init_gemma_expert=false"]
                if self.fmvla_precomputed_dir:
                    cli_overrides.append(f"--precomputed_dir={self.fmvla_precomputed_dir}")
                else:
                    cli_overrides.append("--precomputed_dir=null")
                if self.fmvla_chunk_size is not None:
                    cli_overrides.append(f"--chunk_size={self.fmvla_chunk_size}")
                if self.fmvla_n_action_steps is not None:
                    cli_overrides.append(f"--n_action_steps={self.fmvla_n_action_steps}")
                if self.fmvla_lora_weights_path:
                    cli_overrides.append(f"--lora_weights_path={self.fmvla_lora_weights_path}")
                cli_overrides.append(f"--lora_scale={self.fmvla_lora_scale}")

                fmvla_config = PreTrainedConfig.from_pretrained(
                    policy_path,
                    cli_overrides=cli_overrides,
                )

                self.policy = FMVLA_Policy.from_pretrained(
                    pretrained_name_or_path=policy_path,
                    config=fmvla_config,
                    sd3_model_path=self.fmvla_sd3_model_path,
                    enable_sd3_cpu_offload=self.fmvla_enable_sd3_cpu_offload,
                    lora_weights_path=self.fmvla_lora_weights_path,
                    lora_scale=self.fmvla_lora_scale,
                    precomputed_dir=self.fmvla_precomputed_dir,
                    precomputed_only=self.fmvla_precomputed_only,
                )
            else:
                raise ValueError(f"지원하지 않는 모델 타입: {self.model_type}")
            self.policy.to(self.device)
            self.policy.eval()
            
            # 전처리/후처리기 생성
            preprocessor_overrides = {}
            if self.model_type == "fmvla":
                # Ensure runtime device matches CLI device argument.
                preprocessor_overrides["device_processor"] = {"device": str(self.device)}

            self.preprocessor, self.postprocessor = make_pre_post_processors(
                self.policy.config,
                pretrained_path=policy_path,
                preprocessor_overrides=preprocessor_overrides,
            )
            
            print(f"✅ 정책 로드 완료!")
            print(f"   디바이스: {self.device}")
            input_shapes = getattr(self.policy.config, 'input_shapes', {})
            output_shapes = getattr(self.policy.config, 'output_shapes', {})
            print(f"   State dim: {input_shapes.get('observation.state', 'N/A')}")
            print(f"   Action dim: {output_shapes.get('action', 'N/A')}")
            if self.model_type == 'fmvla' and self.fmvla_hold_on_chunk_boundary:
                hold_sec = "infinite" if self.fmvla_hold_max_sec <= 0 else f"{self.fmvla_hold_max_sec:.2f}s"
                print(f"   FMVLA hold-last-action: enabled (max hold: {hold_sec})")
            
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
    
    def _prepare_batch(self, parsed_data: Dict[str, Any]) -> Dict[str, Any]:
        """요청 데이터를 정책 입력 배치로 변환"""
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
            # Make copy to ensure writable memory (fixes UserWarning)
            img_tensor = torch.from_numpy(img_array.copy()).permute(2, 0, 1).float() / 255.0
            # 배치 차원 추가: (1, C, H, W)
            img_tensor = img_tensor.unsqueeze(0).to(self.device)
            # 관측값에 추가 (키 그대로 사용)
            observation[key] = img_tensor
        
        # 전처리 (이미 배치화된 Tensor가 들어옴)
        # 중요: smolvla_inference_node.py (로컬 노드)와 동일하게 전처리기 통과
        if self.preprocessor:
             batch = self.preprocessor(observation)
        else:
             batch = observation
            
        return batch

    def _finalize_action(self, action):
        """정책 출력 action 후처리 및 numpy 변환"""
        import torch

        # [Emergency Fix] Action 차원 불일치 해결 (32 -> 16)
        # PI0/FMVLA max_action_dim=32 케이스에서 OpenArm 제어 차원(16)으로 맞춤
        if hasattr(action, "shape") and action.shape[-1] == 32:
            action = action[..., :16]

        # 후처리 (unnormalize 등)
        if self.postprocessor:
            action = self.postprocessor(action)
        
        # numpy 변환
        if isinstance(action, torch.Tensor):
            action = action.squeeze().cpu().numpy()
        else:
            action = np.array(action).squeeze()
        
        return action

    def _fmvla_queue_len(self) -> int:
        """FMVLA 내부 action queue 길이 반환"""
        queue = getattr(self.policy, "_action_queue", None)
        if queue is None:
            return 0
        return len(queue)

    def _set_fmvla_last_action(self, action_np: np.ndarray):
        """FMVLA hold용 마지막 action 저장"""
        self._fmvla_last_action = np.array(action_np, dtype=np.float32, copy=True)

    def _start_fmvla_chunk_worker(self, batch: Dict[str, Any]):
        """FMVLA 다음 chunk 비동기 생성 시작"""
        if self._fmvla_worker_running:
            return

        self._fmvla_worker_running = True

        def _worker():
            import torch
            try:
                with torch.no_grad():
                    raw_action = self.policy.select_action(batch)
                next_action = self._finalize_action(raw_action)
                with self._fmvla_lock:
                    self._fmvla_pending_first_action = np.array(next_action, dtype=np.float32, copy=True)
                    self._fmvla_pending_error = None
                    self._fmvla_worker_running = False
            except Exception as worker_error:  # pragma: no cover - runtime safety path
                with self._fmvla_lock:
                    self._fmvla_pending_error = str(worker_error)
                    self._fmvla_worker_running = False

        self._fmvla_worker_thread = threading.Thread(target=_worker, daemon=True)
        self._fmvla_worker_thread.start()

    def _run_inference_fmvla_hold(self, parsed_data: Dict[str, Any]) -> np.ndarray:
        """
        FMVLA 전용 hold-last-action 추론:
        - queue에 action이 남아있으면 즉시 소비
        - queue가 비면 마지막 action 유지 + 다음 chunk 비동기 생성
        """
        import torch

        mode = None
        hold_action = None
        pending_action = None

        with self._fmvla_lock:
            if self._fmvla_pending_error:
                # 안전 우선: 오류가 나도 즉시 실패 응답 대신 마지막 action hold 유지
                print(f"⚠ FMVLA async chunk 생성 오류: {self._fmvla_pending_error}")
                self._fmvla_pending_error = None

            if self._fmvla_pending_first_action is not None:
                pending_action = self._fmvla_pending_first_action
                self._fmvla_pending_first_action = None
                self._fmvla_hold_start_ts = None
                mode = "use_pending_first_action"
            elif self._fmvla_worker_running:
                hold_action = None if self._fmvla_last_action is None else self._fmvla_last_action.copy()
                mode = "hold_while_worker_running"
            else:
                queue_len = self._fmvla_queue_len()
                if queue_len > 0:
                    mode = "consume_from_queue"
                elif self._fmvla_last_action is None:
                    # 첫 요청에는 hold할 action이 없으므로 동기 1회 부트스트랩
                    mode = "bootstrap_sync_first_action"
                else:
                    mode = "start_worker_and_hold"

        if mode == "use_pending_first_action":
            self._set_fmvla_last_action(pending_action)
            if self.debug:
                print("▶ FMVLA async chunk 준비 완료: 새 chunk action 재개")
            return pending_action

        if mode == "consume_from_queue":
            batch = self._prepare_batch(parsed_data)
            with torch.no_grad():
                raw_action = self.policy.select_action(batch)
            action = self._finalize_action(raw_action)
            with self._fmvla_lock:
                self._set_fmvla_last_action(action)
            return action

        if mode == "bootstrap_sync_first_action":
            batch = self._prepare_batch(parsed_data)
            with torch.no_grad():
                raw_action = self.policy.select_action(batch)
            action = self._finalize_action(raw_action)
            with self._fmvla_lock:
                self._set_fmvla_last_action(action)
            if self.debug:
                print("▶ FMVLA 첫 action 동기 부트스트랩 완료")
            return action

        if mode == "start_worker_and_hold":
            batch = self._prepare_batch(parsed_data)
            with self._fmvla_lock:
                # lock 재진입 시점에서 상태 재확인
                if self._fmvla_pending_first_action is not None:
                    pending_action = self._fmvla_pending_first_action
                    self._fmvla_pending_first_action = None
                    self._fmvla_hold_start_ts = None
                else:
                    if not self._fmvla_worker_running:
                        self._start_fmvla_chunk_worker(batch)
                        self._fmvla_hold_start_ts = time.time()
                    hold_action = self._fmvla_last_action.copy()

            if pending_action is not None:
                self._set_fmvla_last_action(pending_action)
                return pending_action

            now = time.time()
            hold_elapsed = 0.0
            if self._fmvla_hold_start_ts is not None:
                hold_elapsed = now - self._fmvla_hold_start_ts

            if self.debug and (now - self._fmvla_last_hold_log_ts) >= 1.0:
                print(f"⏸ FMVLA hold-last-action 유지 중... ({hold_elapsed:.2f}s)")
                self._fmvla_last_hold_log_ts = now

            if self.fmvla_hold_max_sec > 0 and hold_elapsed > self.fmvla_hold_max_sec:
                if (now - self._fmvla_last_hold_warn_ts) >= 5.0:
                    print(
                        f"⚠ FMVLA hold 시간이 {self.fmvla_hold_max_sec:.2f}s를 초과했습니다. "
                        "설정상 무기한 hold가 아니므로 상태를 점검하세요."
                    )
                    self._fmvla_last_hold_warn_ts = now

            return hold_action

        # mode == "hold_while_worker_running"
        if hold_action is None:
            raise RuntimeError("FMVLA hold 상태에서 last_action이 없습니다. 초기화 경로를 점검하세요.")

        now = time.time()
        if self._fmvla_hold_start_ts is not None:
            hold_elapsed = now - self._fmvla_hold_start_ts
        else:
            hold_elapsed = 0.0

        if self.debug and (now - self._fmvla_last_hold_log_ts) >= 1.0:
            print(f"⏸ FMVLA worker 실행 중, hold-last-action 유지... ({hold_elapsed:.2f}s)")
            self._fmvla_last_hold_log_ts = now

        return hold_action

    def _run_inference(self, parsed_data: Dict[str, Any]) -> np.ndarray:
        """VLA 추론 실행"""
        import torch

        # FMVLA 전용: queue 경계 hold-last-action 모드
        if self.model_type == "fmvla" and self.fmvla_hold_on_chunk_boundary:
            return self._run_inference_fmvla_hold(parsed_data)

        # 기본(기존) 동작: 요청마다 즉시 select_action
        batch = self._prepare_batch(parsed_data)

        with torch.no_grad():
            raw_action = self.policy.select_action(batch)

        return self._finalize_action(raw_action)
    
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
        self.debug = debug
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
        description='VLA Inference Server - SmolVLA/Pi0/GROOT/FMVLA 원격 추론 서버',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  # SmolVLA 실행 (기본값)
  python vla_inference_server.py --policy_path ~/checkpoints/smolvla

  # Pi0 실행
  python vla_inference_server.py --policy_path ~/checkpoints/pi0 --model_type pi0

  # GROOT N1.5 실행
  python vla_inference_server.py --policy_path ~/checkpoints/groot --model_type groot

  # FMVLA 실행 (전용 환경 vla_server_fmvla 권장)
  python vla_inference_server.py --policy_path ~/checkpoints/fmvla/pretrained_model --model_type fmvla

  # 디버그 모드 + 다른 포트
  python vla_inference_server.py --policy_path ~/checkpoints/smolvla --debug --port 5556
        """
    )
    
    parser.add_argument(
        '--policy_path',
        type=str,
        required=True,
        help='모델 체크포인트 경로 (필수)'
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
    parser.add_argument(
        '--model_type',
        type=str,
        default='smolvla',
        choices=['smolvla', 'pi0', 'groot', 'fmvla'],
        help="모델 타입 선택: 'smolvla', 'pi0', 'groot', 또는 'fmvla' (기본값: smolvla)"
    )
    parser.add_argument(
        '--fmvla_sd3_model_path',
        type=str,
        default='stabilityai/stable-diffusion-3-medium-diffusers',
        help='FMVLA 전용: SD3 모델 경로 또는 HF repo id'
    )
    parser.add_argument(
        '--fmvla_lora_weights_path',
        type=str,
        default=None,
        help='FMVLA 전용: Vision Planner LoRA 가중치 경로'
    )
    parser.add_argument(
        '--fmvla_lora_scale',
        type=float,
        default=1.0,
        help='FMVLA 전용: LoRA scale (기본값: 1.0)'
    )
    parser.add_argument(
        '--fmvla_enable_sd3_cpu_offload',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='FMVLA 전용: SD3 CPU offload 사용 여부 (기본값: True)'
    )
    parser.add_argument(
        '--fmvla_precomputed_dir',
        type=str,
        default=None,
        help='FMVLA 전용: precomputed 키/서브골 디렉토리'
    )
    parser.add_argument(
        '--fmvla_precomputed_only',
        dest='fmvla_precomputed_only',
        action='store_true',
        default=None,
        help='FMVLA 전용: precomputed-only 모드 강제 활성화'
    )
    parser.add_argument(
        '--fmvla_no_precomputed_only',
        dest='fmvla_precomputed_only',
        action='store_false',
        help='FMVLA 전용: precomputed-only 모드 비활성화'
    )
    parser.add_argument(
        '--fmvla_chunk_size',
        type=int,
        default=None,
        help='FMVLA 전용: chunk_size override'
    )
    parser.add_argument(
        '--fmvla_n_action_steps',
        type=int,
        default=None,
        help='FMVLA 전용: n_action_steps override'
    )
    parser.add_argument(
        '--fmvla_hold_on_chunk_boundary',
        action=argparse.BooleanOptionalAction,
        default=True,
        help='FMVLA 전용: action queue 경계에서 hold-last-action 사용 여부 (기본값: True)'
    )
    parser.add_argument(
        '--fmvla_hold_max_sec',
        type=float,
        default=0.0,
        help='FMVLA 전용: hold 최대 시간(초). 0 이하는 무기한 hold (기본값: 0.0)'
    )
    
    args = parser.parse_args()
    
    # 서버 생성 및 실행
    server = VLAInferenceServer(
        policy_path=args.policy_path,
        port=args.port,
        device=args.device,
        image_size=args.image_size,
        model_type=args.model_type,
        fmvla_sd3_model_path=args.fmvla_sd3_model_path,
        fmvla_lora_weights_path=args.fmvla_lora_weights_path,
        fmvla_lora_scale=args.fmvla_lora_scale,
        fmvla_enable_sd3_cpu_offload=args.fmvla_enable_sd3_cpu_offload,
        fmvla_precomputed_dir=args.fmvla_precomputed_dir,
        fmvla_precomputed_only=args.fmvla_precomputed_only,
        fmvla_chunk_size=args.fmvla_chunk_size,
        fmvla_n_action_steps=args.fmvla_n_action_steps,
        fmvla_hold_on_chunk_boundary=args.fmvla_hold_on_chunk_boundary,
        fmvla_hold_max_sec=args.fmvla_hold_max_sec,
    )
    
    server.run(debug=args.debug)


if __name__ == '__main__':
    main()
