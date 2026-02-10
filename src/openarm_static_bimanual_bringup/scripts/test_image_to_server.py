#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
카메라 이미지 → VLA 서버 전송 테스트 스크립트

로봇 하드웨어 없이 카메라 이미지만으로 서버 통신을 테스트합니다.
joint_states는 더미 데이터(0.0)로 채워서 전송합니다.

사용법:
    # ROS2 환경 소싱 후
    source ~/OpenArm0.3_data/install/setup.bash
    python3 ~/OpenArm0.3_data/src/openarm_static_bimanual_bringup/scripts/test_image_to_server.py

    # 포트 변경 시
    python3 test_image_to_server.py --port 5555
"""

import argparse
import time
import threading
from typing import Dict, Optional

import zmq
import msgpack
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage


class ImageToServerTester(Node):
    """카메라 이미지 → VLA 서버 전송 테스트 노드"""

    # Use keys expected by the VLA server: camera1, camera2, camera3
    CAMERA_TOPICS = {
        'camera1': '/camera/cam_1/color/image_raw/compressed',
        'camera2': '/camera/cam_2/color/image_raw/compressed',
        'camera3': '/camera/cam_3/color/image_raw/compressed',
    }

    def __init__(self, port: int = 5555, image_size: int = 256, timeout_ms: int = 5000):
        super().__init__('image_to_server_tester')

        self.port = port
        self.image_size = image_size
        self.timeout_ms = timeout_ms

        # 이미지 저장
        self.image_lock = threading.Lock()
        self.latest_images: Dict[str, Optional[np.ndarray]] = {k: None for k in self.CAMERA_TOPICS}
        self.image_timestamps: Dict[str, float] = {k: 0.0 for k in self.CAMERA_TOPICS}

        # 카메라 구독
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        for key, topic in self.CAMERA_TOPICS.items():
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, k=key: self._camera_callback(msg, k),
                qos
            )

        # ZeroMQ 설정
        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.REQ)
        self.zmq_socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self.zmq_socket.setsockopt(zmq.SNDTIMEO, timeout_ms)

        # 테스트 타이머 (2초마다)
        self.test_timer = self.create_timer(2.0, self._test_callback)
        self.test_count = 0

        self._print_banner()

    def _print_banner(self):
        self.get_logger().info('=' * 60)
        self.get_logger().info('  🧪 카메라 → VLA 서버 전송 테스트')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  📡 서버: tcp://localhost:{self.port}')
        self.get_logger().info(f'  📷 이미지 크기: {self.image_size}x{self.image_size}')
        self.get_logger().info(f'  ⏱️  타임아웃: {self.timeout_ms}ms')
        self.get_logger().info(f'  🦾 joint_states: 더미 데이터 (16-dim zeros)')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')
        self.get_logger().info('🔍 카메라 이미지 수신 대기 중...')

    def _camera_callback(self, msg: CompressedImage, camera_key: str):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if image is not None:
                with self.image_lock:
                    self.latest_images[camera_key] = image
                    self.image_timestamps[camera_key] = time.time()
        except Exception as e:
            self.get_logger().error(f'카메라 {camera_key} 처리 오류: {e}')

    def _test_callback(self):
        """2초마다 상태 확인 및 서버 전송 테스트"""
        self.test_count += 1

        # ── 1단계: 카메라 이미지 수신 상태 체크 ──
        self.get_logger().info(f'\n--- 테스트 #{self.test_count} ---')

        received = {}
        missing = []
        with self.image_lock:
            for key in self.CAMERA_TOPICS:
                if self.latest_images[key] is not None:
                    h, w = self.latest_images[key].shape[:2]
                    age = time.time() - self.image_timestamps[key]
                    received[key] = (w, h, age)
                else:
                    missing.append(key)

        for key, (w, h, age) in received.items():
            self.get_logger().info(f'  ✅ {key}: {w}x{h}, {age:.1f}초 전 수신')
        for key in missing:
            self.get_logger().warn(f'  ❌ {key}: 이미지 없음')

        if missing:
            self.get_logger().warn(f'⏳ 카메라 이미지 부족 ({len(missing)}개 누락), 서버 전송 건너뜀')
            return

        # ── 2단계: 서버 전송 테스트 ──
        self.get_logger().info('📤 서버로 전송 시도...')

        try:
            # 이미지 준비
            images_bytes = {}
            with self.image_lock:
                for key, image in self.latest_images.items():
                    resized = cv2.resize(image, (self.image_size, self.image_size))
                    rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                    images_bytes[key] = rgb.tobytes()

            # 더미 joint states (16-dim zeros)
            dummy_state = [0.0] * 16

            # Build request using keys expected by the VLA server
            # Server expects observation.state and observation.images.<camera>
            request = {
                'observation.state': dummy_state,
                'task': 'test: camera image transmission check',
            }
            for k, v in images_bytes.items():
                request[f'observation.images.{k}'] = v

            # ZeroMQ 연결 (매번 새로 연결)
            self.zmq_socket.close()
            self.zmq_socket = self.zmq_context.socket(zmq.REQ)
            self.zmq_socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.zmq_socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self.zmq_socket.connect(f"tcp://localhost:{self.port}")

            # 전송
            packed = msgpack.packb(request)
            data_size_kb = len(packed) / 1024
            start = time.time()
            self.zmq_socket.send(packed)
            self.get_logger().info(f'  📦 데이터 전송 완료 ({data_size_kb:.1f} KB)')

            # 응답 수신
            response = msgpack.unpackb(self.zmq_socket.recv(), raw=False)
            rtt = (time.time() - start) * 1000

            if response.get('status') == 'ok':
                action = response.get('action', [])
                server_time = response.get('inference_time_ms', 0)
                self.get_logger().info(f'  ✅ 서버 응답 성공!')
                self.get_logger().info(f'     - RTT: {rtt:.1f}ms (서버 추론: {server_time:.1f}ms)')
                self.get_logger().info(f'     - Action 차원: {len(action)}')
                if len(action) >= 16:
                    self.get_logger().info(
                        f'     - Action 샘플: L_arm={action[:3]}, R_arm={action[8:11]}'
                    )
                self.get_logger().info('')
                self.get_logger().info('🎉 카메라 이미지가 서버로 정상 전달되고 있습니다!')
            else:
                err = response.get('message', 'Unknown')
                self.get_logger().error(f'  ❌ 서버 오류 응답: {err}')

        except zmq.Again:
            self.get_logger().error('  ❌ 서버 응답 타임아웃!')
            self.get_logger().error('     → SSH 터널이 연결되어 있는지 확인: lsof -i :5555')
            self.get_logger().error('     → VLA 서버가 실행 중인지 확인')
        except zmq.ZMQError as e:
            self.get_logger().error(f'  ❌ ZeroMQ 오류: {e}')
            self.get_logger().error('     → SSH 터널 상태 확인 필요')
        except Exception as e:
            self.get_logger().error(f'  ❌ 예외 발생: {e}')

    def destroy_node(self):
        self.zmq_socket.close()
        self.zmq_context.term()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description='카메라 → VLA 서버 전송 테스트')
    parser.add_argument('--port', type=int, default=5555, help='ZeroMQ 포트 (기본: 5555)')
    parser.add_argument('--image-size', type=int, default=256, help='이미지 크기 (기본: 256)')
    parser.add_argument('--timeout', type=int, default=5000, help='타임아웃 ms (기본: 5000)')
    args = parser.parse_args()

    rclpy.init()
    node = ImageToServerTester(
        port=args.port,
        image_size=args.image_size,
        timeout_ms=args.timeout,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
