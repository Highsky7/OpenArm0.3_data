#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLA Remote Client Node - 로봇 laptop에서 실행

ROS2 노드로서 카메라 이미지와 조인트 상태를 수집하고,
SSH 터널을 통해 원격 서버의 VLA 추론 서버로 전송합니다.
응답으로 받은 action을 로봇에 적용합니다.

주의: initial_move 로직은 포함되지 않습니다.
      lerobot_trajectory_recording.launch.py에서 이미 초기화된 상태를 가정합니다.

Author: Antigravity Assistant
Date: 2026-02-09
"""

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
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray


class VLARemoteClientNode(Node):
    """VLA 원격 추론 클라이언트 노드"""
    
    # 카메라 토픽 설정
    # Keys chosen to match server-side expected image feature names
    CAMERA_TOPICS = {
        'camera1': '/camera/cam_1/color/image_raw/compressed',
        'camera2': '/camera/cam_2/color/image_raw/compressed',
        'camera3': '/camera/cam_3/color/image_raw/compressed',
    }
    
    # 16-DOF 조인트 이름 (left 8 + right 8)
    STATE_JOINT_NAMES = [
        'left_rev1', 'left_rev2', 'left_rev3', 'left_rev4',
        'left_rev5', 'left_rev6', 'left_rev7', 'left_rev8',  # left_rev8 = left gripper
        'right_rev1', 'right_rev2', 'right_rev3', 'right_rev4',
        'right_rev5', 'right_rev6', 'right_rev7', 'right_rev8',  # right_rev8 = right gripper
    ]
    
    # 제어 토픽
    LEFT_ARM_TOPIC = '/left_arm_controller/commands'
    RIGHT_ARM_TOPIC = '/right_arm_controller/commands'
    LEFT_GRIPPER_TOPIC = '/left_gripper_controller/commands'
    RIGHT_GRIPPER_TOPIC = '/right_gripper_controller/commands'
    
    def __init__(self):
        super().__init__('vla_remote_client')
        
        # 파라미터 선언
        self._declare_parameters()
        
        # 파라미터 값 로드
        self.server_port = self.get_parameter('server_port').value
        self.inference_rate = self.get_parameter('inference_rate').value
        self.enable_control = self.get_parameter('enable_control').value
        self.task_description = self.get_parameter('task_description').value
        self.debug = self.get_parameter('debug').value
        self.image_size = self.get_parameter('image_size').value
        self.timeout_ms = self.get_parameter('timeout_ms').value
        
        # ZeroMQ 클라이언트 설정
        self._setup_zmq()
        
        # 상태 저장 변수
        self.image_lock = threading.Lock()
        self.latest_images: Dict[str, Optional[np.ndarray]] = {k: None for k in self.CAMERA_TOPICS}
        self.last_joint_state: Optional[JointState] = None
        self.joint_name_to_index: Dict[str, int] = {}
        
        # 통계
        self.request_count = 0
        self.success_count = 0
        self.total_rtt = 0.0
        
        # ROS2 설정
        self._setup_subscribers()
        self._setup_publishers()
        
        # 추론 타이머
        timer_period = 1.0 / self.inference_rate
        self.inference_timer = self.create_timer(timer_period, self.inference_callback)
        
        self._print_banner()
    
    def _declare_parameters(self):
        """ROS2 파라미터 선언"""
        self.declare_parameter('server_port', 5555)
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('enable_control', False)  # 기본값 False (안전)
        self.declare_parameter('task_description', 'manipulation task')
        self.declare_parameter('debug', True)
        self.declare_parameter('image_size', 256)
        self.declare_parameter('timeout_ms', 5000)
    
    def _setup_zmq(self):
        """ZeroMQ REQ 소켓 설정"""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.REQ)
        self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)  # 수신 타임아웃
        self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)  # 송신 타임아웃
        self.socket.connect(f"tcp://localhost:{self.server_port}")
        self.get_logger().info(f'🔌 ZeroMQ 연결: tcp://localhost:{self.server_port}')
    
    def _setup_subscribers(self):
        """ROS2 구독자 설정"""
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 카메라 구독
        self.camera_subs = {}
        for key, topic in self.CAMERA_TOPICS.items():
            self.camera_subs[key] = self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, k=key: self._camera_callback(msg, k),
                qos
            )
        
        # 조인트 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
    
    def _setup_publishers(self):
        """ROS2 퍼블리셔 설정"""
        # 팔 제어
        self.left_arm_pub = self.create_publisher(
            Float64MultiArray, self.LEFT_ARM_TOPIC, 10)
        self.right_arm_pub = self.create_publisher(
            Float64MultiArray, self.RIGHT_ARM_TOPIC, 10)
        
        # 그리퍼 제어
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, self.LEFT_GRIPPER_TOPIC, 10)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, self.RIGHT_GRIPPER_TOPIC, 10)
    
    def _print_banner(self):
        """시작 배너 출력"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('  🤖 VLA Remote Client Node')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  📡 서버: localhost:{self.server_port} (SSH 터널)')
        self.get_logger().info(f'  ⏱️  추론 주기: {self.inference_rate} Hz')
        self.get_logger().info(f'  📝 태스크: {self.task_description}')
        self.get_logger().info(f'  🎮 제어 활성화: {self.enable_control}')
        self.get_logger().info(f'  🔍 디버그 모드: {self.debug}')
        
        if not self.enable_control:
            self.get_logger().warn('⚠️ 제어 비활성화됨 - Dry-run 모드')
            self.get_logger().warn('   실제 로봇 제어를 원하면 enable_control:=true 설정')
        
        self.get_logger().info('=' * 60)
    
    def _camera_callback(self, msg: CompressedImage, camera_key: str):
        """카메라 이미지 콜백"""
        try:
            # 압축 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if image is not None:
                with self.image_lock:
                    self.latest_images[camera_key] = image
        except Exception as e:
            self.get_logger().error(f'카메라 {camera_key} 처리 오류: {e}')
    
    def _joint_state_callback(self, msg: JointState):
        """조인트 상태 콜백"""
        self.last_joint_state = msg
        
        # 조인트 이름 → 인덱스 매핑 (최초 1회)
        if not self.joint_name_to_index:
            for i, name in enumerate(msg.name):
                self.joint_name_to_index[name] = i
    
    def _get_joint_positions(self) -> Optional[np.ndarray]:
        """16-dim 조인트 위치 벡터 추출"""
        if self.last_joint_state is None:
            return None
        
        positions = []
        for name in self.STATE_JOINT_NAMES:
            if name in self.joint_name_to_index:
                idx = self.joint_name_to_index[name]
                positions.append(self.last_joint_state.position[idx])
            else:
                positions.append(0.0)
        
        return np.array(positions, dtype=np.float32)
    
    def _prepare_images(self) -> Optional[Dict[str, bytes]]:
        """이미지 전처리 및 바이트 변환"""
        images_bytes = {}
        
        with self.image_lock:
            for key, image in self.latest_images.items():
                if image is None:
                    return None
                
                # 리사이즈
                resized = cv2.resize(image, (self.image_size, self.image_size))
                # RGB 변환 (OpenCV는 BGR)
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                # 바이트 변환
                images_bytes[key] = rgb.tobytes()
        
        return images_bytes
    
    def inference_callback(self):
        """추론 요청 및 응답 처리 (타이머 콜백)"""
        # 데이터 준비
        images_bytes = self._prepare_images()
        state = self._get_joint_positions()
        
        if images_bytes is None or state is None:
            if self.debug:
                missing = []
                with self.image_lock:
                    for k, v in self.latest_images.items():
                        if v is None:
                            missing.append(k)
                if missing:
                    self.get_logger().debug(f'대기 중: 이미지 누락 {missing}')
                if state is None:
                    self.get_logger().debug('대기 중: 조인트 상태 없음')
            return
        
        try:
            # 요청 데이터 구성: 서버가 기대하는 키 구조로 전송
            # 서버는 'observation.state' 및 'observation.images.<camera>' 키를 기대합니다.
            request = {
                'observation.state': state.tolist(),
                'task': self.task_description,
            }
            for k, v in images_bytes.items():
                request[f'observation.images.{k}'] = v
            
            # 전송
            start_time = time.time()
            self.socket.send(msgpack.packb(request))
            
            if self.debug:
                self.get_logger().info(
                    f'📤 전송: images=3x{self.image_size}x{self.image_size}, '
                    f'state={state.shape}, task="{self.task_description[:30]}..."'
                )
            
            # 응답 수신
            response = msgpack.unpackb(self.socket.recv(), raw=False)
            rtt = (time.time() - start_time) * 1000
            
            self.request_count += 1
            self.total_rtt += rtt
            
            if response.get('status') == 'ok':
                self.success_count += 1
                action = np.array(response['action'], dtype=np.float32)
                server_time = response.get('inference_time_ms', 0)
                
                if self.debug:
                    self.get_logger().info(
                        f'📥 수신: action={action[:4]}... '
                        f'(RTT: {rtt:.1f}ms, 서버: {server_time:.1f}ms)'
                    )
                
                # 로봇 제어 (enable_control=True일 때만)
                if self.enable_control:
                    self._execute_action(action)
                
                # 주기적 통계 출력
                if self.request_count % 10 == 0:
                    avg_rtt = self.total_rtt / self.request_count
                    success_rate = self.success_count / self.request_count * 100
                    self.get_logger().info(
                        f'📊 통계: {self.success_count}/{self.request_count} 성공 '
                        f'({success_rate:.0f}%), 평균 RTT: {avg_rtt:.1f}ms'
                    )
            else:
                error_msg = response.get('message', 'Unknown error')
                self.get_logger().error(f'❌ 서버 오류: {error_msg}')
                
        except zmq.Again:
            self.get_logger().warn('⏰ 응답 타임아웃 - 서버 연결 확인 필요')
            self._reconnect_zmq()
            
        except Exception as e:
            self.get_logger().error(f'❌ 요청 실패: {e}')
    
    def _reconnect_zmq(self):
        """ZeroMQ 재연결"""
        try:
            self.socket.close()
            self.socket = self.context.socket(zmq.REQ)
            self.socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
            self.socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
            self.socket.connect(f"tcp://localhost:{self.server_port}")
            self.get_logger().info('🔄 ZeroMQ 재연결 완료')
        except Exception as e:
            self.get_logger().error(f'❌ ZeroMQ 재연결 실패: {e}')
    
    def _execute_action(self, action: np.ndarray):
        """
        로봇에 액션 명령 전송
        
        action: 16-dim numpy array
            [0:7]  - left arm (rev1~rev7)
            [7]    - left gripper (rev8)
            [8:15] - right arm (rev1~rev7)
            [15]   - right gripper (rev8)
        """
        # Left arm (indices 0-6, 7개 조인트)
        left_arm_msg = Float64MultiArray()
        left_arm_msg.data = [float(p) for p in action[0:7]]
        self.left_arm_pub.publish(left_arm_msg)
        
        # Right arm (indices 8-14, 7개 조인트)
        right_arm_msg = Float64MultiArray()
        right_arm_msg.data = [float(p) for p in action[8:15]]
        self.right_arm_pub.publish(right_arm_msg)
        
        # Left gripper (index 7)
        left_gripper_msg = Float64MultiArray()
        left_gripper_msg.data = [float(action[7])]
        self.left_gripper_pub.publish(left_gripper_msg)
        
        # Right gripper (index 15)
        right_gripper_msg = Float64MultiArray()
        right_gripper_msg.data = [float(action[15])]
        self.right_gripper_pub.publish(right_gripper_msg)
        
        if self.debug:
            self.get_logger().debug(
                f'🦾 액션 적용: L_arm={action[0:3]}..., R_arm={action[8:11]}..., '
                f'L_grip={action[7]:.2f}, R_grip={action[15]:.2f}'
            )
    
    def destroy_node(self):
        """노드 종료 시 정리"""
        self.get_logger().info('🧹 VLA Remote Client 종료 중...')
        self.socket.close()
        self.context.term()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = VLARemoteClientNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
