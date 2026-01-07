# 파일 이름: synchronized_collector.py
# 역할: 카메라(MP4)와 엔코더(NPY)를 동기화하여 에피소드별로 저장

import rclpy
from rclpy.node import Node
import message_filters
import numpy as np
import cv2
from cv_bridge import CvBridge

# 메시지 타입 import
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState

import os
import time
from pathlib import Path

class FinalDataCollector(Node):
    def __init__(self):
        super().__init__('final_data_collector_node')

        # --- 설정 ---
        BASE_DATASET_DIR = Path(os.path.expanduser('~')) / 'my_robot_dataset'
        self.TARGET_FPS = 10
        # --- 설정 끝 ---

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_sync_time = 0
        self.frame_duration = 1.0 / self.TARGET_FPS

        # 에피소드별로 '이미지 프레임'을 임시 저장할 리스트
        self.episode_frames = []

        # 다음 에피소드 번호 자동 찾기
        episode_index = 0
        while (BASE_DATASET_DIR / f"episode_{episode_index}").exists():
            episode_index += 1
        
        self.episode_dir = BASE_DATASET_DIR / f"episode_{episode_index}"
        self.episode_index = episode_index
        
        # 엔코더 .npy 파일을 저장할 폴더 생성
        self.encoder_dir = self.episode_dir / "encoder"

        try:
            self.encoder_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"이번 에피소드 데이터는 '{self.episode_dir}' 폴더에 저장됩니다.")
        except OSError as e:
            self.get_logger().error(f"데이터 저장 폴더 생성 실패: {e}")
            rclpy.shutdown()
            return

        # Subscriber 및 동기화 설정
        image_sub = message_filters.Subscriber(self, Image, '/camera/camera/color/image_raw')
        joint_state_sub = message_filters.Subscriber(self, JointState, '/joint_states')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, joint_state_sub], queue_size=30, slop=0.05)
        
        self.ts.registerCallback(self.sync_callback)
        self.get_logger().info(f"Episode #{episode_index} 데이터 수집을 시작합니다. (10Hz)")

    def sync_callback(self, image_msg, joint_state_msg):
        current_time = time.time()
        if current_time - self.last_sync_time < self.frame_duration:
            return
        self.last_sync_time = current_time

        try:
            # 1. 이미지는 리스트에 임시 저장 (나중에 MP4로 만들기 위해)
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            self.episode_frames.append(cv_image)

            # 2. 엔코더 데이터는 즉시 .npy 파일로 저장
            encoder_data = np.array(joint_state_msg.position, dtype=np.float32)
            self.frame_count += 1
            base_filename = f"{self.frame_count:06d}"
            encoder_filepath = self.encoder_dir / f"{base_filename}_encoder.npy"
            np.save(encoder_filepath, encoder_data)

            self.get_logger().info(f"수집 중: Frame #{self.frame_count}", throttle_duration_sec=1.0)

        except Exception as e:
            self.get_logger().error(f"데이터 처리/저장 중 에러 발생: {e}")

    def save_video_on_shutdown(self):
        # 3. 노드 종료 시, 모아둔 이미지 프레임으로 MP4 비디오 파일 생성
        if not self.episode_frames:
            self.get_logger().warn("수집된 이미지 프레임이 없어 비디오를 저장하지 않습니다.")
            return

        self.get_logger().info(f"수집된 {len(self.episode_frames)}개의 프레임으로 MP4 비디오 저장을 시작합니다...")
        
        video_filename = f"episode_{self.episode_index}_rgb.mp4"
        video_path = self.episode_dir / video_filename
        
        height, width, _ = self.episode_frames[0].shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(str(video_path), fourcc, self.TARGET_FPS, (width, height))
        
        for frame in self.episode_frames:
            video_writer.write(frame)
            
        video_writer.release()
        self.get_logger().info(f"비디오 저장 완료: {video_path}")


def main(args=None):
    rclpy.init(args=args)
    node = FinalDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('데이터 수집을 중지합니다. 최종 파일 저장을 시작합니다.')
        node.save_video_on_shutdown() # 종료 시 비디오 저장 함수 호출
        node.destroy_node()
        if rclpy.ok():
            time.sleep(1) # 파일 쓰기 완료를 위한 대기
            rclpy.shutdown()

if __name__ == '__main__':
    main()