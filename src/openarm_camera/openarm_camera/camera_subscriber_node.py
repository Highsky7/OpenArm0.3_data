import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os # os 모듈 추가

class CameraSubscriberNode(Node):
    def __init__(self):
        super().__init__('camera_subscriber_node')
        
        # 변경점 1: '~'를 사용자의 실제 홈 디렉토리 경로로 명확하게 변경
        default_path = os.path.join(os.path.expanduser('~'), 'output_video.mp4')
        
        self.declare_parameter('output_path', default_path)
        self.declare_parameter('fps', 10.0)
        
        output_path = self.get_parameter('output_path').get_parameter_value().string_value
        fps = self.get_parameter('fps').get_parameter_value().double_value
        
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw', 
            self.image_callback,
            10)
        
        self.video_writer = None
        self.output_path = output_path
        self.fps = fps
        self.get_logger().info(f"카메라 토픽을 구독합니다. 영상은 '{output_path}'에 저장됩니다.")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CvBridge 변환 실패: {e}')
            return

        if self.video_writer is None:
            height, width, _ = cv_image.shape
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(self.output_path, fourcc, self.fps, (width, height))
            if not self.video_writer.isOpened():
                self.get_logger().error("!!! VideoWriter 초기화 실패. 코덱이나 경로를 확인하세요. !!!")
                return
            self.get_logger().info(f"첫 프레임 수신! 동영상 녹화를 시작합니다. (해상도: {width}x{height})")

        self.video_writer.write(cv_image)

    def cleanup(self):
        # 노드가 종료될 때 비디오 파일을 안전하게 닫고 저장
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(f"동영상 파일 저장 완료: {self.output_path}")

# 변경점 2: 종료 시 발생하는 rclpy 에러를 방지하기 위해 main 함수 구조를 정리
def main(args=None):
    rclpy.init(args=args)
    camera_subscriber_node = CameraSubscriberNode()
    try:
        rclpy.spin(camera_subscriber_node)
    except KeyboardInterrupt:
        camera_subscriber_node.get_logger().info('사용자에 의해 노드가 중지되었습니다.')
    finally:
        camera_subscriber_node.cleanup()
        camera_subscriber_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()