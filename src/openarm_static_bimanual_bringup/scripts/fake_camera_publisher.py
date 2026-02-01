#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Fake Camera Publisher for Mock Hardware Testing

Publishes synthetic images to all 3 camera topics for testing
VLA data collection workflow without real cameras.

Usage:
    ros2 run openarm_static_bimanual_bringup fake_camera_publisher.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class FakeCameraPublisher(Node):
    """Publishes fake camera images for mock testing."""
    
    # Must match lerobot_vla_replay_recorder.py CAMERA_TOPICS
    CAMERA_CONFIG = {
        'cam_1': '/camera/cam_1/color/image_raw',  # top
        'cam_2': '/camera/cam_2/color/image_raw',  # wrist_left
        'cam_3': '/camera/cam_3/color/image_raw',  # wrist_right
    }
    IMAGE_SIZE = (256, 256)
    PUBLISH_RATE = 20.0  # Hz
    
    def __init__(self):
        super().__init__('fake_camera_publisher')
        self.bridge = CvBridge()
        
        # Create publishers for each camera (use _camera_pubs to avoid Node.publishers conflict)
        self._camera_pubs = {}
        for name, topic in self.CAMERA_CONFIG.items():
            self._camera_pubs[name] = self.create_publisher(Image, topic, 10)
            self.get_logger().info(f'Publishing to {topic}')
        
        # Timer for periodic publishing
        period = 1.0 / self.PUBLISH_RATE
        self.timer = self.create_timer(period, self.publish_images)
        self.frame_count = 0
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('  Fake Camera Publisher Started!')
        self.get_logger().info(f'  Publishing at {self.PUBLISH_RATE} Hz')
        self.get_logger().info(f'  Image size: {self.IMAGE_SIZE}')
        self.get_logger().info('=' * 50)
    
    def publish_images(self):
        """Generate and publish synthetic images."""
        h, w = self.IMAGE_SIZE
        
        for name, pub in self._camera_pubs.items():
            # Generate distinct colors for each camera
            color_offset = {'cam_1': 0, 'cam_2': 85, 'cam_3': 170}[name]
            hue = (self.frame_count * 2 + color_offset) % 256
            
            # Create gradient image with camera name indicator
            img = np.zeros((h, w, 3), dtype=np.uint8)
            
            # Background gradient
            for y in range(h):
                img[y, :, 0] = hue  # Blue
                img[y, :, 1] = int(y / h * 128)  # Green gradient
                img[y, :, 2] = (hue + 100) % 256  # Red
            
            # Add frame counter indicator (top-left corner brightness)
            brightness = (self.frame_count % 30) * 8
            img[:20, :20] = brightness
            
            # Convert to ROS message
            msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f'{name}_color_optical_frame'
            pub.publish(msg)
        
        self.frame_count += 1
        
        # Periodic status log
        if self.frame_count % 100 == 0:
            self.get_logger().info(f'Published {self.frame_count} frames')


def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
