#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class JSPFPCBridge(Node):
    def __init__(self):
        super().__init__("jsp_fpc_bridge")

        # 구독: GUI가 퍼블리시한 joint_states
        self.sub = self.create_subscription(
            JointState, "/fake_joint_states", self.cb, 10)

        # 퍼블리셔: FPC 명령
        self.pub_left = self.create_publisher(
            Float64MultiArray, "/left_forward_position_controller/commands", 10)
        self.pub_right = self.create_publisher(
            Float64MultiArray, "/right_forward_position_controller/commands", 10)

        # 왼팔/오른팔 조인트 인덱스 정의 (URDF 순서 기반)
        self.left_names = [f"left_rev{i}" for i in range(1, 8)]
        self.right_names = [f"right_rev{i}" for i in range(1, 8)]

    def cb(self, msg: JointState):
        name2pos = dict(zip(msg.name, msg.position))

        # 왼쪽
        left_cmd = Float64MultiArray()
        left_cmd.data = [name2pos.get(n, 0.0) for n in self.left_names]
        self.pub_left.publish(left_cmd)

        # 오른쪽
        right_cmd = Float64MultiArray()
        right_cmd.data = [name2pos.get(n, 0.0) for n in self.right_names]
        self.pub_right.publish(right_cmd)

def main():
    rclpy.init()
    node = JSPFPCBridge()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
