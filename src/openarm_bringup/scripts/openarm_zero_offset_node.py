# save as openarm_zero_offset_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

OFFSETS = {
    "rev1": -0.0,
    "rev2": +1.5218966964,
    "rev3": +2.8845273518,
    "rev4": -0.2183947509,
    "rev5": -0.1375219348,
    "rev6": -1.5959029526,
    "rev7": -0.0177386130,
    "rev8": +0.0192645151,
}

class ZeroOffset(Node):
    def __init__(self):
        super().__init__("zero_offset")
        self.sub = self.create_subscription(JointState, "/joint_states", self.cb, 10)
        self.pub = self.create_publisher(JointState, "/joint_states/zeroed", 10)

    def cb(self, msg):
        out = JointState()
        out.header = msg.header
        out.name = msg.name
        out.position = list(msg.position)
        out.velocity = msg.velocity
        out.effort = msg.effort
        name_to_idx = {n:i for i,n in enumerate(msg.name)}
        for jn, off in OFFSETS.items():
            if jn in name_to_idx:
                i = name_to_idx[jn]
                out.position[i] = msg.position[i] + off
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(ZeroOffset())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
