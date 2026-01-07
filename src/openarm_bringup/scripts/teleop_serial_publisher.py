#!/usr/bin/python3
import rclpy, time, serial
from rclpy.node import Node
from sensor_msgs.msg import JointState

class TeleopSerialPublisher(Node):
    def __init__(self):
        super().__init__('teleop_serial_publisher')
        port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 230400).get_parameter_value().integer_value
        self.names = self.declare_parameter('names', [
            'tele0','tele1','tele2','tele3','tele4','tele5','tele6','tele7'
        ]).get_parameter_value().string_array_value
        self.n = len(self.names)

        self.get_logger().info(f'Opening {port} @ {baud}')
        self.ser = serial.Serial(port, baud, timeout=0.01)
        time.sleep(0.2)

        self.pub = self.create_publisher(JointState, '/teleop/joint_states', 10)
        self.create_timer(1.0/200.0, self.tick)  # 200 Hz

    def tick(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line.startswith('@,'):
                return
            parts = line.split(',')[1:]
            if len(parts) < self.n:
                return
            vals = [float(p) for p in parts[:self.n]]
        except Exception:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.names)
        msg.position = vals
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TeleopSerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
