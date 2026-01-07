import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
import serial
import time

class TeleopMagnetometerNode(Node):
    def __init__(self):
        super().__init__('teleop_magnetometer_node')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('left_arm_controller_topic', '/left_forward_position_controller/commands')
        self.declare_parameter('right_arm_controller_topic', '/right_forward_position_controller/commands')
        self.declare_parameter('left_gripper_controller_topic', '/left_gripper_controller/commands')
        self.declare_parameter('right_gripper_controller_topic', '/right_gripper_controller/commands')

        # Get parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.left_arm_controller_topic = self.get_parameter('left_arm_controller_topic').get_parameter_value().string_value
        self.right_arm_controller_topic = self.get_parameter('right_arm_controller_topic').get_parameter_value().string_value
        self.left_gripper_controller_topic = self.get_parameter('left_gripper_controller_topic').get_parameter_value().string_value
        self.right_gripper_controller_topic = self.get_parameter('right_gripper_controller_topic').get_parameter_value().string_value

        # Publishers
        self.left_arm_publisher = self.create_publisher(Float64MultiArray, self.left_arm_controller_topic, 10)
        self.right_arm_publisher = self.create_publisher(Float64MultiArray, self.right_arm_controller_topic, 10)
        self.left_gripper_publisher = self.create_publisher(Float64, self.left_gripper_controller_topic, 10)
        self.right_gripper_publisher = self.create_publisher(Float64, self.right_gripper_controller_topic, 10)

        # Serial connection
        try:
            self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Successfully connected to serial port {self.port} at {self.baud_rate} baud.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to serial port {self.port}: {e}')
            self.serial_port = None
            # rclpy.shutdown() # Don't shutdown, just log error and continue trying or exit gracefully

        # Timer for reading serial data
        self.timer = self.create_timer(0.01, self.read_serial_data) # 100 Hz

        self.get_logger().info('TeleopMagnetometerNode has been started.')

    def read_serial_data(self):
        if self.serial_port is None:
            return

        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith('@,'):
                    data_str = line[2:].split(',')
                    if len(data_str) >= 18: # r0-r15 (16 values) + s1 + s2 (2 values) = 18
                        try:
                            data_values = [float(x) for x in data_str]

                            # Arm joint positions (r0-r13)
                            left_arm_positions = data_values[0:7]
                            right_arm_positions = data_values[7:14]

                            # Gripper commands (s1, s2)
                            left_gripper_command = data_values[16] # s1
                            right_gripper_command = data_values[17] # s2

                            # Publish arm commands
                            left_arm_msg = Float64MultiArray()
                            left_arm_msg.data = left_arm_positions
                            self.left_arm_publisher.publish(left_arm_msg)

                            right_arm_msg = Float64MultiArray()
                            right_arm_msg.data = right_arm_positions
                            self.right_arm_publisher.publish(right_arm_msg)

                            # Publish gripper commands
                            left_gripper_msg = Float64()
                            left_gripper_msg.data = left_gripper_command
                            self.left_gripper_publisher.publish(left_gripper_msg)

                            right_gripper_msg = Float64()
                            right_gripper_msg.data = right_gripper_command
                            self.right_gripper_publisher.publish(right_gripper_msg)

                        except ValueError as ve:
                            self.get_logger().warn(f'Error parsing serial data: {ve} in line: {line}')
                    else:
                        self.get_logger().warn(f'Received incomplete data: {line}')
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {e}')
            self.serial_port.close()
            self.serial_port = None
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMagnetometerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port and node.serial_port.is_open:
            node.serial_port.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
