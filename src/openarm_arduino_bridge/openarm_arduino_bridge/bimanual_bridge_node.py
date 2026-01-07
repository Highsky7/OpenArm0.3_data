#!/usr/bin/env python3
import time, glob, serial, math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

def find_serial_port():
    pats = [
        "/dev/serial/by-id/*Arduino*",
        "/dev/serial/by-id/*USB*Serial*",
        "/dev/serial/by-id/*ttyACM*",
        "/dev/serial/by-id/*ttyUSB*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]
    for p in pats:
        m = sorted(glob.glob(p))
        if m:
            return m[0]
    return None

class BimanualGripperBridge(Node):
    def __init__(self):
        super().__init__('openarm_bimanual_arduino_bridge')

        # === 파라미터 선언 ===
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200) # 아두이노 스케치와 일치
        self.declare_parameter('timeout_ms', 1000)

        self.declare_parameter('left_cmd_topic', '/left_gripper_cmd')
        self.declare_parameter('right_cmd_topic', '/right_gripper_cmd')

        self.declare_parameter('pub_js_topic', '/gripper_states') # 토픽 이름 변경
        self.declare_parameter('left_joint_name', 'left_gripper_joint')
        self.declare_parameter('right_joint_name', 'right_gripper_joint')

        self.declare_parameter('write_min_period_ms', 50) # 아두이노 코드의 루프 속도와 비슷하게 설정
        self.declare_parameter('publish_rate_hz', 20.0)
        
        self.declare_parameter('rad_to_deg_scale', 60.0) # 라디안을 아두이노가 사용할 값(0-60)으로 변환하는 스케일

        # 부드럽게: EMA + 데드밴드
        self.declare_parameter('use_smoothing', False) # 기본적으로 비활성화
        self.declare_parameter('grip_alpha', 0.25)
        self.declare_parameter('min_delta_rad', 0.01)

        # === 파라미터 가져오기 ===
        self.port_param = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.timeout = float(self.get_parameter('timeout_ms').value) / 1000.0

        self.left_cmd_topic = self.get_parameter('left_cmd_topic').value
        self.right_cmd_topic = self.get_parameter('right_cmd_topic').value
        self.pub_js_topic = self.get_parameter('pub_js_topic').value # 변수 이름 변경
        self.left_jname = self.get_parameter('left_joint_name').value
        self.right_jname = self.get_parameter('right_joint_name').value

        self.write_min_period = float(self.get_parameter('write_min_period_ms').value) / 1000.0
        self.pub_period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.rad_to_deg_scale = float(self.get_parameter('rad_to_deg_scale').value)

        self.use_smoothing = self.get_parameter('use_smoothing').value
        self.alpha = float(self.get_parameter('grip_alpha').value)
        self.min_delta = float(self.get_parameter('min_delta_rad').value)

        # 시리얼 포트
        self.ser = None
        self._connected = False
        self._connect_serial()
        self.create_timer(2.0, self._maybe_reconnect)

        # ROS I/O
        self.sub_cmd_left = self.create_subscription(Float64, self.left_cmd_topic, self.cb_cmd_left, 10)
        self.sub_cmd_right = self.create_subscription(Float64, self.right_cmd_topic, self.cb_cmd_right, 10)
        
        self.pub_js = self.create_publisher(JointState, self.pub_js_topic, 10) # 변경된 토픽 사용
        self.create_timer(self.pub_period, self._tick_publish)

        # 상태
        self._filt_r_left = 0.0
        self._filt_r_right = 0.0
        self._last_sent_time = 0.0
        self._last_sent_val_left = None
        self._last_sent_val_right = None
        self._warned_about_range = False

        self.get_logger().info(
            f'Bimanual Gripper bridge up: port={self.port_param} baud={self.baud} '
            f'joints=["{self.left_jname}", "{self.right_jname}"] '
            f'cmd_topics=["{self.left_cmd_topic}", "{self.right_cmd_topic}"]'
        )

    def _connect_serial(self):
        port = self.port_param
        if port == 'auto':
            port = find_serial_port()
            if not port:
                self._connected = False
                self.get_logger().warn('No serial device found (auto). Retrying...')
                self.ser = None
                return
        try:
            self.ser = serial.Serial(port, self.baud, timeout=self.timeout)
            time.sleep(2.0) # 아두이노 리셋 대기
            self._connected = True
            self.get_logger().info(f'Connected to {port}')
        except Exception as e:
            self._connected = False
            self.get_logger().warn(f'Cannot open {port}: {e}')
            self.ser = None

    def _maybe_reconnect(self):
        if self.ser is None or not self.ser.is_open:
            self._connect_serial()

    def _write_line(self, text: str):
        if not self._connected or self.ser is None:
            return
        try:
            self.ser.write((text.strip() + '\n').encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            self._connected = False
            try: self.ser.close() 
            except Exception: pass
            self.ser = None

    def _process_cmd(self, r: float, is_left: bool):
        # Clamp the input value to the expected range [0.0, 1.0]
        clamped_r = max(0.0, min(1.0, r))
        if clamped_r != r and not self._warned_about_range:
            self.get_logger().warn(
                f'Received command value {r} which is outside the expected range [0.0, 1.0]. '
                f'Value will be clamped to {clamped_r}. This warning will not be shown again.'
            )
            self._warned_about_range = True
        
        if self.use_smoothing:
            if is_left:
                self._filt_r_left = self.alpha * clamped_r + (1.0 - self.alpha) * self._filt_r_left
            else:
                self._filt_r_right = self.alpha * clamped_r + (1.0 - self.alpha) * self._filt_r_right
        else:
            if is_left:
                self._filt_r_left = clamped_r
            else:
                self._filt_r_right = clamped_r
        
        self._try_send_serial()

    def cb_cmd_left(self, msg: Float64):
        self._process_cmd(float(msg.data), is_left=True)

    def cb_cmd_right(self, msg: Float64):
        self._process_cmd(float(msg.data), is_left=False)

    def _try_send_serial(self):
        now = time.time()
        if (now - self._last_sent_time) < self.write_min_period:
            return

        delta_left = abs(self._filt_r_left - (self._last_sent_val_left or 0.0))
        delta_right = abs(self._filt_r_right - (self._last_sent_val_right or 0.0))

        if delta_left >= self.min_delta or delta_right >= self.min_delta or self._last_sent_val_left is None:
            # 라디안 값을 아두이노가 사용할 정수 값으로 변환 (0-60 범위)
            val_a = int(self._filt_r_left * self.rad_to_deg_scale)
            val_b = int(self._filt_r_right * self.rad_to_deg_scale)
            
            self._write_line(f'{val_a},{val_b}')
            
            self._last_sent_time = now
            self._last_sent_val_left = self._filt_r_left
            self._last_sent_val_right = self._filt_r_right

    def _tick_publish(self):
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.left_jname, self.right_jname]
        out.position = [self._filt_r_left, self._filt_r_right]
        self.pub_js.publish(out)

def main():
    rclpy.init()
    node = BimanualGripperBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
