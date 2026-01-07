#!/usr/bin/env python3
import time, glob, serial
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

class GripperBridge(Node):
    def __init__(self):
        super().__init__('openarm_arduino_bridge')

        # === 편한 기본값 ===
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('timeout_ms', 1000)

        self.declare_parameter('cmd_topic', '/gripper_cmd')   # 라디안 입력
        self.declare_parameter('compat_cmd_topic', '')        # 기본 비활성

        self.declare_parameter('js_topic', '/joint_states')
        self.declare_parameter('joint_name', 'left_pris1')

        # 20Hz 송신(아두이노 DUR_MS=50ms에 맞춤)
        self.declare_parameter('write_min_period_ms', 10)
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('inject_when_absent', True)
        self.declare_parameter('absent_timeout_ms', 300)
        self.declare_parameter('default_when_off', 0.0)

        # 부드럽게: EMA + 데드밴드
        self.declare_parameter('grip_alpha', 0.25)       # 0.2~0.5 권장
        self.declare_parameter('min_delta_rad', 0.01)    # ≈0.57°

        self.port_param = self.get_parameter('port').value
        self.baud    = int(self.get_parameter('baud').value)
        self.timeout = float(self.get_parameter('timeout_ms').value)/1000.0

        self.cmd_topic  = self.get_parameter('cmd_topic').value
        self.compat_cmd = self.get_parameter('compat_cmd_topic').value
        self.js_topic   = self.get_parameter('js_topic').value
        self.jname      = self.get_parameter('joint_name').value

        self.write_min_period = float(self.get_parameter('write_min_period_ms').value)/1000.0
        self.pub_period       = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.inject_when_absent = bool(self.get_parameter('inject_when_absent').value)
        self.absent_timeout   = float(self.get_parameter('absent_timeout_ms').value)/1000.0
        self.default_when_off = float(self.get_parameter('default_when_off').value)

        self.alpha      = float(self.get_parameter('grip_alpha').value)
        self.min_delta  = float(self.get_parameter('min_delta_rad').value)

        # Serial
        self.ser = None
        self._connected = False
        self._connect_serial()
        self.create_timer(2.0, self._maybe_reconnect)

        # ROS I/O
        self.sub_cmd = self.create_subscription(Float64, self.cmd_topic, self.cb_cmd, 10)
        if self.compat_cmd and self.compat_cmd != self.cmd_topic:
            self.sub_cmd_compat = self.create_subscription(Float64, self.compat_cmd, self.cb_cmd, 10)

        self.sub_probe = self.create_subscription(JointState, self.js_topic, self.cb_probe_js, 10)
        self.pub_js = self.create_publisher(JointState, self.js_topic, 10)
        self.create_timer(self.pub_period, self._tick_publish)

        # 상태
        self._last_cmd_r = 0.0           # 최근 원시 명령(rad)
        self._filt_r     = None          # EMA 결과(rad)
        self._last_sent_time = 0.0
        self._last_sent_val  = None
        self._last_seen_left_ts = 0.0

        topics = [self.cmd_topic] + ([self.compat_cmd] if self.compat_cmd else [])
        self.get_logger().info(
            f'Gripper bridge up: port={self.port_param} baud={self.baud} joint="{self.jname}" '
            f'cmd_topics={topics} inject_when_absent={self.inject_when_absent}'
        )

    # ───────── Serial ─────────
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
            time.sleep(2.0)
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
        if self.ser is None or not self.ser.is_open:
            self._connected = False
            return
        try:
            self.ser.write((text.strip() + '\n').encode('utf-8'))
            self._connected = True
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')
            try: self.ser.close()
            except Exception: pass
            self._connected = False
            self.ser = None

    # ───────── Callbacks ─────────
    def cb_cmd(self, msg: Float64):
        r = float(msg.data)
        self._last_cmd_r = r
        # EMA
        self._filt_r = r if self._filt_r is None else (self.alpha*r + (1.0-self.alpha)*self._filt_r)
        # 데드밴드 + 주기 제한(20Hz)
        now = time.time()
        sendable = (self._last_sent_val is None) or (abs(self._filt_r - self._last_sent_val) >= self.min_delta)
        if sendable and (now - self._last_sent_time) >= self.write_min_period:
            self._write_line(f'{self._filt_r:.6f}')
            self._last_sent_time = now
            self._last_sent_val  = self._filt_r

    def cb_probe_js(self, msg: JointState):
        try:
            idx = msg.name.index(self.jname)
        except ValueError:
            return
        if idx < len(msg.position):
            self._last_seen_left_ts = time.time()

    # RViz용 상태 퍼블리시(필요할 때만)
    def _tick_publish(self):
        if self.inject_when_absent and (time.time() - self._last_seen_left_ts) < self.absent_timeout:
            return
        r_for_vis = self._filt_r if self._filt_r is not None else (self._last_cmd_r if self._connected else self.default_when_off)
        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = [self.jname]
        out.position = [float(r_for_vis)]
        self.pub_js.publish(out)

def main():
    rclpy.init()
    node = GripperBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
