#!/usr/bin/env python3

import math
import re
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers, SwitchController

try:
    import serial
except ImportError:  # pragma: no cover
    serial = None

try:
    from urdf_parser_py.urdf import URDF
except ImportError:  # pragma: no cover
    URDF = None


LINE_REGEX = re.compile(r"TELEOP\s+left:\[(.*?)\]\s+right:\[(.*?)\]")
LEGACY_REGEX = re.compile(r"left arm\s*:\s*\[.*?\]\s*=\s*\[(.*?)\]\s+right arm\s*:\s*\[.*?\]\s*=\s*\[(.*?)\]")


class TeleopSerialBridge(Node):
    def __init__(self) -> None:
        super().__init__("teleop_serial_bridge")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud", 500000)
        self.declare_parameter("serial_timeout_sec", 0.05)
        self.declare_parameter("input_unit", "centi_deg")  # centi_deg | deg | rad
        self.declare_parameter("control_rate_hz", 150.0)

        self.declare_parameter("left_joint_names", [
            "left_rev1", "left_rev2", "left_rev3", "left_rev4",
            "left_rev5", "left_rev6", "left_rev7",
        ])
        self.declare_parameter("right_joint_names", [
            "right_rev1", "right_rev2", "right_rev3", "right_rev4",
            "right_rev5", "right_rev6", "right_rev7",
        ])
        self.declare_parameter("left_joint_indices", [0, 1, 2, 3, 4, 5, 6])
        self.declare_parameter("right_joint_indices", [0, 1, 2, 3, 4, 5, 6])
        self.declare_parameter("enable_grippers", True)
        self.declare_parameter("left_gripper_joint_name", "left_rev8")
        self.declare_parameter("right_gripper_joint_name", "right_rev8")
        self.declare_parameter("left_gripper_index", 7)
        self.declare_parameter("right_gripper_index", 7)
        self.declare_parameter("flat_left_joint_indices", [0, 1, 2, 3, 4, 5, 6])
        self.declare_parameter("flat_right_joint_indices", [7, 8, 9, 10, 11, 12, 13])
        self.declare_parameter("flat_left_gripper_index", 16)
        self.declare_parameter("flat_right_gripper_index", 17)
        self.declare_parameter("gripper_clip_to_urdf", False)
        self.declare_parameter("gripper_unit", "deg")  # deg | rad | input
        self.declare_parameter("min_gripper", -0.05)
        self.declare_parameter("max_gripper", 0.9)

        self.declare_parameter("warmup_sec", 3.0)
        self.declare_parameter("startup_ramp_sec", 5.0)
        self.declare_parameter("auto_switch_controllers", False)
        self.declare_parameter("switch_retry_period_sec", 2.0)
        self.declare_parameter("enable_sample_step_filter", True)
        self.declare_parameter("max_sample_step_rad", 0.35)
        self.declare_parameter("max_gripper_sample_step_rad", 0.35)

        self.declare_parameter("robot_description_node", "robot_state_publisher")
        self.declare_parameter("robot_description_param", "robot_description")

        self.declare_parameter("left_jtc_controller", "left_joint_trajectory_controller")
        self.declare_parameter("right_jtc_controller", "right_joint_trajectory_controller")
        self.declare_parameter("left_fpc_controller", "left_forward_position_controller")
        self.declare_parameter("right_fpc_controller", "right_forward_position_controller")
        self.declare_parameter("left_teleop_stream_controller", "left_teleop_stream_controller")
        self.declare_parameter("right_teleop_stream_controller", "right_teleop_stream_controller")

        self.serial_port = self.get_parameter("serial_port").get_parameter_value().string_value
        self.serial_baud = int(self.get_parameter("serial_baud").value)
        self.serial_timeout_sec = float(self.get_parameter("serial_timeout_sec").value)
        self.input_unit = self.get_parameter("input_unit").get_parameter_value().string_value
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.left_joint_names = list(self.get_parameter("left_joint_names").value)
        self.right_joint_names = list(self.get_parameter("right_joint_names").value)
        self.left_joint_indices = list(self.get_parameter("left_joint_indices").value)
        self.right_joint_indices = list(self.get_parameter("right_joint_indices").value)
        self.enable_grippers = bool(self.get_parameter("enable_grippers").value)
        self.left_gripper_joint_name = self.get_parameter("left_gripper_joint_name").get_parameter_value().string_value
        self.right_gripper_joint_name = self.get_parameter("right_gripper_joint_name").get_parameter_value().string_value
        self.left_gripper_index = int(self.get_parameter("left_gripper_index").value)
        self.right_gripper_index = int(self.get_parameter("right_gripper_index").value)
        self.flat_left_joint_indices = list(self.get_parameter("flat_left_joint_indices").value)
        self.flat_right_joint_indices = list(self.get_parameter("flat_right_joint_indices").value)
        self.flat_left_gripper_index = int(self.get_parameter("flat_left_gripper_index").value)
        self.flat_right_gripper_index = int(self.get_parameter("flat_right_gripper_index").value)
        self.gripper_clip_to_urdf = bool(self.get_parameter("gripper_clip_to_urdf").value)
        self.gripper_unit = self.get_parameter("gripper_unit").get_parameter_value().string_value
        self.min_gripper = float(self.get_parameter("min_gripper").value)
        self.max_gripper = float(self.get_parameter("max_gripper").value)

        self.warmup_sec = float(self.get_parameter("warmup_sec").value)
        self.startup_ramp_sec = float(self.get_parameter("startup_ramp_sec").value)
        self.trajectory_duration_sec = max(0.1, self.startup_ramp_sec)
        self.auto_switch_controllers = bool(self.get_parameter("auto_switch_controllers").value)
        self.switch_retry_period_sec = float(self.get_parameter("switch_retry_period_sec").value)
        self.enable_sample_step_filter = bool(self.get_parameter("enable_sample_step_filter").value)
        self.max_sample_step_rad = float(self.get_parameter("max_sample_step_rad").value)
        self.max_gripper_sample_step_rad = float(self.get_parameter("max_gripper_sample_step_rad").value)

        self.robot_description_node = self.get_parameter("robot_description_node").get_parameter_value().string_value
        self.robot_description_param = self.get_parameter("robot_description_param").get_parameter_value().string_value

        self.left_jtc_controller = self.get_parameter("left_jtc_controller").get_parameter_value().string_value
        self.right_jtc_controller = self.get_parameter("right_jtc_controller").get_parameter_value().string_value
        self.left_fpc_controller = self.get_parameter("left_fpc_controller").get_parameter_value().string_value
        self.right_fpc_controller = self.get_parameter("right_fpc_controller").get_parameter_value().string_value
        self.left_teleop_stream_controller = self.get_parameter("left_teleop_stream_controller").get_parameter_value().string_value
        self.right_teleop_stream_controller = self.get_parameter("right_teleop_stream_controller").get_parameter_value().string_value

        self._limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
        self._joint_state_map: Dict[str, float] = {}

        self._latest_left: List[float] = [0.0] * len(self.left_joint_names)
        self._latest_right: List[float] = [0.0] * len(self.right_joint_names)
        self._latest_left_gripper = 0.0
        self._latest_right_gripper = 0.0
        self._have_any_sample = False

        self._lock = threading.Lock()
        self._first_data_time: Optional[float] = None
        self._sent_jtc = False
        self._switched_to_fpc = False
        self._ramp_start_time: Optional[float] = None
        self._ramp_start_left: Optional[List[float]] = None
        self._ramp_start_right: Optional[List[float]] = None
        self._ramp_start_left_gripper: Optional[float] = None
        self._ramp_start_right_gripper: Optional[float] = None
        self._last_parse_time: Optional[float] = None
        self._last_publish_time: Optional[float] = None
        self._parse_count = 0
        self._publish_count = 0
        self._reject_count = 0
        self._last_switch_attempt: Optional[float] = None

        self._serial_thread: Optional[threading.Thread] = None
        self._serial_stop = threading.Event()
        self._flat_frame_values: Optional[List[Optional[float]]] = None

        self.left_jtc_pub = self.create_publisher(
            JointTrajectory,
            f"/{self.left_jtc_controller}/joint_trajectory",
            10,
        )
        self.right_jtc_pub = self.create_publisher(
            JointTrajectory,
            f"/{self.right_jtc_controller}/joint_trajectory",
            10,
        )
        self.left_fpc_pub = self.create_publisher(
            Float64MultiArray,
            f"/{self.left_fpc_controller}/commands",
            10,
        )
        self.right_fpc_pub = self.create_publisher(
            Float64MultiArray,
            f"/{self.right_fpc_controller}/commands",
            10,
        )
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/left_gripper_controller/commands",
            10,
        )
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/right_gripper_controller/commands",
            10,
        )

        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)

        self._switch_client = self.create_client(
            SwitchController,
            "/controller_manager/switch_controller",
        )
        self._list_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
        )

        self._urdf_ready = False
        self._urdf_attempts = 0
        self._urdf_future = None
        self._param_client = self.create_client(
            GetParameters,
            f"/{self.robot_description_node}/get_parameters",
        )
        self._init_urdf_timer = self.create_timer(0.5, self._try_load_urdf)

        control_period = 1.0 / max(1.0, self.control_rate_hz)
        self._control_timer = self.create_timer(control_period, self._control_tick)

        self._start_serial_thread()

    def _try_load_urdf(self) -> None:
        if self._urdf_ready:
            return

        if URDF is None:
            self.get_logger().error("urdf_parser_py is not available.")
            self._urdf_ready = True
            return

        if self._urdf_future is None:
            if not self._param_client.wait_for_service(timeout_sec=0.2):
                self._urdf_attempts += 1
                if self._urdf_attempts % 5 == 0:
                    self.get_logger().warn("Waiting for robot_description parameter service...")
                if self._urdf_attempts >= 20:
                    self.get_logger().warn("Skipping URDF limits; parameter service not available.")
                    self._urdf_ready = True
                return

            request = GetParameters.Request()
            request.names = [self.robot_description_param]
            self._urdf_future = self._param_client.call_async(request)
            return

        if not self._urdf_future.done():
            return

        result = self._urdf_future.result()
        self._urdf_future = None

        if result is None:
            self._urdf_attempts += 1
            if self._urdf_attempts % 5 == 0:
                self.get_logger().warn("Failed to get robot_description.")
            if self._urdf_attempts >= 20:
                self.get_logger().warn("Skipping URDF limits; using unclipped inputs.")
                self._urdf_ready = True
            return

        values = result.values
        if not values or values[0].string_value == "":
            self._urdf_attempts += 1
            if self._urdf_attempts % 5 == 0:
                self.get_logger().warn("robot_description is empty.")
            if self._urdf_attempts >= 20:
                self.get_logger().warn("Skipping URDF limits; using unclipped inputs.")
                self._urdf_ready = True
            return

        try:
            robot = URDF.from_xml_string(values[0].string_value)
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to parse robot_description: {exc}")
            return

        limits = {}
        for joint in robot.joints:
            if joint.limit is None:
                limits[joint.name] = (None, None)
                continue
            limits[joint.name] = (joint.limit.lower, joint.limit.upper)

        self._limits = limits
        self._urdf_ready = True
        self.get_logger().info("Loaded joint limits from URDF.")

    def _start_serial_thread(self) -> None:
        if serial is None:
            self.get_logger().error("pyserial is not available. Install python3-serial.")
            return

        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()

    def _serial_loop(self) -> None:
        ser = None
        while not self._serial_stop.is_set():
            if ser is None:
                try:
                    ser = serial.Serial(
                        self.serial_port,
                        self.serial_baud,
                        timeout=self.serial_timeout_sec,
                    )
                    self.get_logger().info(f"Connected to {self.serial_port} @ {self.serial_baud}")
                except Exception as exc:
                    self.get_logger().warn(f"Serial open failed: {exc}")
                    time.sleep(1.0)
                    continue

            try:
                line = ser.readline().decode("utf-8", errors="ignore").strip()
            except Exception as exc:
                self.get_logger().warn(f"Serial read failed: {exc}")
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
                time.sleep(0.5)
                continue

            if not line:
                continue

            self._parse_line(line)

    def _parse_line(self, line: str) -> None:
        if line.startswith("@,"):
            values = self._parse_values(line[2:])
            if values is None:
                return
            self._flat_frame_values = values
            if len(values) < self._flat_frame_min_length():
                return
            values = self._consume_flat_frame()
            if values is None:
                return
            self._parse_flat_frame(values)
            return

        if self._flat_frame_values is not None:
            values = self._parse_values(line)
            if values is not None:
                self._flat_frame_values.extend(values)
                if len(self._flat_frame_values) < self._flat_frame_min_length():
                    return
                values = self._consume_flat_frame()
                if values is None:
                    return
                self._parse_flat_frame(values)
                return
            self._flat_frame_values = None

        match = LINE_REGEX.search(line)
        if match:
            left_raw = match.group(1)
            right_raw = match.group(2)
        else:
            legacy = LEGACY_REGEX.search(line)
            if not legacy:
                return
            left_raw = legacy.group(1)
            right_raw = legacy.group(2)

        left_vals = self._parse_values(left_raw)
        right_vals = self._parse_values(right_raw)

        if left_vals is None or right_vals is None:
            return

        left_targets = self._convert_and_select(left_vals, self.left_joint_indices)
        right_targets = self._convert_and_select(right_vals, self.right_joint_indices)
        left_gripper_target = self._convert_gripper_index(left_vals, self.left_gripper_index)
        right_gripper_target = self._convert_gripper_index(right_vals, self.right_gripper_index)
        self._update_targets(left_targets, right_targets, left_gripper_target, right_gripper_target)

    def _parse_flat_frame(self, values: List[Optional[float]]) -> None:
        left_targets = self._convert_and_select(values, self.flat_left_joint_indices)
        right_targets = self._convert_and_select(values, self.flat_right_joint_indices)
        left_gripper_target = self._convert_gripper_index(values, self.flat_left_gripper_index)
        right_gripper_target = self._convert_gripper_index(values, self.flat_right_gripper_index)
        self._update_targets(left_targets, right_targets, left_gripper_target, right_gripper_target)

    def _update_targets(
        self,
        left_targets: List[Optional[float]],
        right_targets: List[Optional[float]],
        left_gripper_target: Optional[float],
        right_gripper_target: Optional[float],
    ) -> None:
        with self._lock:
            for idx, value in enumerate(left_targets):
                if value is None:
                    continue
                clipped = self._clip_value(self.left_joint_names[idx], value)
                if self._accept_sample_step(
                    self.left_joint_names[idx], clipped, self._latest_left[idx], self.max_sample_step_rad
                ):
                    self._latest_left[idx] = clipped
            for idx, value in enumerate(right_targets):
                if value is None:
                    continue
                clipped = self._clip_value(self.right_joint_names[idx], value)
                if self._accept_sample_step(
                    self.right_joint_names[idx], clipped, self._latest_right[idx], self.max_sample_step_rad
                ):
                    self._latest_right[idx] = clipped

            if self.enable_grippers:
                if left_gripper_target is not None:
                    clipped = self._clip_gripper_value(
                        self.left_gripper_joint_name, left_gripper_target)
                    if self._accept_sample_step(
                        self.left_gripper_joint_name,
                        clipped,
                        self._latest_left_gripper,
                        self.max_gripper_sample_step_rad,
                    ):
                        self._latest_left_gripper = clipped
                if right_gripper_target is not None:
                    clipped = self._clip_gripper_value(
                        self.right_gripper_joint_name, right_gripper_target)
                    if self._accept_sample_step(
                        self.right_gripper_joint_name,
                        clipped,
                        self._latest_right_gripper,
                        self.max_gripper_sample_step_rad,
                    ):
                        self._latest_right_gripper = clipped

            if not self._have_any_sample:
                if (
                    any(v is not None for v in left_targets)
                    or any(v is not None for v in right_targets)
                    or left_gripper_target is not None
                    or right_gripper_target is not None
                ):
                    self._have_any_sample = True

            if self._first_data_time is None:
                self._first_data_time = time.time()
            self._last_parse_time = time.time()
            self._parse_count += 1

    def _flat_frame_min_length(self) -> int:
        return max(
            self.flat_left_joint_indices
            + self.flat_right_joint_indices
            + [self.flat_left_gripper_index, self.flat_right_gripper_index]
        ) + 1

    def _consume_flat_frame(self) -> Optional[List[Optional[float]]]:
        values = self._flat_frame_values
        self._flat_frame_values = None
        if values is None or len(values) < self._flat_frame_min_length():
            return None
        return values

    def _accept_sample_step(
        self,
        joint_name: str,
        value: float,
        previous_value: float,
        max_step_rad: float,
    ) -> bool:
        if not self.enable_sample_step_filter or not self._have_any_sample:
            return True
        if max_step_rad <= 0.0:
            return True

        step = abs(value - previous_value)
        if step <= max_step_rad:
            return True

        self._reject_count += 1
        self.get_logger().warn(
            f"Rejected serial outlier for {joint_name}: step={step:.3f} rad > {max_step_rad:.3f} rad",
            throttle_duration_sec=1.0,
        )
        return False

    def _parse_values(self, raw: str) -> Optional[List[Optional[float]]]:
        parts = [p.strip() for p in raw.split(",") if p.strip() != ""]
        values: List[Optional[float]] = []
        for part in parts:
            if part.lower() == "nan":
                values.append(None)
                continue
            try:
                values.append(float(part))
            except ValueError:
                return None
        return values

    def _convert_and_select(self, values: List[Optional[float]], indices: List[int]) -> List[Optional[float]]:
        result: List[Optional[float]] = []
        for idx in indices:
            if idx >= len(values):
                result.append(None)
                continue
            value = values[idx]
            if value is None:
                result.append(None)
                continue
            result.append(self._to_rad(value))
        return result

    def _convert_index(self, values: List[Optional[float]], idx: int) -> Optional[float]:
        if idx < 0 or idx >= len(values):
            return None
        value = values[idx]
        if value is None:
            return None
        return self._to_rad(value)

    def _convert_gripper_index(self, values: List[Optional[float]], idx: int) -> Optional[float]:
        if idx < 0 or idx >= len(values):
            return None
        value = values[idx]
        if value is None:
            return None
        if self.gripper_unit == "deg":
            return math.radians(value)
        if self.gripper_unit == "rad":
            return value
        return self._to_rad(value)

    def _to_rad(self, value: float) -> float:
        if self.input_unit == "rad":
            return value
        if self.input_unit == "deg":
            return math.radians(value)
        # centi_deg
        return math.radians(value / 100.0)

    def _clip_value(self, joint_name: str, value: float) -> float:
        if not self._limits:
            return value
        lower, upper = self._limits.get(joint_name, (None, None))
        if lower is not None:
            value = max(lower, value)
        if upper is not None:
            value = min(upper, value)
        return value

    def _clip_gripper_value(self, joint_name: str, value: float) -> float:
        if self.gripper_clip_to_urdf:
            return self._clip_value(joint_name, value)
        return max(self.min_gripper, min(self.max_gripper, value))

    def _on_joint_state(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_state_map[name] = pos

    def _control_tick(self) -> None:
        now = time.time()
        if self._last_parse_time is not None and now - self._last_parse_time > 2.0:
            if self._parse_count > 0:
                self.get_logger().warn("No recent serial data parsed (last > 2s).")
            self._parse_count = 0

        if self._first_data_time is None:
            return

        elapsed = now - self._first_data_time

        if elapsed < self.warmup_sec:
            return

        # In sbopenarm.launch.py the FPC controllers are spawned active for serial teleop.
        # Keep controller-manager service calls out of the hot path; repeated switch
        # timeouts here can collapse the command topic to ~0.3 Hz.
        if self.auto_switch_controllers:
            self._try_activate_fpc_throttled(now)

        if self._ramp_start_time is None:
            start_left, start_right = self._get_current_positions()
            start_left_gripper, start_right_gripper = self._get_current_gripper_positions()
            if start_left is None or start_right is None or self.startup_ramp_sec <= 0.0:
                self._ramp_start_time = now
                self._publish_fpc()
                return
            self._ramp_start_time = now
            self._ramp_start_left = start_left
            self._ramp_start_right = start_right
            self._ramp_start_left_gripper = start_left_gripper
            self._ramp_start_right_gripper = start_right_gripper

        ramp_elapsed = now - self._ramp_start_time
        if ramp_elapsed < self.startup_ramp_sec:
            self._publish_fpc_ramped(ramp_elapsed / self.startup_ramp_sec)
        else:
            self._publish_fpc()

    def _try_activate_fpc_throttled(self, now: float) -> None:
        if (
            self._last_switch_attempt is not None
            and now - self._last_switch_attempt < self.switch_retry_period_sec
        ):
            return

        self._last_switch_attempt = now
        if self._fpc_already_active():
            return

        self._activate_controllers([
            self.left_fpc_controller,
            self.right_fpc_controller,
        ])

    def _get_latest_targets(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        with self._lock:
            if not self._have_any_sample:
                return None, None
            return list(self._latest_left), list(self._latest_right)

    def _get_latest_gripper_targets(self) -> Optional[Tuple[float, float]]:
        if not self.enable_grippers:
            return None
        with self._lock:
            if not self._have_any_sample:
                return None
            return self._latest_left_gripper, self._latest_right_gripper

    def _send_jtc_goal(self) -> bool:
        if not self._ensure_jtc_active():
            return False

        left_targets, right_targets = self._get_latest_targets()
        if left_targets is None or right_targets is None:
            return False

        with self._lock:
            for joint in self.left_joint_names + self.right_joint_names:
                if joint not in self._joint_state_map:
                    self.get_logger().warn("Waiting for complete joint_states before JTC goal.")
                    return False

        left_msg = JointTrajectory()
        left_msg.joint_names = self.left_joint_names
        left_point = JointTrajectoryPoint()
        left_point.positions = left_targets
        left_point.time_from_start = Duration(seconds=self.trajectory_duration_sec).to_msg()
        left_msg.points = [left_point]

        right_msg = JointTrajectory()
        right_msg.joint_names = self.right_joint_names
        right_point = JointTrajectoryPoint()
        right_point.positions = right_targets
        right_point.time_from_start = Duration(seconds=self.trajectory_duration_sec).to_msg()
        right_msg.points = [right_point]

        self.left_jtc_pub.publish(left_msg)
        self.right_jtc_pub.publish(right_msg)
        self.get_logger().info("Sent JTC goal for safe 5s transition.")
        return True

    def _get_current_positions(self) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        with self._lock:
            left_positions = []
            right_positions = []
            for joint in self.left_joint_names:
                if joint not in self._joint_state_map:
                    return None, None
                left_positions.append(self._joint_state_map[joint])
            for joint in self.right_joint_names:
                if joint not in self._joint_state_map:
                    return None, None
                right_positions.append(self._joint_state_map[joint])
            return left_positions, right_positions

    def _get_current_gripper_positions(self) -> Tuple[Optional[float], Optional[float]]:
        if not self.enable_grippers:
            return None, None
        with self._lock:
            return (
                self._joint_state_map.get(self.left_gripper_joint_name),
                self._joint_state_map.get(self.right_gripper_joint_name),
            )

    def _publish_fpc_ramped(self, alpha: float) -> None:
        left_targets, right_targets = self._get_latest_targets()
        if left_targets is None or right_targets is None:
            return
        if self._ramp_start_left is None or self._ramp_start_right is None:
            return

        alpha = max(0.0, min(1.0, alpha))
        left_msg = Float64MultiArray()
        left_msg.data = [
            s + alpha * (t - s) for s, t in zip(self._ramp_start_left, left_targets)
        ]
        right_msg = Float64MultiArray()
        right_msg.data = [
            s + alpha * (t - s) for s, t in zip(self._ramp_start_right, right_targets)
        ]

        self.left_fpc_pub.publish(left_msg)
        self.right_fpc_pub.publish(right_msg)
        self._publish_grippers_ramped(alpha)
        self._last_publish_time = time.time()
        self._publish_count += 1

    def _publish_grippers_ramped(self, alpha: float) -> None:
        targets = self._get_latest_gripper_targets()
        if targets is None:
            return

        left_target, right_target = targets
        left_start = self._ramp_start_left_gripper
        right_start = self._ramp_start_right_gripper

        left_value = left_target if left_start is None else left_start + alpha * (left_target - left_start)
        right_value = right_target if right_start is None else right_start + alpha * (right_target - right_start)
        self._publish_grippers(left_value, right_value)

    def _publish_grippers(self, left_value: float, right_value: float) -> None:
        if not self.enable_grippers:
            return

        left_msg = Float64MultiArray()
        left_msg.data = [left_value]
        right_msg = Float64MultiArray()
        right_msg.data = [right_value]

        self.left_gripper_pub.publish(left_msg)
        self.right_gripper_pub.publish(right_msg)

    def _ensure_jtc_active(self) -> bool:
        if self._controllers_state([self.left_jtc_controller, self.right_jtc_controller]).get(self.left_jtc_controller) == "active":
            return True

        if not self._switch_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Waiting for /controller_manager/switch_controller")
            return False

        # Deactivate FPC to avoid resource conflicts
        self._deactivate_controllers([
            self.left_fpc_controller,
            self.right_fpc_controller,
        ])

        # Activate JTC controllers
        self._activate_controllers([
            self.left_jtc_controller,
            self.right_jtc_controller,
        ])

        return self._wait_for_controllers_state(
            [self.left_jtc_controller, self.right_jtc_controller],
            target_state="active",
            timeout_sec=2.0,
        )

    def _switch_to_fpc(self) -> bool:
        now = time.time()
        if self._last_switch_attempt is not None and now - self._last_switch_attempt < 1.0:
            return False
        self._last_switch_attempt = now

        if self._fpc_already_active():
            return True

        if not self._switch_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn("Waiting for /controller_manager/switch_controller")
            return False

        # 1) Deactivate any controllers that might hold position interfaces
        self._deactivate_controllers([
            self.left_jtc_controller,
            self.right_jtc_controller,
            self.left_teleop_stream_controller,
            self.right_teleop_stream_controller,
        ])

        if not self._wait_for_controllers_state(
            [self.left_jtc_controller, self.right_jtc_controller],
            target_state="inactive",
            timeout_sec=2.0,
        ):
            self.get_logger().warn("Waiting for JTC controllers to deactivate...")
            return False

        # 2) Activate FPC controllers
        self._activate_controllers([
            self.left_fpc_controller,
            self.right_fpc_controller,
        ])

        if self._fpc_already_active():
            self.get_logger().info("Switched to ForwardPositionController.")
            return True

        self.get_logger().warn("Failed to switch controllers.")
        return False

    def _activate_controllers(self, controllers: List[str]) -> None:
        request = SwitchController.Request()
        request.activate_controllers = controllers
        request.start_controllers = controllers
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout.sec = 2

        future = self._switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.5)

    def _deactivate_controllers(self, controllers: List[str]) -> None:
        request = SwitchController.Request()
        request.deactivate_controllers = controllers
        request.stop_controllers = controllers
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.activate_asap = True
        request.timeout.sec = 2

        future = self._switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.5)

    def _fpc_already_active(self) -> bool:
        if not self._list_client.wait_for_service(timeout_sec=0.2):
            return False

        future = self._list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return False

        controllers = future.result().controller
        states = {c.name: c.state for c in controllers}
        return (
            states.get(self.left_fpc_controller) == "active"
            and states.get(self.right_fpc_controller) == "active"
        )

    def _controllers_state(self, names: List[str]) -> Dict[str, str]:
        if not self._list_client.wait_for_service(timeout_sec=0.2):
            return {}

        future = self._list_client.call_async(ListControllers.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if not future.done() or future.result() is None:
            return {}

        controllers = future.result().controller
        states = {c.name: c.state for c in controllers}
        return {name: states.get(name, "") for name in names}

    def _wait_for_controllers_state(
        self,
        names: List[str],
        target_state: str,
        timeout_sec: float,
    ) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            states = self._controllers_state(names)
            if states and all(state == target_state for state in states.values()):
                return True
            time.sleep(0.1)
        return False

    def _publish_fpc(self) -> None:
        left_targets, right_targets = self._get_latest_targets()
        if left_targets is None or right_targets is None:
            return

        left_msg = Float64MultiArray()
        left_msg.data = left_targets
        right_msg = Float64MultiArray()
        right_msg.data = right_targets

        self.left_fpc_pub.publish(left_msg)
        self.right_fpc_pub.publish(right_msg)
        gripper_targets = self._get_latest_gripper_targets()
        if gripper_targets is not None:
            self._publish_grippers(*gripper_targets)
        self._last_publish_time = time.time()
        self._publish_count += 1

    def destroy_node(self) -> bool:
        self._serial_stop.set()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TeleopSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
