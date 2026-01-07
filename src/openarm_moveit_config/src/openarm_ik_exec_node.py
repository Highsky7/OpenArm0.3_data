#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetPositionIK
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_JOINTS = ['rev1','rev2','rev3','rev4','rev5','rev6','rev7']
DEFAULT_SEED   = [0.0, -0.3, 0.6, 0.0, 0.0, 0.0, 0.0]


class IkExecNode(Node):
    def __init__(self):
        super().__init__('ik_exec_node')

        # ---- parameters (declare ONCE)
        self.declare_parameter('group_name', 'arm')
        self.declare_parameter('ik_link', 'gripper_center')
        self.declare_parameter('base_frame', 'link1')
        self.declare_parameter('joint_names', DEFAULT_JOINTS)
        self.declare_parameter('seed', DEFAULT_SEED)
        self.declare_parameter('move_time_sec', 3)

        def _as_list(v, fb):
            return list(v) if isinstance(v, (list, tuple)) else list(fb)

        self.group_name    = self.get_parameter('group_name').value or 'arm'
        self.ik_link       = self.get_parameter('ik_link').value or 'gripper_center'
        self.base_frame    = self.get_parameter('base_frame').value or 'link1'
        self.joint_names   = _as_list(self.get_parameter('joint_names').value, DEFAULT_JOINTS)
        self.seed          = _as_list(self.get_parameter('seed').value, DEFAULT_SEED)
        self.move_time_sec = int(self.get_parameter('move_time_sec').value or 3)
        self.current_state = None

        if len(self.seed) != len(self.joint_names):
            if len(self.seed) > len(self.joint_names):
                self.seed = self.seed[:len(self.joint_names)]
            else:
                self.seed = self.seed + [0.0] * (len(self.joint_names) - len(self.seed))

        # ---- IK service client
        self.ik_cli = self.create_client(GetPositionIK, '/compute_ik')
        if not self.ik_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service /compute_ik not available')
        else:
            self.get_logger().info('IK service ready')

        # ---- Trajectory action client
        self.traj_ac = ActionClient(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # ---- Subscription (CALLBACK MUST EXIST in this class)
        self.sub = self.create_subscription(PoseStamped, '/ee_pose_goal', self.on_goal_pose, 10)
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.get_logger().info('Listening: /ee_pose_goal (PoseStamped)')

    # ===================== CALL BACKS =====================

    def _joint_cb(self, msg: JointState):
        """ /joint_states callback """
        self.current_state = msg

    def on_goal_pose(self, msg: PoseStamped):
        # frame fix
        if not msg.header.frame_id:
            msg.header.frame_id = self.base_frame
        if msg.header.frame_id != self.base_frame:
            self.get_logger().warn(
                f"Pose frame_id is '{msg.header.frame_id}', expected '{self.base_frame}'. Using as-is, but ensure TF is connected."
            )

        # IK request
        req = GetPositionIK.Request()
        req.ik_request.group_name   = self.group_name
        req.ik_request.ik_link_name = self.ik_link

        js = JointState()
        js.name = list(self.joint_names)

        if self.current_state and set(self.joint_names).issubset(set(self.current_state.name)):
            # 현재 로봇 상태를 seed로 사용
            name_to_pos = {n: p for n, p in zip(self.current_state.name, self.current_state.position)}
            js.position = [name_to_pos[j] for j in self.joint_names]
        else:
            # fallback: 기본 시드 사용
            js.position = list(self.seed)

        js.velocity = [0.0] * len(self.joint_names)
        js.effort   = [0.0] * len(self.joint_names)
        req.ik_request.robot_state.joint_state = js

        req.ik_request.pose_stamped = msg
        req.ik_request.timeout = Duration(seconds=5).to_msg()

        self.get_logger().info('Calling /compute_ik ...')
        future = self.ik_cli.call_async(req)
        future.add_done_callback(self._after_ik)

    def _after_ik(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().error(f'IK call failed: {e}')
            return
        if not res or res.error_code.val != 1:
            code = None if not res else res.error_code.val
            self.get_logger().error(
                f'IK failed with code {code}. '
                f'pose=({res.solution.joint_state.header.frame_id if res else "?"})'
            )
            return


        js = res.solution.joint_state
        name_to_pos = {n: p for n, p in zip(js.name, js.position)}
        q = [name_to_pos[j] for j in self.joint_names]
        self.get_logger().info('IK solution: ' + ', '.join(f'{n}:{v:.3f}' for n, v in zip(self.joint_names, q)))

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = q
        pt.time_from_start = Duration(seconds=self.move_time_sec).to_msg()
        traj.points.append(pt)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        if not self.traj_ac.wait_for_server(timeout_sec=0.5):
            self.get_logger().error('Trajectory action server not available')
            return

        self.get_logger().info('Sending trajectory...')
        send_future = self.traj_ac.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, send_future):
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return
        self.get_logger().info('Trajectory accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, result_future):
        try:
            result_msg = result_future.result().result
            self.get_logger().info(f'Action finished. error_code={getattr(result_msg, "error_code", 0)}')
        except Exception as e:
            self.get_logger().error(f'Action result error: {e}')


def main():
    rclpy.init()
    node = IkExecNode()
    try:
        rclpy.spin(node)  # 비동기 체인이라 단일 스레드로 OK
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
