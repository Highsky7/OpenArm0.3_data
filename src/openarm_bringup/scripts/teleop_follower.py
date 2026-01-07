#!/usr/bin/python3
# teleop_follower.py (핵심 부분만)
import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray

def clamp(x, lo, hi): return lo if x < lo else hi if x > hi else x

class TeleopFollower(Node):
    def __init__(self):
        super().__init__('teleop_follower')
        # 기존 팔 7축 세팅 그대로...
        self.idx_map = self.declare_parameter('idx_map', [0,1,2,3,4,5,6]).value
        self.rate_hz = self.declare_parameter('rate_hz', 200.0).value
        self.alpha   = self.declare_parameter('alpha', 0.15).value
        self.vel_lim = self.declare_parameter('vel_limit', 2.0).value
        mins = self.declare_parameter('min_limits', [-math.pi]*7).value
        maxs = self.declare_parameter('max_limits', [ math.pi]*7).value
        self.min_lim = list(mins); self.max_lim = list(maxs)
        self.target = [0.0]*7; self.prev=[0.0]*7; self.have_tele=False

        # ★ 그리퍼: tele 인덱스만 받아 그대로 퍼블리시
        self.grip_enable = self.declare_parameter('gripper_enable', True).value
        self.grip_idx    = self.declare_parameter('gripper_tele_index', 7).value
        self.grip_topic  = self.declare_parameter('gripper_topic', '/gripper_cmd').value
        self.pub_grip = self.create_publisher(Float64, self.grip_topic, 10)

        self.create_subscription(JointState, '/teleop/joint_states', self.on_tele, 10)
        self.pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.create_timer(1.0/self.rate_hz, self.tick)

    def on_tele(self, js: JointState):
        vals = list(js.position)

        # 팔 7축 처리(기존)
        mapped=[]
        for k in range(7):
            i = self.idx_map[k] if k < len(self.idx_map) else k
            v = vals[i] if i < len(vals) else 0.0
            v = clamp(v, self.min_lim[k], self.max_lim[k])
            mapped.append(v)
        if not self.have_tele:
            self.target = mapped; self.prev = mapped[:]; self.have_tele=True
        else:
            self.target = [ self.alpha*m + (1.0-self.alpha)*t for m,t in zip(mapped, self.target) ]

        # ★ 그리퍼: tele7(기본)을 라디안 그대로 패스스루
        if self.grip_enable and self.grip_idx < len(vals):
            self.pub_grip.publish(Float64(data=float(vals[self.grip_idx])))

    def tick(self):
        if not self.have_tele: return
        dt = 1.0/self.rate_hz
        out=[]
        for t,p,lo,hi in zip(self.target, self.prev, self.min_lim, self.max_lim):
            dv = clamp(t-p, -self.vel_lim*dt, self.vel_lim*dt)
            out.append(clamp(p+dv, lo, hi))
        self.prev = out
        msg = Float64MultiArray(); msg.data = out
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TeleopFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
