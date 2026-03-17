"""
R1 Simulator — drives the physical R1 model in Gazebo.

R1 is manual/automatic in the real competition.  Here we drive it as a
scripted state machine so R2 can practise the full match sequence.

Sequence:
  START → pick staff at staff rack → wait for R2 (at staff rack) →
  assemble weapon → exit Martial Club → DONE

Uses a second navigator instance (on /r1/* topics) to drive R1.

Publishes:
  /r1_status       (String)     — current state name
  /r1/goal_pose    (PoseStamped) — navigation goals for the R1 navigator
"""

import math
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class S(Enum):
    START = auto()
    NAV_TO_STAFF_RACK = auto()
    PICKING_STAFF = auto()
    WAITING_FOR_R2 = auto()
    ASSEMBLING = auto()
    EXITING = auto()
    DONE = auto()


# ── World-frame positions (left team) ───────────────────────────────────────
START_POS       = (-5.54, 5.54)
STAFF_RACK_LEFT = (-3.44, 6.065)
EXIT_POS        = (-5.54, 5.54)

# R1 starts at (-5.54, 5.54, yaw=0) — facing +x
R1_START_X   = -5.54
R1_START_Y   =  5.54
R1_START_YAW =  0.0


class R1Sim(Node):

    def __init__(self):
        super().__init__('r1_sim')

        # -- Parameters --
        self.declare_parameter('pick_staff_dur', 2.0)
        self.declare_parameter('assemble_dur', 3.0)

        self.pick_dur = self.get_parameter('pick_staff_dur').value
        self.asm_dur = self.get_parameter('assemble_dur').value

        # -- Pub / Sub --
        self.pub_status = self.create_publisher(String, '/r1_status', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/r1/goal_pose', 10)
        self.sub_r2 = self.create_subscription(
            String, '/kfs_status', self._r2_status_cb, 10,
        )
        self.sub_reached = self.create_subscription(
            PoseStamped, '/r1/waypoint_reached', self._reached_cb, 10,
        )

        # -- State --
        self.state = S.START
        self.action_start = None
        self._action_dur = 0.0
        self.r2_ready = False

        # 10 Hz tick
        self.create_timer(0.1, self._tick)

        # 2 s boot delay
        self.action_start = self.get_clock().now()
        self._action_dur = 2.0
        self.get_logger().info('R1 Sim ready — starting in 2 s')

    # ── Coordinate transform (world → R1 odom) ──────────────────────────────
    @staticmethod
    def _world_to_odom(wx, wy):
        dx = wx - R1_START_X
        dy = wy - R1_START_Y
        c = math.cos(-R1_START_YAW)
        s = math.sin(-R1_START_YAW)
        return c * dx - s * dy, s * dx + c * dy

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _r2_status_cb(self, msg: String):
        if msg.data == 'WAITING_FOR_R1':
            self.r2_ready = True

    def _reached_cb(self, _msg: PoseStamped):
        if self.state == S.NAV_TO_STAFF_RACK:
            self._begin_action(S.PICKING_STAFF, self.pick_dur)
            self.get_logger().info('At staff rack — picking staff …')

        elif self.state == S.EXITING:
            self._set_state(S.DONE)
            self.get_logger().info('R1 exited — R2 may enter Meihua Forest')

    # ── Main tick ────────────────────────────────────────────────────────────
    def _tick(self):
        if self.state == S.START:
            if self._timer_done():
                self.get_logger().info('Driving to staff rack')
                self._set_state(S.NAV_TO_STAFF_RACK)
                self._send_goal(*STAFF_RACK_LEFT)

        elif self.state == S.PICKING_STAFF:
            if self._timer_done():
                self.get_logger().info('Staff picked — waiting for R2 at staff rack')
                self._set_state(S.WAITING_FOR_R2)

        elif self.state == S.WAITING_FOR_R2:
            if self.r2_ready:
                self._begin_action(S.ASSEMBLING, self.asm_dur)
                self.get_logger().info('Assembling weapon …')

        elif self.state == S.ASSEMBLING:
            if self._timer_done():
                self.get_logger().info('Weapon assembled! Exiting Martial Club')
                self._set_state(S.EXITING)
                self._send_goal(*EXIT_POS)

        # S.DONE — nothing to do

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _send_goal(self, wx: float, wy: float):
        ox, oy = self._world_to_odom(wx, wy)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'r1_odom'
        msg.pose.position.x = ox
        msg.pose.position.y = oy
        self.pub_goal.publish(msg)
        self.get_logger().info(
            f'R1 goal  world({wx:.2f}, {wy:.2f}) → odom({ox:.2f}, {oy:.2f})'
        )

    def _begin_action(self, state: S, duration: float):
        self.state = state
        self.action_start = self.get_clock().now()
        self._action_dur = duration
        self._pub_status()

    def _set_state(self, state: S):
        self.state = state
        self._pub_status()

    def _timer_done(self) -> bool:
        if self.action_start is None:
            return True
        elapsed = (self.get_clock().now() - self.action_start).nanoseconds / 1e9
        return elapsed >= self._action_dur

    def _pub_status(self):
        msg = String()
        msg.data = self.state.name
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = R1Sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
