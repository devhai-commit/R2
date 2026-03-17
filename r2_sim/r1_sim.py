"""
R1 Simulator — scripted R1 robot behaviour (no physics).

R1 is manual/automatic in the real competition.  Here we simulate its
behaviour as a timed state machine so R2 can practise the full match
sequence including weapon assembly.

Sequence:
  START → pick staff → go to assembly point → wait for R2 →
  assemble weapon → exit Martial Club → DONE

Publishes:
  /r1_status  (String)   — current state name
  /r1_pose    (PoseStamped) — simulated world-frame position
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
    NAV_TO_ASSEMBLY = auto()
    WAITING_FOR_R2 = auto()
    ASSEMBLING = auto()
    EXITING = auto()
    DONE = auto()


# ── Waypoints (world frame, left team) ──────────────────────────────────────
WAYPOINTS = {
    S.START:              (-5.54,  5.54),
    S.NAV_TO_STAFF_RACK:  (-3.44,  6.065),
    S.NAV_TO_ASSEMBLY:    (-2.40,  5.20),
    S.EXITING:            (-5.54,  5.54),
}

# Timed-action durations (seconds)
PICK_STAFF_DUR = 2.0
ASSEMBLE_DUR   = 3.0
# Simulated drive speed (m/s) — used to estimate travel time
DRIVE_SPEED = 0.5


class R1Sim(Node):

    def __init__(self):
        super().__init__('r1_sim')

        # -- Parameters --
        self.declare_parameter('drive_speed', DRIVE_SPEED)
        self.declare_parameter('pick_staff_dur', PICK_STAFF_DUR)
        self.declare_parameter('assemble_dur', ASSEMBLE_DUR)

        self.speed = self.get_parameter('drive_speed').value
        self.pick_dur = self.get_parameter('pick_staff_dur').value
        self.asm_dur = self.get_parameter('assemble_dur').value

        # -- Pub / Sub --
        self.pub_status = self.create_publisher(String, '/r1_status', 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/r1_pose', 10)
        self.sub_r2 = self.create_subscription(
            String, '/kfs_status', self._r2_status_cb, 10,
        )

        # -- State --
        self.state = S.START
        self.wx, self.wy = WAYPOINTS[S.START]
        self.action_start = None
        self.action_dur = 0.0
        self.r2_ready = False   # R2 at assembly point?

        # 10 Hz tick
        self.create_timer(0.1, self._tick)
        self.get_logger().info('R1 Sim ready — waiting 2 s then starting')

        # Small initial delay so everything boots up
        self.action_start = self.get_clock().now()
        self.action_dur = 2.0

    # ── R2 status listener ───────────────────────────────────────────────────
    def _r2_status_cb(self, msg: String):
        if msg.data == 'WAITING_FOR_R1':
            self.r2_ready = True

    # ── Main tick ────────────────────────────────────────────────────────────
    def _tick(self):
        self._pub_pose()

        if self.state == S.START:
            if self._timer_done():
                self._drive_to(S.NAV_TO_STAFF_RACK)

        elif self.state == S.NAV_TO_STAFF_RACK:
            if self._timer_done():
                self._start_action(S.PICKING_STAFF, self.pick_dur)

        elif self.state == S.PICKING_STAFF:
            if self._timer_done():
                self.get_logger().info('Staff picked')
                self._drive_to(S.NAV_TO_ASSEMBLY)

        elif self.state == S.NAV_TO_ASSEMBLY:
            if self._timer_done():
                self._set_state(S.WAITING_FOR_R2)
                self.get_logger().info('At assembly point — waiting for R2')

        elif self.state == S.WAITING_FOR_R2:
            if self.r2_ready:
                self._start_action(S.ASSEMBLING, self.asm_dur)
                self.get_logger().info('Assembling weapon …')

        elif self.state == S.ASSEMBLING:
            if self._timer_done():
                self.get_logger().info('Weapon assembled! Exiting Martial Club')
                self._drive_to(S.EXITING)

        elif self.state == S.EXITING:
            if self._timer_done():
                self._set_state(S.DONE)
                self.get_logger().info('R1 exited — R2 may enter Meihua Forest')

        # S.DONE — nothing to do

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _drive_to(self, state: S):
        """Simulate driving to the waypoint for *state*."""
        tx, ty = WAYPOINTS[state]
        dist = math.hypot(tx - self.wx, ty - self.wy)
        drive_time = dist / self.speed if self.speed > 0 else 0.0
        self.wx, self.wy = tx, ty
        self._start_action(state, drive_time)
        self.get_logger().info(
            f'Driving to {state.name} ({tx:.2f}, {ty:.2f}) '
            f'ETA {drive_time:.1f} s'
        )

    def _start_action(self, state: S, duration: float):
        self.state = state
        self.action_start = self.get_clock().now()
        self.action_dur = duration
        self._pub_status()

    def _set_state(self, state: S):
        self.state = state
        self._pub_status()

    def _timer_done(self) -> bool:
        if self.action_start is None:
            return True
        elapsed = (self.get_clock().now() - self.action_start).nanoseconds / 1e9
        return elapsed >= self.action_dur

    def _pub_status(self):
        msg = String()
        msg.data = self.state.name
        self.pub_status.publish(msg)

    def _pub_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x = self.wx
        msg.pose.position.y = self.wy
        self.pub_pose.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = R1Sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
