"""
Waypoint navigator node.

Supports two input modes:
  /goal_pose  (PoseStamped)  — single waypoint (backwards-compatible)
  /path       (Path)         — multi-waypoint path from A* planner

Drives the robot along each waypoint using a P-controller on /cmd_vel.
Publishes /waypoint_reached when the final waypoint is reached.
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path


def _yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # -- Parameters --
        self.declare_parameter('pos_tolerance', 0.20)
        self.declare_parameter('ang_tolerance', 0.15)
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.5)
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 2.0)
        # Intermediate waypoints can use a looser tolerance
        self.declare_parameter('intermediate_tolerance', 0.30)

        self.pos_tol = self.get_parameter('pos_tolerance').value
        self.ang_tol = self.get_parameter('ang_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value
        self.inter_tol = self.get_parameter('intermediate_tolerance').value

        # -- State --
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False
        self.waypoints = deque()   # queue of (x, y)
        self.final_goal = None     # the last waypoint (for reached msg)

        # -- Pub / Sub --
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reached = self.create_publisher(PoseStamped, '/waypoint_reached', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)
        self.sub_path = self.create_subscription(Path, '/path', self._path_cb, 10)

        self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info('Navigator ready (single goal + path modes)')

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self.odom_received = True

    def _goal_cb(self, msg: PoseStamped):
        """Single waypoint — clear any existing path."""
        self.waypoints.clear()
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.waypoints.append((gx, gy))
        self.final_goal = (gx, gy)
        self.get_logger().info(f'Goal: ({gx:.2f}, {gy:.2f})')

    def _path_cb(self, msg: Path):
        """Multi-waypoint path from A* planner."""
        self.waypoints.clear()
        for pose in msg.poses:
            self.waypoints.append((pose.pose.position.x,
                                   pose.pose.position.y))
        if self.waypoints:
            self.final_goal = self.waypoints[-1]
            self.get_logger().info(
                f'Path received: {len(self.waypoints)} waypoints → '
                f'({self.final_goal[0]:.2f}, {self.final_goal[1]:.2f})'
            )

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self.waypoints or not self.odom_received:
            return

        gx, gy = self.waypoints[0]
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        # Choose tolerance: tighter for final goal, looser for intermediates
        is_final = len(self.waypoints) == 1
        tol = self.pos_tol if is_final else self.inter_tol

        # --- Reached current waypoint ---
        if dist < tol:
            self.waypoints.popleft()

            if not self.waypoints:
                # Final waypoint reached
                self.pub_cmd.publish(Twist())
                self.get_logger().info(
                    f'Reached final ({gx:.2f}, {gy:.2f})  '
                    f'[err {dist:.3f} m]'
                )
                reached = PoseStamped()
                reached.header.stamp = self.get_clock().now().to_msg()
                reached.header.frame_id = 'odom'
                reached.pose.position.x = self.final_goal[0]
                reached.pose.position.y = self.final_goal[1]
                self.pub_reached.publish(reached)
                self.final_goal = None
            return

        # --- Drive toward current waypoint ---
        desired_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw),
        )

        cmd = Twist()

        if abs(yaw_err) > self.ang_tol:
            cmd.angular.z = max(-self.max_ang,
                                min(self.max_ang, self.kp_ang * yaw_err))
        else:
            cmd.linear.x = min(self.max_lin, self.kp_lin * dist)
            cmd.angular.z = max(-self.max_ang,
                                min(self.max_ang, self.kp_ang * yaw_err))

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
