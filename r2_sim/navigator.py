"""
Waypoint navigator node.

Subscribes to /goal_pose, drives the robot toward the target using a simple
proportional controller on /cmd_vel, and publishes /waypoint_reached on arrival.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


def _yaw_from_quat(q):
    """Extract yaw from a quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class Navigator(Node):

    def __init__(self):
        super().__init__('navigator')

        # -- Parameters --
        self.declare_parameter('pos_tolerance', 0.20)    # metres
        self.declare_parameter('ang_tolerance', 0.15)     # radians
        self.declare_parameter('max_linear', 0.5)         # m/s
        self.declare_parameter('max_angular', 1.5)        # rad/s
        self.declare_parameter('kp_linear', 0.8)
        self.declare_parameter('kp_angular', 2.0)

        self.pos_tol = self.get_parameter('pos_tolerance').value
        self.ang_tol = self.get_parameter('ang_tolerance').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.kp_lin = self.get_parameter('kp_linear').value
        self.kp_ang = self.get_parameter('kp_angular').value

        # -- State --
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.goal = None          # (x, y) or None
        self.odom_received = False

        # -- Pub / Sub --
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_reached = self.create_publisher(PoseStamped, '/waypoint_reached', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self._goal_cb, 10)

        # 20 Hz control loop
        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Navigator ready')

    # --------------------------------------------------------------------- #
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self.odom_received = True

    def _goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Goal received: ({self.goal[0]:.2f}, {self.goal[1]:.2f})')

    # --------------------------------------------------------------------- #
    def _control_loop(self):
        if self.goal is None or not self.odom_received:
            return

        gx, gy = self.goal
        dx = gx - self.x
        dy = gy - self.y
        dist = math.hypot(dx, dy)

        # --- Arrived ---
        if dist < self.pos_tol:
            self.pub_cmd.publish(Twist())          # stop
            self.get_logger().info(
                f'Reached ({gx:.2f}, {gy:.2f})  '
                f'[pos err {dist:.3f} m]'
            )
            reached = PoseStamped()
            reached.header.stamp = self.get_clock().now().to_msg()
            reached.header.frame_id = 'odom'
            reached.pose.position.x = gx
            reached.pose.position.y = gy
            self.pub_reached.publish(reached)
            self.goal = None
            return

        # --- Compute heading error ---
        desired_yaw = math.atan2(dy, dx)
        yaw_err = math.atan2(
            math.sin(desired_yaw - self.yaw),
            math.cos(desired_yaw - self.yaw),
        )

        cmd = Twist()

        if abs(yaw_err) > self.ang_tol:
            # Rotate in place first
            cmd.angular.z = max(-self.max_ang,
                                min(self.max_ang, self.kp_ang * yaw_err))
        else:
            # Drive forward + minor angular correction
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
