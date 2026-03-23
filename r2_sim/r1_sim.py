"""
R1 Keyboard Teleop — manual control of R1 in Gazebo.

Drive R1 with keyboard and publish status signals that R2 listens for.

Controls:
  Movement:
    W / ↑    — forward
    S / ↓    — backward
    A / ←    — turn left
    D / →    — turn right
    Q        — forward + turn left
    E        — forward + turn right
    SPACE    — stop

  Status signals (R2 coordination):
    1  — publish WAITING_FOR_R2  (R1 ready at staff rack)
    2  — publish ASSEMBLING      (assembly in progress)
    3  — publish DONE            (R1 finished, R2 may enter forest)

  ESC / Ctrl+C — quit

Publishes:
  /r1/cmd_vel    (Twist)   — velocity commands
  /r1_status     (String)  — state name for R2 coordination
"""

import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Speed settings
LIN_SPEED  = 0.5   # m/s
ANG_SPEED  = 1.5   # rad/s
LIN_STEP   = 0.05
ANG_STEP   = 0.15

KEY_BINDINGS = {
    'w':  ( 1,  0),
    's':  (-1,  0),
    'a':  ( 0,  1),
    'd':  ( 0, -1),
    'q':  ( 1,  1),
    'e':  ( 1, -1),
    ' ':  ( 0,  0),  # stop
}

STATUS_KEYS = {
    '1': 'WAITING_FOR_R2',
    '2': 'ASSEMBLING',
    '3': 'DONE',
}

BANNER = """
╔═══════════════════════════════════════════════╗
║          R1 Keyboard Teleop                   ║
╠═══════════════════════════════════════════════╣
║  Movement:                                    ║
║    W — forward    S — backward                ║
║    A — turn left  D — turn right              ║
║    Q — fwd+left   E — fwd+right               ║
║    SPACE — stop                               ║
║    +/- — increase/decrease linear speed       ║
║    [/] — increase/decrease angular speed      ║
║                                               ║
║  R2 coordination signals:                     ║
║    1 — WAITING_FOR_R2 (ready at staff rack)   ║
║    2 — ASSEMBLING     (weapon assembly)       ║
║    3 — DONE           (R2 may enter forest)   ║
║                                               ║
║  ESC — quit                                   ║
╚═══════════════════════════════════════════════╝
"""


def _get_key(settings):
    """Read a single keypress (non-blocking-ish)."""
    tty.setraw(sys.stdin.fileno())
    try:
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class R1Teleop(Node):

    def __init__(self):
        super().__init__('r1_sim')

        self.pub_cmd = self.create_publisher(Twist, '/r1/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/r1_status', 10)

        self.lin_speed = LIN_SPEED
        self.ang_speed = ANG_SPEED
        self.current_status = 'IDLE'

        self.get_logger().info('R1 keyboard teleop started')

    def publish_cmd(self, lin_x: float, ang_z: float):
        msg = Twist()
        msg.linear.x = lin_x
        msg.angular.z = ang_z
        self.pub_cmd.publish(msg)

    def publish_status(self, status: str):
        self.current_status = status
        msg = String()
        msg.data = status
        self.pub_status.publish(msg)
        self.get_logger().info(f'Published R1 status: {status}')

    def stop(self):
        self.publish_cmd(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = R1Teleop()

    settings = termios.tcgetattr(sys.stdin)

    print(BANNER)
    print(f'  Linear speed:  {node.lin_speed:.2f} m/s')
    print(f'  Angular speed: {node.ang_speed:.2f} rad/s')
    print()

    try:
        while rclpy.ok():
            key = _get_key(settings)

            if key == '\x1b':  # ESC
                break
            elif key == '\x03':  # Ctrl+C
                break

            if key.lower() in KEY_BINDINGS:
                lin_dir, ang_dir = KEY_BINDINGS[key.lower()]
                lin = lin_dir * node.lin_speed
                ang = ang_dir * node.ang_speed
                node.publish_cmd(lin, ang)
                if lin_dir == 0 and ang_dir == 0:
                    print('\r  STOP                        ', end='', flush=True)
                else:
                    print(f'\r  lin={lin:+.2f}  ang={ang:+.2f}      ',
                          end='', flush=True)

            elif key in STATUS_KEYS:
                status = STATUS_KEYS[key]
                node.publish_status(status)
                print(f'\r  → R1 status: {status}            ',
                      end='', flush=True)

            elif key == '+' or key == '=':
                node.lin_speed = min(node.lin_speed + LIN_STEP, 2.0)
                print(f'\r  Linear speed: {node.lin_speed:.2f}      ',
                      end='', flush=True)

            elif key == '-':
                node.lin_speed = max(node.lin_speed - LIN_STEP, 0.05)
                print(f'\r  Linear speed: {node.lin_speed:.2f}      ',
                      end='', flush=True)

            elif key == ']':
                node.ang_speed = min(node.ang_speed + ANG_STEP, 4.0)
                print(f'\r  Angular speed: {node.ang_speed:.2f}     ',
                      end='', flush=True)

            elif key == '[':
                node.ang_speed = max(node.ang_speed - ANG_STEP, 0.15)
                print(f'\r  Angular speed: {node.ang_speed:.2f}     ',
                      end='', flush=True)

            # Spin once to handle callbacks
            rclpy.spin_once(node, timeout_sec=0.01)

    except Exception as e:
        print(f'\nError: {e}')
    finally:
        node.stop()
        print('\nR1 teleop stopped.')
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
