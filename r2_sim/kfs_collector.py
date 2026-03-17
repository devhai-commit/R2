"""
KFS Collector — autonomous Kung Fu Scroll collection state machine.

Reads the KFS layout YAML (ground truth), plans an optimal collection order
using nearest-neighbour heuristic, drives to each R2 scroll via the navigator
node, simulates a timed pick, then returns to the Arena for placement.

States:  IDLE → NAVIGATING → COLLECTING → (loop) → PLACING → DONE
"""

import math
from enum import Enum, auto
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# ── Field geometry (mirrors apply_kfs_layout.py) ────────────────────────────
_X = {
    'lf': {'c1': -4.2, 'c2': -3.0, 'c3': -1.8},
    'rf': {'c1':  1.8, 'c2':  3.0, 'c3':  4.2},
}
_Y = {'r1': 1.8, 'r2': 0.6, 'r3': -0.6, 'r4': -1.8}

ROWS = ['r1', 'r2', 'r3', 'r4']
COLS = ['c1', 'c2', 'c3']
FOREST_KEYS = [('left_forest', 'lf'), ('right_forest', 'rf')]

# Arena Tic-Tac-Toe rack — approximate approach positions (middle row, 3 cols)
# R2 places scrolls in the middle row (40 pts each).
ARENA_SLOTS = [
    {'name': 'arena_mid_left',  'x': -1.8, 'y': -4.5},
    {'name': 'arena_mid_centre', 'x':  0.0, 'y': -4.5},
    {'name': 'arena_mid_right', 'x':  1.8, 'y': -4.5},
]


class State(Enum):
    IDLE = auto()
    NAVIGATING_TO_KFS = auto()
    COLLECTING = auto()
    NAVIGATING_TO_ARENA = auto()
    PLACING = auto()
    DONE = auto()


class KFSCollector(Node):

    def __init__(self):
        super().__init__('kfs_collector')

        # -- Parameters --
        self.declare_parameter('config_path', '')
        self.declare_parameter('team', 'r2')            # KFS label to collect
        self.declare_parameter('collect_duration', 2.0)  # seconds per pick
        self.declare_parameter('place_duration', 2.0)    # seconds per place
        # Robot start pose in world frame (for world→odom transform).
        # Diff-drive odom starts at (0, 0, yaw=0); we must convert world
        # coordinates into the odom frame before sending navigation goals.
        self.declare_parameter('start_x', -1.34)
        self.declare_parameter('start_y',  5.54)
        self.declare_parameter('start_yaw', math.pi)     # π — facing −x

        config_path = self.get_parameter('config_path').value
        self.team = self.get_parameter('team').value
        self.collect_dur = self.get_parameter('collect_duration').value
        self.place_dur = self.get_parameter('place_duration').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value

        # -- Pub / Sub --
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_status = self.create_publisher(String, '/kfs_status', 10)
        self.sub_reached = self.create_subscription(
            PoseStamped, '/waypoint_reached', self._reached_cb, 10,
        )

        # -- Load targets --
        self.targets = []
        if config_path:
            self.targets = self._load_targets(config_path)
        else:
            self.get_logger().warn('No config_path set — collector idle')

        # -- State --
        self.state = State.IDLE
        self.kfs_idx = 0          # index into self.targets
        self.arena_idx = 0        # index into ARENA_SLOTS
        self.collected = []       # names of collected KFS
        self.action_start = None  # timestamp for timed actions

        # 10 Hz tick
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'KFS Collector ready  |  team={self.team}  '
            f'targets={len(self.targets)}'
        )
        for t in self.targets:
            self.get_logger().info(f'  → {t["name"]}  ({t["x"]:.1f}, {t["y"]:.1f})')

    # ── Load & plan ──────────────────────────────────────────────────────────
    def _load_targets(self, config_path: str):
        with open(config_path) as f:
            layout = yaml.safe_load(f) or {}

        targets = []
        for yaml_key, forest in FOREST_KEYS:
            forest_cfg = layout.get(yaml_key) or {}
            for row in ROWS:
                row_cfg = forest_cfg.get(row)
                if not row_cfg:
                    continue
                for i, col in enumerate(COLS):
                    raw = row_cfg[i] if i < len(row_cfg) else None
                    if raw is None:
                        continue
                    label = str(raw).strip().lower()
                    if label in ('none', 'null', '~', ''):
                        continue
                    if label == self.team:
                        targets.append({
                            'name': f'kfs_{forest}_{row}{col}',
                            'x': _X[forest][col],
                            'y': _Y[row],
                        })

        return self._nearest_neighbour_order(targets, start_x=self.start_x,
                                                     start_y=self.start_y)

    @staticmethod
    def _nearest_neighbour_order(targets, start_x, start_y):
        remaining = list(targets)
        ordered = []
        cx, cy = start_x, start_y
        while remaining:
            nearest = min(
                remaining,
                key=lambda t: math.hypot(t['x'] - cx, t['y'] - cy),
            )
            ordered.append(nearest)
            cx, cy = nearest['x'], nearest['y']
            remaining.remove(nearest)
        return ordered

    # ── Callbacks ────────────────────────────────────────────────────────────
    def _reached_cb(self, _msg: PoseStamped):
        if self.state == State.NAVIGATING_TO_KFS:
            self.state = State.COLLECTING
            self.action_start = self.get_clock().now()
            name = self.targets[self.kfs_idx]['name']
            self.get_logger().info(f'Arrived at {name} — collecting …')
            self._pub(f'COLLECTING {name}')

        elif self.state == State.NAVIGATING_TO_ARENA:
            self.state = State.PLACING
            self.action_start = self.get_clock().now()
            slot = ARENA_SLOTS[self.arena_idx]['name']
            self.get_logger().info(f'At arena slot {slot} — placing …')
            self._pub(f'PLACING {slot}')

    # ── Main tick ────────────────────────────────────────────────────────────
    def _tick(self):
        if self.state == State.IDLE and self.targets:
            self._navigate_to_kfs()

        elif self.state == State.COLLECTING:
            elapsed = self._elapsed_secs()
            if elapsed >= self.collect_dur:
                target = self.targets[self.kfs_idx]
                self.collected.append(target['name'])
                self.get_logger().info(
                    f'Collected {target["name"]}  '
                    f'({len(self.collected)}/{len(self.targets)})'
                )
                self._pub(f'COLLECTED {target["name"]}')
                self.kfs_idx += 1

                if self.kfs_idx < len(self.targets):
                    self._navigate_to_kfs()
                else:
                    self.get_logger().info(
                        'All KFS collected — heading to arena'
                    )
                    self._navigate_to_arena()

        elif self.state == State.PLACING:
            elapsed = self._elapsed_secs()
            if elapsed >= self.place_dur:
                slot = ARENA_SLOTS[self.arena_idx]['name']
                self.get_logger().info(f'Placed scroll at {slot}')
                self._pub(f'PLACED {slot}')
                self.arena_idx += 1

                if self.arena_idx < min(len(self.collected), len(ARENA_SLOTS)):
                    self._navigate_to_arena()
                else:
                    self.state = State.DONE
                    self.get_logger().info(
                        f'DONE — collected {len(self.collected)} scrolls, '
                        f'placed {self.arena_idx}'
                    )
                    self._pub('DONE')

    # ── Helpers ──────────────────────────────────────────────────────────────
    def _navigate_to_kfs(self):
        target = self.targets[self.kfs_idx]
        self.state = State.NAVIGATING_TO_KFS
        self._send_goal(target['x'], target['y'])
        self._pub(f'NAV_TO_KFS {target["name"]}')

    def _navigate_to_arena(self):
        slot = ARENA_SLOTS[self.arena_idx]
        self.state = State.NAVIGATING_TO_ARENA
        self._send_goal(slot['x'], slot['y'])
        self._pub(f'NAV_TO_ARENA {slot["name"]}')

    def _world_to_odom(self, wx: float, wy: float):
        """Transform world-frame (x, y) into the odom frame.

        The diff-drive odom starts at (0, 0, yaw=0).  The odom frame origin
        sits at the robot's world start position, rotated by start_yaw.
        """
        dx = wx - self.start_x
        dy = wy - self.start_y
        c = math.cos(-self.start_yaw)
        s = math.sin(-self.start_yaw)
        return c * dx - s * dy, s * dx + c * dy

    def _send_goal(self, wx: float, wy: float):
        ox, oy = self._world_to_odom(wx, wy)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = ox
        msg.pose.position.y = oy
        self.pub_goal.publish(msg)
        self.get_logger().info(
            f'Goal  world({wx:.2f}, {wy:.2f}) → odom({ox:.2f}, {oy:.2f})'
        )

    def _pub(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)

    def _elapsed_secs(self) -> float:
        return (self.get_clock().now() - self.action_start).nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = KFSCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
