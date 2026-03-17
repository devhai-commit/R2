"""
KFS Collector — full R2 autonomous match sequence.

Phase 1  Martial Club (weapon assembly):
  IDLE → NAV_TO_SPEARHEAD → PICKING_SPEARHEAD → NAV_TO_ASSEMBLY →
  WAITING_FOR_R1 → ASSEMBLING → WAIT_R1_EXIT

Phase 2  Meihua Forest (scroll collection):
  → NAV_TO_KFS → COLLECTING → (loop) →

Phase 3  Arena (scroll placement):
  → NAV_TO_ARENA → PLACING → (loop) → DONE
"""

import math
from enum import Enum, auto

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from r2_sim.astar_planner import FieldPlanner


# ── Field geometry ───────────────────────────────────────────────────────────
_X = {
    'lf': {'c1': -4.2, 'c2': -3.0, 'c3': -1.8},
    'rf': {'c1':  1.8, 'c2':  3.0, 'c3':  4.2},
}
_Y = {'r1': 1.8, 'r2': 0.6, 'r3': -0.6, 'r4': -1.8}

ROWS = ['r1', 'r2', 'r3', 'r4']
COLS = ['c1', 'c2', 'c3']
FOREST_KEYS = [('left_forest', 'lf'), ('right_forest', 'rf')]

# Key world-frame positions (left team)
SPEARHEAD_RACK  = (-0.30, 4.69)   # approach from the left side of center rack
STAFF_RACK_LEFT = (-3.44, 6.065)  # assembly happens here — R2 brings spearhead to R1
R2_ENTRANCE     = (-3.045, 4.165) # entrance to Meihua Forest

# Forest block edges (platforms are 1.2×1.2m, touching edge-to-edge)
# Left forest:  x ∈ [-4.8, -1.2],  y ∈ [-2.4, 2.4]
# Right forest: x ∈ [ 1.2,  4.8],  y ∈ [-2.4, 2.4]
# Robot approaches KFS from the nearest forest edge.
_LF_BOUNDS = {'x_min': -4.8, 'x_max': -1.2, 'y_min': -2.4, 'y_max': 2.4}
_RF_BOUNDS = {'x_min':  1.2, 'x_max':  4.8, 'y_min': -2.4, 'y_max': 2.4}
_APPROACH_OFFSET = 0.45   # metres outside the forest edge

ARENA_SLOTS = [
    {'name': 'arena_mid_left',   'x': -1.8, 'y': -4.5},
    {'name': 'arena_mid_centre', 'x':  0.0, 'y': -4.5},
    {'name': 'arena_mid_right',  'x':  1.8, 'y': -4.5},
]


class State(Enum):
    # Phase 1 — Martial Club
    IDLE                = auto()
    NAV_TO_SPEARHEAD    = auto()
    PICKING_SPEARHEAD   = auto()
    NAV_TO_ASSEMBLY     = auto()
    WAITING_FOR_R1      = auto()
    ASSEMBLING          = auto()
    WAIT_R1_EXIT        = auto()
    # Phase 2 — Meihua Forest
    NAV_TO_KFS          = auto()
    COLLECTING          = auto()
    # Phase 3 — Arena
    NAV_TO_ARENA        = auto()
    PLACING             = auto()
    DONE                = auto()


class KFSCollector(Node):

    def __init__(self):
        super().__init__('kfs_collector')

        # -- Parameters --
        self.declare_parameter('config_path', '')
        self.declare_parameter('team', 'r2')
        self.declare_parameter('collect_duration', 2.0)
        self.declare_parameter('place_duration', 2.0)
        self.declare_parameter('pick_spearhead_duration', 2.0)
        self.declare_parameter('assemble_duration', 3.0)
        self.declare_parameter('start_x', -1.34)
        self.declare_parameter('start_y',  5.54)
        self.declare_parameter('start_yaw', math.pi)

        config_path = self.get_parameter('config_path').value
        self.team = self.get_parameter('team').value
        self.collect_dur = self.get_parameter('collect_duration').value
        self.place_dur = self.get_parameter('place_duration').value
        self.spearhead_dur = self.get_parameter('pick_spearhead_duration').value
        self.assemble_dur = self.get_parameter('assemble_duration').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_yaw = self.get_parameter('start_yaw').value

        # -- A* planner --
        self.planner = FieldPlanner(resolution=0.05, robot_radius=0.25)
        self.get_logger().info('A* field planner initialized')

        # -- Pub / Sub --
        self.pub_path = self.create_publisher(Path, '/path', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pub_status = self.create_publisher(String, '/kfs_status', 10)
        self.sub_reached = self.create_subscription(
            PoseStamped, '/waypoint_reached', self._reached_cb, 10,
        )
        self.sub_r1 = self.create_subscription(
            String, '/r1_status', self._r1_status_cb, 10,
        )
        self.sub_odom = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10,
        )

        # -- Load KFS targets --
        self.targets = []
        if config_path:
            self.targets = self._load_targets(config_path)

        # -- State --
        self.state = State.IDLE
        self.kfs_idx = 0
        self.arena_idx = 0
        self.collected = []
        self.action_start = None
        self.r1_state = ''
        self.odom_x = 0.0
        self.odom_y = 0.0

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'KFS Collector ready  |  team={self.team}  '
            f'KFS targets={len(self.targets)}'
        )
        for t in self.targets:
            self.get_logger().info(f'  → {t["name"]}  ({t["x"]:.1f}, {t["y"]:.1f})')

    # ── Load & plan KFS targets ──────────────────────────────────────────────
    @staticmethod
    def _approach_point(kfs_x, kfs_y, forest):
        """Compute a reachable approach point on the nearest forest edge."""
        bounds = _LF_BOUNDS if forest == 'lf' else _RF_BOUNDS

        # Distances to each edge
        edges = {
            'left':   abs(kfs_x - bounds['x_min']),
            'right':  abs(kfs_x - bounds['x_max']),
            'top':    abs(kfs_y - bounds['y_max']),
            'bottom': abs(kfs_y - bounds['y_min']),
        }
        nearest = min(edges, key=edges.get)

        ax, ay = kfs_x, kfs_y
        if nearest == 'left':
            ax = bounds['x_min'] - _APPROACH_OFFSET
        elif nearest == 'right':
            ax = bounds['x_max'] + _APPROACH_OFFSET
        elif nearest == 'top':
            ay = bounds['y_max'] + _APPROACH_OFFSET
        elif nearest == 'bottom':
            ay = bounds['y_min'] - _APPROACH_OFFSET

        return ax, ay

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
                        kx = _X[forest][col]
                        ky = _Y[row]
                        ax, ay = self._approach_point(kx, ky, forest)
                        targets.append({
                            'name': f'kfs_{forest}_{row}{col}',
                            'x': ax,     # approach position (navigable)
                            'y': ay,
                            'kfs_x': kx, # actual KFS position (on platform)
                            'kfs_y': ky,
                        })

        # Order from the Meihua Forest entrance, not the start zone
        return self._nearest_neighbour_order(
            targets, start_x=R2_ENTRANCE[0], start_y=R2_ENTRANCE[1],
        )

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
    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

    def _r1_status_cb(self, msg: String):
        self.r1_state = msg.data

    def _reached_cb(self, _msg: PoseStamped):
        if self.state == State.NAV_TO_SPEARHEAD:
            self._begin_action(State.PICKING_SPEARHEAD, self.spearhead_dur)
            self.get_logger().info('At spearhead rack — picking …')

        elif self.state == State.NAV_TO_ASSEMBLY:
            self._set_state(State.WAITING_FOR_R1)
            self._pub('WAITING_FOR_R1')
            self.get_logger().info('At assembly point — waiting for R1')

        elif self.state == State.NAV_TO_KFS:
            self._begin_action(State.COLLECTING, self.collect_dur)
            name = self.targets[self.kfs_idx]['name']
            self.get_logger().info(f'Arrived at {name} — collecting …')
            self._pub(f'COLLECTING {name}')

        elif self.state == State.NAV_TO_ARENA:
            self._begin_action(State.PLACING, self.place_dur)
            slot = ARENA_SLOTS[self.arena_idx]['name']
            self.get_logger().info(f'At arena slot {slot} — placing …')
            self._pub(f'PLACING {slot}')

    # ── Main tick ────────────────────────────────────────────────────────────
    def _tick(self):

        # ─── Phase 1: Martial Club ───────────────────────────────────────
        if self.state == State.IDLE:
            self.get_logger().info('=== Phase 1: Martial Club ===')
            self._set_state(State.NAV_TO_SPEARHEAD)
            self._navigate_to(*SPEARHEAD_RACK)
            self._pub('NAV_TO_SPEARHEAD')

        elif self.state == State.PICKING_SPEARHEAD:
            if self._timer_done():
                self.get_logger().info('Spearhead picked — heading to staff rack for assembly')
                self._set_state(State.NAV_TO_ASSEMBLY)
                self._navigate_to(*STAFF_RACK_LEFT)
                self._pub('NAV_TO_ASSEMBLY')

        elif self.state == State.WAITING_FOR_R1:
            # Wait for R1 to be at assembly point (WAITING_FOR_R2 or ASSEMBLING)
            if self.r1_state in ('WAITING_FOR_R2', 'ASSEMBLING'):
                self._begin_action(State.ASSEMBLING, self.assemble_dur)
                self.get_logger().info('R1 ready — assembling weapon …')
                self._pub('ASSEMBLING')

        elif self.state == State.ASSEMBLING:
            if self._timer_done():
                self.get_logger().info('Weapon assembled! Waiting for R1 to exit')
                self._set_state(State.WAIT_R1_EXIT)
                self._pub('WAIT_R1_EXIT')

        elif self.state == State.WAIT_R1_EXIT:
            if self.r1_state == 'DONE':
                self.get_logger().info(
                    '=== Phase 2: Meihua Forest ===  R1 exited — entering forest'
                )
                if self.targets:
                    self._navigate_to_kfs()
                else:
                    self.get_logger().warn('No KFS targets — skipping to arena')
                    self._navigate_to_arena()

        # ─── Phase 2: Meihua Forest ─────────────────────────────────────
        elif self.state == State.COLLECTING:
            if self._timer_done():
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
                        '=== Phase 3: Arena ===  All KFS collected'
                    )
                    self._navigate_to_arena()

        # ─── Phase 3: Arena ─────────────────────────────────────────────
        elif self.state == State.PLACING:
            if self._timer_done():
                slot = ARENA_SLOTS[self.arena_idx]['name']
                self.get_logger().info(f'Placed scroll at {slot}')
                self._pub(f'PLACED {slot}')
                self.arena_idx += 1

                if self.arena_idx < min(len(self.collected), len(ARENA_SLOTS)):
                    self._navigate_to_arena()
                else:
                    self._set_state(State.DONE)
                    self.get_logger().info(
                        f'DONE — collected {len(self.collected)} scrolls, '
                        f'placed {self.arena_idx}'
                    )
                    self._pub('DONE')

    # ── Navigation helpers ───────────────────────────────────────────────────
    def _navigate_to_kfs(self):
        target = self.targets[self.kfs_idx]
        self._set_state(State.NAV_TO_KFS)
        self._navigate_to(target['x'], target['y'])
        self._pub(f'NAV_TO_KFS {target["name"]}')

    def _navigate_to_arena(self):
        slot = ARENA_SLOTS[self.arena_idx]
        self._set_state(State.NAV_TO_ARENA)
        self._navigate_to(slot['x'], slot['y'])
        self._pub(f'NAV_TO_ARENA {slot["name"]}')

    # ── Coordinate transform ─────────────────────────────────────────────────
    def _world_to_odom(self, wx: float, wy: float):
        """Transform world-frame (x, y) into the odom frame."""
        dx = wx - self.start_x
        dy = wy - self.start_y
        c = math.cos(-self.start_yaw)
        s = math.sin(-self.start_yaw)
        return c * dx - s * dy, s * dx + c * dy

    def _odom_to_world(self, ox: float, oy: float):
        """Transform odom-frame (x, y) back to world frame."""
        c = math.cos(self.start_yaw)
        s = math.sin(self.start_yaw)
        wx = c * ox - s * oy + self.start_x
        wy = s * ox + c * oy + self.start_y
        return wx, wy

    def _navigate_to(self, wx: float, wy: float):
        """Plan an A* path from current position to (wx, wy) in world frame."""
        # Get current world position from odom
        cur_wx, cur_wy = self._odom_to_world(self.odom_x, self.odom_y)

        # Run A* in world frame
        world_path = self.planner.plan(cur_wx, cur_wy, wx, wy)

        if not world_path:
            self.get_logger().warn(
                f'A* found no path to ({wx:.2f}, {wy:.2f}) — sending direct goal'
            )
            # Fallback: send single goal directly
            ox, oy = self._world_to_odom(wx, wy)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = ox
            msg.pose.position.y = oy
            self.pub_goal.publish(msg)
            return

        # Convert world path to odom frame and publish as Path
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for pwx, pwy in world_path:
            ox, oy = self._world_to_odom(pwx, pwy)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = ox
            pose.pose.position.y = oy
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)
        self.get_logger().info(
            f'A* path to world({wx:.2f}, {wy:.2f}): '
            f'{len(world_path)} waypoints'
        )

    # ── Utilities ────────────────────────────────────────────────────────────
    def _begin_action(self, state: State, duration: float):
        self.state = state
        self.action_start = self.get_clock().now()
        self._action_dur = duration

    def _set_state(self, state: State):
        self.state = state

    def _timer_done(self) -> bool:
        if self.action_start is None:
            return True
        elapsed = (self.get_clock().now() - self.action_start).nanoseconds / 1e9
        return elapsed >= self._action_dur

    def _pub(self, text: str):
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KFSCollector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
