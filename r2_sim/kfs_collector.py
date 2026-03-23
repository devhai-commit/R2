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

from r2_sim.astar_planner import FieldPlanner, ForestGraph


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

# Ground-level approach point just north of each forest's top row
# (the robot drives here on the ground, then steps onto a row-1 platform)
_FOREST_APPROACH_Y = 2.50   # just above the r1 row (y=1.8 + half platform + margin)

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

        # -- Planners --
        self.planner = FieldPlanner(resolution=0.05, robot_radius=0.25)
        self.forest_lf = ForestGraph('lf', max_step=0.20)
        self.forest_rf = ForestGraph('rf', max_step=0.20)
        self.get_logger().info('A* ground planner + forest graphs initialized')

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
        self.on_platform = None   # (forest, row, col) if currently on a platform

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'KFS Collector ready  |  team={self.team}  '
            f'KFS targets={len(self.targets)}'
        )
        for t in self.targets:
            self.get_logger().info(f'  → {t["name"]}  ({t["x"]:.1f}, {t["y"]:.1f})')

    # ── Load & plan KFS targets ──────────────────────────────────────────────
    def _load_targets(self, config_path: str):
        """Load KFS targets from YAML.

        Each target stores:
          - forest, row, col  — platform identity
          - x, y              — platform centre (world frame)
          - entry_row/col     — which r1 platform to enter through
          - approach_x/y      — ground point to drive to before stepping on
        """
        with open(config_path) as f:
            layout = yaml.safe_load(f) or {}

        targets = []
        for yaml_key, forest in FOREST_KEYS:
            forest_cfg = layout.get(yaml_key) or {}
            fg = self.forest_lf if forest == 'lf' else self.forest_rf

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

                        # Find best entry platform (row r1) to reach this KFS
                        entry = fg.nearest_entry(row, col)
                        if entry is None:
                            continue
                        entry_x, _ = fg.world_pos(*entry)

                        targets.append({
                            'name': f'kfs_{forest}_{row}{col}',
                            'forest': forest,
                            'row': row,
                            'col': col,
                            'x': kx,
                            'y': ky,
                            'entry': entry,
                            'approach_x': entry_x,
                            'approach_y': _FOREST_APPROACH_Y,
                        })

        # Order from the Meihua Forest entrance
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
                self.get_logger().info(
                    'Weapon assembled! R1 enters forest first — waiting'
                )
                self._set_state(State.WAIT_R1_EXIT)
                self._pub('WAIT_R1_EXIT')

        elif self.state == State.WAIT_R1_EXIT:
            if self.r1_state == 'DONE':
                self.get_logger().info(
                    '=== Phase 2: Meihua Forest ===  '
                    'R1 finished in forest — R2 entering'
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
        """Navigate to a KFS target.

        If on the ground: drive to approach point → step onto entry platform
        → hop to target.
        If already on a platform in the same forest: hop directly.
        If switching forests: exit current → ground → enter new.
        """
        target = self.targets[self.kfs_idx]
        fg = self.forest_lf if target['forest'] == 'lf' else self.forest_rf

        self._set_state(State.NAV_TO_KFS)
        self._pub(f'NAV_TO_KFS {target["name"]}')

        all_waypoints = []

        if (self.on_platform is not None
                and self.on_platform[0] == target['forest']):
            # Already on a platform in the same forest — hop directly
            _, cur_row, cur_col = self.on_platform
            platform_wps = fg.plan_world(
                cur_row, cur_col, target['row'], target['col'],
            )
            # Skip the first waypoint (current position)
            all_waypoints.extend(platform_wps[1:] if len(platform_wps) > 1
                                 else platform_wps)
        else:
            # Coming from the ground (or different forest)
            if self.on_platform is not None:
                # Exit current forest first: hop to entry, step off
                all_waypoints.extend(self._exit_forest_waypoints())

            # Drive on ground to the approach point
            cur_wx, cur_wy = self._odom_to_world(self.odom_x, self.odom_y)
            ground_path = self.planner.plan(
                cur_wx, cur_wy,
                target['approach_x'], target['approach_y'],
            )
            if ground_path:
                all_waypoints.extend(ground_path)
            else:
                all_waypoints.append(
                    (target['approach_x'], target['approach_y'])
                )

            # Step onto entry platform → hop to target
            entry = target['entry']
            platform_wps = fg.plan_world(
                entry[0], entry[1], target['row'], target['col'],
            )
            all_waypoints.extend(platform_wps)

        self.on_platform = (target['forest'], target['row'], target['col'])
        self._send_path(all_waypoints)
        self.get_logger().info(
            f'KFS path to {target["name"]}: {len(all_waypoints)} waypoints'
        )

    def _exit_forest_waypoints(self) -> list:
        """Compute waypoints to exit the current forest back to the ground."""
        if self.on_platform is None:
            return []
        forest, cur_row, cur_col = self.on_platform
        fg = self.forest_lf if forest == 'lf' else self.forest_rf

        # Hop to the nearest entry platform (row r1)
        entry = fg.nearest_entry(cur_row, cur_col)
        if entry is None:
            return []

        wps = fg.plan_world(cur_row, cur_col, entry[0], entry[1])
        # Skip current position
        exit_wps = wps[1:] if len(wps) > 1 else []

        # Step off to ground approach point
        entry_x, _ = fg.world_pos(*entry)
        exit_wps.append((entry_x, _FOREST_APPROACH_Y))

        self.on_platform = None
        return exit_wps

    def _navigate_to_arena(self):
        slot = ARENA_SLOTS[self.arena_idx]
        self._set_state(State.NAV_TO_ARENA)
        self._pub(f'NAV_TO_ARENA {slot["name"]}')

        all_waypoints = []

        # Exit forest if still on a platform
        if self.on_platform is not None:
            all_waypoints.extend(self._exit_forest_waypoints())

        # Ground path to the arena slot
        # Use the last exit point or current odom as start
        if all_waypoints:
            start_x, start_y = all_waypoints[-1]
        else:
            start_x, start_y = self._odom_to_world(self.odom_x, self.odom_y)

        ground_path = self.planner.plan(start_x, start_y, slot['x'], slot['y'])
        if ground_path:
            all_waypoints.extend(ground_path)
        else:
            all_waypoints.append((slot['x'], slot['y']))

        self._send_path(all_waypoints)
        self.get_logger().info(
            f'Arena path to {slot["name"]}: {len(all_waypoints)} waypoints'
        )

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
        """Plan an A* ground path from current position to (wx, wy)."""
        cur_wx, cur_wy = self._odom_to_world(self.odom_x, self.odom_y)
        world_path = self.planner.plan(cur_wx, cur_wy, wx, wy)

        if not world_path:
            self.get_logger().warn(
                f'A* found no path to ({wx:.2f}, {wy:.2f}) — sending direct goal'
            )
            ox, oy = self._world_to_odom(wx, wy)
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.pose.position.x = ox
            msg.pose.position.y = oy
            self.pub_goal.publish(msg)
            return

        self._send_path(world_path)
        self.get_logger().info(
            f'A* path to world({wx:.2f}, {wy:.2f}): '
            f'{len(world_path)} waypoints'
        )

    def _send_path(self, world_waypoints: list):
        """Publish a list of (wx, wy) world-frame waypoints as a Path msg."""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for pwx, pwy in world_waypoints:
            ox, oy = self._world_to_odom(pwx, pwy)
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = ox
            pose.pose.position.y = oy
            path_msg.poses.append(pose)

        self.pub_path.publish(path_msg)

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
