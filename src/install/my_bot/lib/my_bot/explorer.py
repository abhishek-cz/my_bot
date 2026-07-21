#!/usr/bin/env python3
"""Autonomous frontier-based explorer with reverse recovery.

Drives the robot around the environment so slam_toolbox can map the whole
world. Runs alongside the reactive lidar_avoider — this node only issues
high-level heading changes when it detects the robot is stuck or has been
in the same area too long. lidar_avoider still handles moment-to-moment
collision avoidance on /cmd_vel.

Strategy:
  - Subscribe to /map (from slam_toolbox), /odom, and /scan.
  - Track how long the robot has stayed within a small radius.
  - If stuck for STUCK_TIMEOUT seconds:
      1. Check side clearance from the laser scan.
      2. If clearance is insufficient to turn, reverse for REVERSE_DURATION
         seconds. Repeat up to MAX_REVERSE_BURSTS times until the robot
         reaches a safe distance where left/right turns are possible.
      3. Once clearance is ok, rotate toward the nearest unexplored frontier
         cell (or spin randomly if no map is available yet).
  - If no unknown cells remain within the current map bounds, the world
    is fully explored — publish a status log and idle.
"""

import math
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan


# ── Stuck detection ──────────────────────────────────────────────────────────
STUCK_RADIUS   = 0.4    # m  — robot must move farther than this to be "not stuck"
STUCK_TIMEOUT  = 8.0    # s  — time within STUCK_RADIUS before triggering recovery

# ── Nudge (rotation) recovery ────────────────────────────────────────────────
NUDGE_ANGULAR  = 0.8    # rad/s — rotation speed during a nudge
NUDGE_DURATION = 2.0    # s     — how long each nudge lasts

# ── Reverse recovery ─────────────────────────────────────────────────────────
REVERSE_SPEED      = -0.15  # m/s  — negative = backward
REVERSE_DURATION   = 3.0    # s    — duration of each reverse burst
MAX_REVERSE_BURSTS = 5      # max consecutive reverse bursts before giving up
REVERSE_SAFE_DIST  = 0.6    # m    — minimum side clearance before turning is safe
SCAN_SIDE_ANGLE    = 0.7    # rad  — half-cone on each side used for clearance check

# ── Map cell values ───────────────────────────────────────────────────────────
UNKNOWN = -1
FREE    =  0


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')

        # /map from slam_toolbox is latched (TRANSIENT_LOCAL).
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, map_qos)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State
        self._map   = None
        self._pose  = None   # (x, y, yaw)
        self._scan  = None   # latest LaserScan

        self._anchor          = None   # (x, y, t) — reference position for stuck check
        self._nudge_until     = 0.0    # wall-clock time when current nudge ends
        self._nudge_dir       = 1.0    # +1 = left, -1 = right
        self._reversing_until = 0.0    # wall-clock time when current reverse burst ends
        self._reverse_count   = 0      # consecutive reverse bursts issued

        # 5 Hz tick so reverse commands are issued promptly
        self.create_timer(0.2, self._tick)
        self.get_logger().info(
            'explorer started — forward/left/right/reverse recovery active')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self._map = msg

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny  = 2.0 * (q.w * q.z + q.x * q.y)
        cosy  = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw   = math.atan2(siny, cosy)
        self._pose = (p.x, p.y, yaw)

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg


    # ── Main loop ────────────────────────────────────────────────────────────

    def _tick(self):
        if self._pose is None:
            return

        now = self.get_clock().now().nanoseconds * 1e-9

        # ── Phase 1: actively reversing ──────────────────────────────────────
        if now < self._reversing_until:
            cmd = Twist()
            cmd.linear.x = REVERSE_SPEED
            self.cmd_pub.publish(cmd)
            return

        # ── Phase 2: reverse burst just finished — check if we have room now ─
        if self._reverse_count > 0:
            if self._side_clearance_ok():
                # Enough room to turn — reset stuck state and let nudge proceed
                x, y, _ = self._pose
                self._reverse_count = 0
                self._anchor = (x, y, now)
                self.get_logger().info(
                    'reverse recovery succeeded — side clearance restored',
                    throttle_duration_sec=5.0)
            elif self._reverse_count < MAX_REVERSE_BURSTS:
                # Still not enough room — issue another reverse burst
                x, y, _ = self._pose
                self._reverse_count += 1
                self._reversing_until = now + REVERSE_DURATION
                self._anchor = (x, y, now + REVERSE_DURATION)
                self.get_logger().info(
                    f'still confined — reverse burst {self._reverse_count}/'  
                    f'{MAX_REVERSE_BURSTS}',
                    throttle_duration_sec=2.0)
                cmd = Twist()
                cmd.linear.x = REVERSE_SPEED
                self.cmd_pub.publish(cmd)
                return
            else:
                # Exhausted reverse bursts — fall through to spin nudge as last resort
                self._reverse_count = 0
                self.get_logger().warn(
                    'max reverse bursts reached — falling back to spin nudge',
                    throttle_duration_sec=5.0)

        # ── Phase 3: actively nudging (rotating) ─────────────────────────────
        if now < self._nudge_until:
            cmd = Twist()
            cmd.angular.z = NUDGE_ANGULAR * self._nudge_dir
            self.cmd_pub.publish(cmd)
            return

        # ── Phase 4: normal driving — check if stuck ─────────────────────────
        x, y, _ = self._pose
        if self._anchor is None:
            self._anchor = (x, y, now)
            return

        ax, ay, at = self._anchor
        dist = math.hypot(x - ax, y - ay)

        if dist > STUCK_RADIUS:
            # Making progress — refresh anchor
            self._anchor = (x, y, now)
            return

        if (now - at) < STUCK_TIMEOUT:
            return

        # ── Robot is stuck ───────────────────────────────────────────────────
        if not self._side_clearance_ok():
            # Not enough room to turn — start reverse recovery
            self._reverse_count = 1
            self._reversing_until = now + REVERSE_DURATION
            self._anchor = (x, y, now + REVERSE_DURATION)
            self.get_logger().info(
                'stuck with insufficient side clearance — starting reverse recovery',
                throttle_duration_sec=5.0)
            cmd = Twist()
            cmd.linear.x = REVERSE_SPEED
            self.cmd_pub.publish(cmd)
            return

        # Side clearance is fine — rotate toward unexplored frontier
        direction = self._pick_frontier_direction()
        if direction is None:
            direction = random.choice([-1.0, 1.0])
            self.get_logger().info(
                'stuck — no map yet, spinning to look around',
                throttle_duration_sec=5.0)
        else:
            self.get_logger().info(
                'stuck — turning toward unexplored frontier',
                throttle_duration_sec=5.0)

        self._nudge_dir   = direction
        self._nudge_until = now + NUDGE_DURATION
        self._anchor      = (x, y, now + NUDGE_DURATION)

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _side_clearance_ok(self) -> bool:
        """Return True if both left and right sides have >= REVERSE_SAFE_DIST
        clearance, meaning the robot has room to execute a turn.

        Checks the angular window [-SCAN_SIDE_ANGLE, -0.1] (right side) and
        [0.1, SCAN_SIDE_ANGLE] (left side). Returns True when no scan is
        available (fail-open so the robot doesn't get stuck waiting for a scan).
        """
        if self._scan is None:
            return True

        scan      = self._scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges    = scan.ranges

        right_ranges: list[float] = []
        left_ranges:  list[float] = []

        for i, r in enumerate(ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue
            angle = angle_min + i * angle_inc
            if -SCAN_SIDE_ANGLE <= angle <= -0.1:
                right_ranges.append(r)
            elif 0.1 <= angle <= SCAN_SIDE_ANGLE:
                left_ranges.append(r)

        right_ok = (min(right_ranges) >= REVERSE_SAFE_DIST) if right_ranges else True
        left_ok  = (min(left_ranges)  >= REVERSE_SAFE_DIST) if left_ranges  else True

        return right_ok and left_ok

    def _pick_frontier_direction(self):
        """Return +1.0 (turn left) or -1.0 (turn right) toward the nearest
        unexplored frontier cell, or None if no map is available.

        A frontier cell is an UNKNOWN cell adjacent to at least one FREE cell.
        """
        if self._map is None or self._pose is None:
            return None

        m  = self._map
        w, h = m.info.width, m.info.height
        res  = m.info.resolution
        ox   = m.info.origin.position.x
        oy   = m.info.origin.position.y
        data = m.data

        rx, ry, ryaw = self._pose
        rcx = int((rx - ox) / res)
        rcy = int((ry - oy) / res)

        best    = None
        best_d2 = float('inf')

        # Sparse sampling to keep this cheap at 5 Hz
        step = max(1, min(w, h) // 60)
        for cy in range(0, h, step):
            row = cy * w
            for cx in range(0, w, step):
                if data[row + cx] != UNKNOWN:
                    continue
                if not self._has_free_neighbor(data, cx, cy, w, h):
                    continue
                d2 = (cx - rcx) ** 2 + (cy - rcy) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = (cx, cy)

        if best is None:
            self.get_logger().info(
                'no unexplored frontier cells found — map may be complete',
                throttle_duration_sec=10.0)
            return None

        fx = ox + best[0] * res
        fy = oy + best[1] * res
        bearing = math.atan2(fy - ry, fx - rx)
        err     = math.atan2(math.sin(bearing - ryaw), math.cos(bearing - ryaw))
        return 1.0 if err > 0 else -1.0

    @staticmethod
    def _has_free_neighbor(data, cx: int, cy: int, w: int, h: int) -> bool:
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h:
                if data[ny * w + nx] == FREE:
                    return True
        return False


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
