#!/usr/bin/env python3
"""Simple obstacle-avoidance node driven by /scan.

Behavior:
  - Drives forward at a steady linear speed.
  - When an obstacle appears in the forward arc within OBSTACLE_THRESHOLD,
    stops linear motion and rotates in place away from the nearest side.
  - Resumes forward motion once the arc is clear.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


LINEAR_SPEED = 0.2          # m/s forward
ANGULAR_SPEED = 0.6         # rad/s in-place turn
OBSTACLE_THRESHOLD = 0.6    # m — stop & turn when anything closer than this
FORWARD_ARC_DEG = 60        # total forward cone width (±30° from heading)


class LidarAvoider(Node):
    def __init__(self):
        super().__init__('lidar_avoider')

        # Gazebo's gpu_lidar bridge publishes /scan with BEST_EFFORT reliability.
        # Match it or we'll get zero messages.
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._scan_cb, scan_qos)

        # Latest decision, republished at 10 Hz so cmd_vel doesn't stall
        # if scans slow down.
        self._latest_cmd = Twist()
        self.create_timer(0.1, self._publish_cmd)

        self.get_logger().info(
            f'lidar_avoider up: forward={LINEAR_SPEED} m/s, '
            f'turn={ANGULAR_SPEED} rad/s, threshold={OBSTACLE_THRESHOLD} m')

    def _publish_cmd(self):
        self.cmd_pub.publish(self._latest_cmd)

    def _scan_cb(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # The lidar scans -pi..pi with 360 samples, so index 0 is dead ahead
        # (angle_min = -pi wraps around; we handle it by using angle_min/increment).
        half_arc = math.radians(FORWARD_ARC_DEG / 2.0)

        # Split forward arc into left half (0..+half_arc) and right half
        # (-half_arc..0) so we can decide which way to turn.
        left_min = math.inf
        right_min = math.inf

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r <= msg.range_min or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if -half_arc <= angle <= 0.0:
                if r < right_min:
                    right_min = r
            elif 0.0 < angle <= half_arc:
                if r < left_min:
                    left_min = r

        forward_min = min(left_min, right_min)

        cmd = Twist()
        if forward_min < OBSTACLE_THRESHOLD:
            # Obstacle ahead — turn toward the side with more room.
            cmd.linear.x = 0.0
            if left_min > right_min:
                cmd.angular.z = ANGULAR_SPEED       # turn left (CCW)
            else:
                cmd.angular.z = -ANGULAR_SPEED      # turn right (CW)
            self.get_logger().info(
                f'obstacle at {forward_min:.2f} m — turning '
                f'{"left" if cmd.angular.z > 0 else "right"}',
                throttle_duration_sec=1.0)
        else:
            cmd.linear.x = LINEAR_SPEED
            cmd.angular.z = 0.0
            self.get_logger().info(
                f'clear ({forward_min:.2f} m) — forward',
                throttle_duration_sec=2.0)

        self._latest_cmd = cmd


def main():
    rclpy.init()
    node = LidarAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot on shutdown.
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
