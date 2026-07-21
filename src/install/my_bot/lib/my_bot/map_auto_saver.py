#!/usr/bin/env python3
"""Periodically save the SLAM map to disk.

Calls slam_toolbox's /slam_toolbox/save_map service every N seconds so the
current map is persisted while mapping. On relaunch, localization can load
the same file.
"""
import os
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String


class MapAutoSaver(Node):
    def __init__(self):
        super().__init__('map_auto_saver')
        self.declare_parameter('map_name', os.path.expanduser('~/dev_ws/src/my_bot/maps/my_map'))
        self.declare_parameter('save_period_sec', 30.0)
        self.declare_parameter('initial_delay_sec', 45.0)
        self.map_name = self.get_parameter('map_name').value
        period = float(self.get_parameter('save_period_sec').value)
        initial_delay = float(self.get_parameter('initial_delay_sec').value)

        self._map_ready = False
        self._save_period = period

        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        self.get_logger().info('Waiting for /slam_toolbox/save_map service...')
        self.cli.wait_for_service()
        self.get_logger().info(
            f'Service available. Will start auto-saving to {self.map_name} '
            f'after {initial_delay}s initial delay, then every {period}s'
        )

        # Subscribe to /map to detect when slam_toolbox has published a valid map
        from nav_msgs.msg import OccupancyGrid
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, 1
        )

        # Initial delay timer — fires once to allow SLAM to build up scan data
        self._init_timer = self.create_timer(initial_delay, self._start_periodic_saves)

    def _on_map(self, msg):
        """Track whether slam_toolbox has published at least one valid map."""
        if not self._map_ready:
            self._map_ready = True
            self.get_logger().info('Map topic is live — auto-saver will begin saving shortly.')

    def _start_periodic_saves(self):
        """Called once after the initial delay. Starts the periodic save timer."""
        self._init_timer.cancel()
        if not self._map_ready:
            self.get_logger().warn(
                'Initial delay elapsed but /map not yet published. '
                'Will retry in 15s...'
            )
            self._init_timer = self.create_timer(15.0, self._start_periodic_saves)
            return
        self.get_logger().info(
            f'Starting periodic map saves every {self._save_period}s to {self.map_name}'
        )
        self._save_timer = self.create_timer(self._save_period, self._save)
        # Save immediately on first activation
        self._save()

    def _save(self):
        if not self._map_ready:
            self.get_logger().warn('Map not ready yet — skipping save.')
            return
        req = SaveMap.Request()
        name = String()
        name.data = self.map_name
        req.name = name
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_saved)

    def _on_saved(self, future):
        try:
            res = future.result()
            if res.result == 0:
                self.get_logger().info(f'Map saved successfully to {self.map_name}')
            else:
                self.get_logger().warn(
                    f'Map save returned code {res.result} — map may not be ready yet.'
                )
        except Exception as e:
            self.get_logger().warn(f'Save failed: {e}')


def main():
    rclpy.init()
    node = MapAutoSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
