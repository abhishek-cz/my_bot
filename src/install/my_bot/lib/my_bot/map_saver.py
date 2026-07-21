#!/usr/bin/env python3
"""
Periodic map saver node.

Saves the occupancy grid map (.yaml + .pgm) every 30 seconds and
serializes the SLAM Toolbox pose graph every 60 seconds.

The `map_name` ROS2 parameter (default: 'simple_world') controls which
subdirectory under /home/navbot/dev_ws/maps/ the files are written to,
so each world keeps its own persistent map.

Occupancy grid saving uses the standard nav2_map_server `map_saver_cli`
tool, invoked through a login shell so the ROS 2 environment and the
workspace overlay are always sourced correctly regardless of how this
node was launched. After the CLI returns, the resulting .yaml and .pgm
files are verified on disk before success is logged.
"""

import os
import shlex
import subprocess
import rclpy
from rclpy.node import Node


MAP_DIR = '/home/navbot/dev_ws/maps'
SAVE_INTERVAL_SEC = 30.0
SERIALIZE_INTERVAL_SEC = 60.0

# Sources that must be active for `ros2 run nav2_map_server map_saver_cli`
# to resolve and to see the running /map topic.
ROS_SETUP = '/opt/ros/jazzy/setup.bash'
WS_SETUP = '/home/navbot/dev_ws/install/setup.bash'


class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')

        self.declare_parameter('map_name', 'simple_world')
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        if not self.map_name:
            self.map_name = 'simple_world'  # default world

        self.map_subdir = os.path.join(MAP_DIR, self.map_name)
        os.makedirs(self.map_subdir, exist_ok=True)

        # map_saver_cli takes a path STEM (no extension) and writes
        # <stem>.yaml and <stem>.pgm next to each other. So the stem here
        # is <map_subdir>/<map_name>, producing e.g.
        #   /home/navbot/dev_ws/maps/simple_world/simple_world.yaml
        #   /home/navbot/dev_ws/maps/simple_world/simple_world.pgm
        # The repeated 'simple_world' segment in the log is intentional:
        # the first is the per-world directory, the second is the file stem.
        self.map_stem = os.path.join(self.map_subdir, self.map_name)

        self.get_logger().info(
            f'Map saver started — saving to {self.map_stem}.(yaml|pgm) '
            f'every {SAVE_INTERVAL_SEC:.0f}s (occupancy grid) '
            f'and every {SERIALIZE_INTERVAL_SEC:.0f}s (SLAM pose graph)'
        )

        self.create_timer(SAVE_INTERVAL_SEC, self._save_map)
        self.create_timer(SERIALIZE_INTERVAL_SEC, self._serialize_slam_map)

    # ------------------------------------------------------------------
    def _run_sourced(self, ros_cmd: str, timeout: float):
        """Run a `ros2 ...` command inside a bash shell that has both the
        distro and the workspace overlay sourced. This is the key fix:
        when this node is spawned as a subprocess (e.g. via a launch file
        that itself was started from a fresh shell), the child process
        does NOT necessarily inherit a sourced ROS 2 environment. Without
        sourcing, `ros2 run nav2_map_server map_saver_cli` either fails
        to resolve or runs without a working DDS discovery config and
        never sees the /map topic — which is exactly the symptom the
        user reported (map never gets written to disk).
        """
        shell_cmd = (
            f'set -e; '
            f'source {shlex.quote(ROS_SETUP)}; '
            f'[ -f {shlex.quote(WS_SETUP)} ] && source {shlex.quote(WS_SETUP)}; '
            f'{ros_cmd}'
        )
        return subprocess.run(
            ['bash', '-lc', shell_cmd],
            capture_output=True,
            text=True,
            timeout=timeout,
        )

    # ------------------------------------------------------------------
    def _save_map(self):
        """Save occupancy grid (.yaml + .pgm) via nav2_map_server CLI."""
        yaml_path = self.map_stem + '.yaml'
        pgm_path = self.map_stem + '.pgm'

        ros_cmd = (
            f'ros2 run nav2_map_server map_saver_cli '
            f'-f {shlex.quote(self.map_stem)} '
            f'--ros-args -p use_sim_time:=true'
        )

        try:
            result = self._run_sourced(ros_cmd, timeout=20)
        except subprocess.TimeoutExpired:
            self.get_logger().warn('map_saver_cli timed out — will retry next interval')
            return
        except Exception as exc:
            self.get_logger().error(f'map_saver_cli error: {exc}')
            return

        if result.returncode != 0:
            self.get_logger().warn(
                f'map_saver_cli returned {result.returncode}: '
                f'{(result.stderr or result.stdout).strip()}'
            )
            return

        # Verify the files actually exist on disk. map_saver_cli can
        # return 0 even when the /map topic never delivered a message,
        # so a returncode check alone is not proof of a real save.
        yaml_ok = os.path.isfile(yaml_path) and os.path.getsize(yaml_path) > 0
        pgm_ok = os.path.isfile(pgm_path) and os.path.getsize(pgm_path) > 0

        if yaml_ok and pgm_ok:
            self.get_logger().info(
                f'Occupancy grid saved → {yaml_path} '
                f'({os.path.getsize(yaml_path)} B) and '
                f'{pgm_path} ({os.path.getsize(pgm_path)} B)'
            )
        else:
            self.get_logger().warn(
                f'map_saver_cli reported success but files missing/empty '
                f'(yaml_ok={yaml_ok}, pgm_ok={pgm_ok}). '
                f'stdout: {result.stdout.strip()[-300:]}'
            )

    # ------------------------------------------------------------------
    def _serialize_slam_map(self):
        """Serialize the SLAM Toolbox pose graph via ros2 service call CLI.

        Left as-is per the user's instruction to change ONLY the map
        saving mechanism. This writes <stem>.posegraph and <stem>.data
        which are used to re-initialise slam_toolbox in localization
        mode on the next run.
        """
        payload = '{filename: "' + self.map_stem + '"}'
        ros_cmd = (
            f'ros2 service call '
            f'/slam_toolbox/serialize_map '
            f'slam_toolbox/srv/SerializePoseGraph '
            f'{shlex.quote(payload)}'
        )
        try:
            result = self._run_sourced(ros_cmd, timeout=15)
            if result.returncode == 0:
                self.get_logger().info(
                    f'SLAM pose graph serialized → {self.map_stem}.posegraph / .data'
                )
            else:
                self.get_logger().warn(
                    f'serialize_map service call returned {result.returncode}: '
                    f'{(result.stderr or result.stdout).strip()}'
                )
        except subprocess.TimeoutExpired:
            self.get_logger().warn('serialize_map timed out — will retry next interval')
        except Exception as exc:
            self.get_logger().warn(f'serialize_map error: {exc} — will retry next interval')


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
