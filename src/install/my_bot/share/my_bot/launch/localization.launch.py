# Localization launch for my_bot: brings up map_server + AMCL with a
# lifecycle_manager that drives both to 'active' automatically.
#
# Usage:
#   ros2 launch my_bot localization.launch.py
#   ros2 launch my_bot localization.launch.py map:=/absolute/path/to/my_map.yaml use_sim_time:=true
#
# By default this points at the my_map.yaml installed under the my_bot
# share directory (my_bot/maps/my_map.yaml). Save a map there with
# nav2_map_server's map_saver_cli and re-run `colcon build` so it lands
# in the install tree.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_bot')

    default_map_path = os.path.join(pkg_share, 'maps', 'my_map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map yaml file to load',
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the AMCL + map_server ROS 2 parameters file',
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically transition lifecycle nodes to active on startup',
    )

    # map_server: loads the occupancy grid from `map` and publishes it on /map.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
            {'topic': 'map'},
        ],
    )

    # AMCL: localizes the robot in the loaded map. Config (frames, initial
    # pose, scan topic) comes from amcl_params.yaml.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    # Drives map_server + amcl through configure -> activate on startup.
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 4.0,
        }],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_autostart,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
