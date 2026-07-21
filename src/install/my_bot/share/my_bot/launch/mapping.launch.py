"""All-in-one mapping launch.

Brings up: Gazebo + robot + ros_gz_bridge + SLAM (mapping mode) +
lifecycle manager + map auto-saver + RViz.

The robot starts, SLAM draws the map live in RViz, and the auto-saver
periodically writes maps/my_map.yaml / maps/my_map.pgm to disk. When
you close and relaunch (either this file or localization.launch.py),
the saved map is available on disk for reuse.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)

    # Where the auto-saver will write the map on disk. Points at the
    # source-tree maps/ folder so the saved map survives rebuilds and
    # is picked up by localization.launch.py on the next run.
    map_save_path = os.path.expanduser(
        '~/dev_ws/src/my_bot/maps/my_map'
    )

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')
    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')

    # robot_state_publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # World argument + Gazebo
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_world.world'),
        description='World to load',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items(),
    )

    # Spawn the robot from /robot_description (wait for Gazebo to init)
    spawn_entity = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-topic', '/robot_description',
                           '-name', 'my_bot',
                           '-z', '0.1'],
                output='screen',
            )
        ],
    )

    # ROS <-> Gazebo bridge (clock, cmd_vel, odom, scan, tf, joint states)
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    # SLAM Toolbox (mapping mode)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': True}],
    )

    # Dedicated lifecycle manager for slam_toolbox (bond_timeout=0.0 is required)
    lifecycle_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'bond_timeout': 0.0,
            'node_names': ['slam_toolbox'],
        }],
    )

    # Auto-save the map periodically (starts after SLAM has time to activate)
    map_auto_saver = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='my_bot',
                executable='map_auto_saver.py',
                name='map_auto_saver',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'save_period_sec': 10.0,
                    'map_path': map_save_path,
                }],
            )
        ],
    )

    # RViz for live visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        slam_toolbox,
        lifecycle_manager_slam,
        map_auto_saver,
        rviz2,
    ])
