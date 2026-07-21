import os
import pathlib

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)

    # Resolve the world file path so we can derive a world-specific map name
    world_path = LaunchConfiguration('world').perform(context)
    world_stem = pathlib.Path(world_path).stem          # e.g. 'simple_world'
    map_dir    = f'/home/navbot/dev_ws/maps/{world_stem}'
    map_name   = world_stem                              # e.g. 'simple_world'
    os.makedirs(map_dir, exist_ok=True)

    slam_params_file = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': f'-v4 {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'my_bot', '-z', '0.1'],
        output='screen',
    )

    bridge_params = os.path.join(pkg_share, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
    )

    lidar_avoider = Node(
        package='my_bot',
        executable='lidar_avoider.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    explorer = Node(
        package='my_bot',
        executable='explorer.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    rviz_config = os.path.join(pkg_share, 'config', 'slam.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # SLAM Toolbox — override map_file_name with the world-specific path
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {
                'use_sim_time': True,
                'map_file_name': os.path.join(map_dir, map_name),
            },
        ],
    )

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

    # Map saver — saves occupancy grid to the world-specific directory.
    # map_name is just the world stem (e.g. 'simple_world'); map_saver.py
    # builds the full path internally as MAP_DIR / map_name / map_name.
    map_saver = Node(
        package='my_bot',
        executable='map_saver.py',
        name='map_saver_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_name': map_name,
        }],
    )

    return [
        rsp,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        slam_toolbox,
        lifecycle_manager_slam,
        lidar_avoider,
        explorer,
        rviz2,
        map_saver,
    ]


def generate_launch_description():
    package_name = 'my_bot'
    pkg_share = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_share, 'worlds', 'simple_world.world'),
        description='Full path to the world file to load',
    )

    return LaunchDescription([
        world_arg,
        OpaqueFunction(function=_launch_setup),
    ])
