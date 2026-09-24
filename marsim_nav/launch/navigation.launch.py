"""Nav2 + static map + TF. Assumes MARSIM and quad_pid are already running."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('marsim_nav')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    params = LaunchConfiguration('params_file')
    map_yaml = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # navigation_launch.py starts controller_server, smoother_server,
    # planner_server, route_server, behavior_server, bt_navigator,
    # waypoint_follower, velocity_smoother, collision_monitor, docking_server
    # AND its own lifecycle_manager_navigation. Do not add a second manager.
    #
    # Its remap chain (verified against the installed file):
    #   controller_server  --cmd_vel -> cmd_vel_nav-->
    #   velocity_smoother  --in cmd_vel_nav, out cmd_vel_smoothed-->
    #   collision_monitor  --in cmd_vel_smoothed, out cmd_vel-->
    # quad_pid subscribes /cmd_vel_nav (the raw controller output), which is
    # the choice documented in quad_pid/config/quad_pid_nav.yaml.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params,
            'autostart': autostart,
            'use_composition': 'False',
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server',
        name='map_server', output='screen',
        parameters=[params, {'yaml_filename': map_yaml}],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
    )
    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[{'autostart': autostart, 'node_names': ['map_server']}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg, 'maps', 'forest.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'tf.launch.py'))),
        map_server, lifecycle_map,
        # Sequence, do not overlap: let map_server reach ACTIVE before the nav
        # stack's own lifecycle manager starts transitioning 10 nodes. Nav2's
        # lifecycle manager calls change_state via
        # rclcpp::spin_until_future_complete WITHOUT a timeout, so a single lost
        # service response hangs bringup forever. Two managers transitioning
        # concurrently on a busy machine makes that materially more likely.
        TimerAction(period=8.0, actions=[nav]),
    ])
