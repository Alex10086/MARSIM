#!/usr/bin/env python3
"""
MARSIM + quad_pid launch file.
Same as single_drone_simple.launch.py but with quad_pid replacing cascadePID.
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument('drone_id', default_value='0')
    init_x_arg = DeclareLaunchArgument('init_x_', default_value='0.0')
    init_y_arg = DeclareLaunchArgument('init_y_', default_value='0.0')
    init_z_arg = DeclareLaunchArgument('init_z_', default_value='1.0')
    init_yaw_arg = DeclareLaunchArgument('init_yaw', default_value='0.0')
    odom_topic_arg = DeclareLaunchArgument('odom_topic', default_value='odom')
    # Optional: which RViz config to load. Defaults to MARSIM's
    # traj_simple.rviz, which has NO Nav2 Goal tool (only the default
    # 2D Goal Pose, whose /goal_pose topic Nav2 does not subscribe to).
    # marsim_nav's sim_navigation.launch.py overrides this with
    # marsim_nav.rviz so Nav2 goals can actually be sent from RViz.
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('test_interface'), 'config',
            'traj_simple.rviz']))

    quad_pid_params_arg = DeclareLaunchArgument(
        'quad_pid_params',
        default_value=PathJoinSubstitution([
            FindPackageShare('quad_pid'), 'config', 'quad_pid_params.yaml']),
        description='quad_pid parameter file. Pass '
                    'config/quad_pid_nav.yaml for Nav2 velocity mode.')

    drone_id = LaunchConfiguration('drone_id')
    init_x = LaunchConfiguration('init_x_')
    init_y = LaunchConfiguration('init_y_')
    init_z = LaunchConfiguration('init_z_')
    init_yaw = LaunchConfiguration('init_yaw')
    odom_topic = LaunchConfiguration('odom_topic')

    map_path = PathJoinSubstitution([
        FindPackageShare('map_generator'),
        'resource',
        'small_forest01cutoff.pcd'
    ])

    # ── Quadrotor dynamics ──
    quadrotor_dynamics_node = Node(
        package='mars_drone_sim',
        executable='quadrotor_dynamics_node',
        name='quadrotor_dynamics_node',
        output='screen',
        parameters=[{
            'mass': 1.9,
            'simulation_rate': 200.0,
            'quadrotor_name': 'quadrotor',
            'init_state_x': init_x,
            'init_state_y': init_y,
            'init_state_z': init_z,
        }]
    )

    # ── quad_pid (replaces cascadePID) ──
    quad_pid_params = LaunchConfiguration('quad_pid_params')
    quad_pid_node = Node(
        package='quad_pid',
        executable='quad_pid_node',
        name='quad_pid_node',
        output='screen',
        parameters=[quad_pid_params],
    )

    # ── Map generator ──
    map_generator_node = Node(
        package='map_generator',
        executable='map_pub',
        name='map_pub',
        output='screen',
        arguments=[map_path],
        parameters=[{
            'add_boundary': 0,
            'is_bridge': 0,
            'downsample_res': 0.1,
            'map_offset_x': 0.0,
            'map_offset_y': 0.0,
            'map_offset_z': 0.0,
        }]
    )

    # ── Test interface ──
    test_interface_node = Node(
        package='test_interface',
        executable='test_interface_node',
        name='test_interface_node',
        output='screen'
    )

    # ── LiDAR simulation (OpenGL renderer) ──
    lidar_node = Node(
        package='local_sensing_node',
        executable='opengl_render_node',
        name='quad0_pcl_render_node',
        output='screen',
        arguments=[map_path],
        remappings=[
            ('global_map', '/map_generator/global_cloud'),
            ('odometry', '/odom'),
        ],
        parameters=[{
            'drone_id': drone_id,
            'quadrotor_name': 'quad_0',
            'uav_num': 1,
            'is_360lidar': 0,
            'sensing_horizon': 30.0,
            'sensing_rate': 10.0,
            'estimation_rate': 10.0,
            'polar_resolution': 0.2,
            'yaw_fov': 70.4,
            'vertical_fov': 77.2,
            'min_raylength': 1.0,
            # NOTE: livox_linestep is declared as an INTEGER by the node
            # (opengl_render_node.cpp:44 -> declare_parameter("livox_linestep", 1),
            # read at :75 via .as_int()). Passing a float here makes the node
            # throw rclcpp::exceptions::InvalidParameterTypeException and die,
            # so /cloud never exists and Nav2's obstacle layer has no input.
            # (pointcloud_render_node.cpp is the dead ROS1-style node; do not
            # look there for parameter types.)
            'livox_linestep': 1,
            'curvature_limit': 100.0,
            'hash_cubesize': 5.0,
            'use_avia_pattern': 1,
            'downsample_res': 0.1,
            'dynobj_enable': 0,
            'dynobject_size': 0.5,
            'dynobject_num': 5,
            'dyn_mode': 1,
            'dyn_velocity': 1.0,
            'use_uav_extra_model': 1,
            'collisioncheck_enable': 0,
            'collision_range': 0.5,
            'output_pcd': 0,
        }]
    )

    # ── Odom visualization ──
    odom_visualization_node = Node(
        package='odom_visualization',
        executable='odom_visualization',
        name='odom_visualization',
        output='screen',
        parameters=[{
            'mesh_resource': 'package://odom_visualization/meshes/yunque.dae',
            'color_r': 1.0,
            'color_g': 0.0,
            'color_b': 0.0,
            'color_a': 1.0,
            'robot_scale': 2.0,
            'frame_id': 'world',
            'drone_id': drone_id,
            'quadrotor_name': 'quadrotor',
        }],
        remappings=[
            ('odom', 'odom'),
        ]
    )

    # ── RViz ──
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rvizvisualisation',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )

    return LaunchDescription([
        drone_id_arg,
        init_x_arg,
        init_y_arg,
        init_z_arg,
        init_yaw_arg,
        odom_topic_arg,
        quad_pid_params_arg,
        rviz_config_arg,
        # Start the controller FIRST so it is already publishing hover RPM
        # before the dynamics node comes up. Otherwise the dynamics runs with
        # zero RPM for ~1s (Python/numpy startup) and the drone free-falls.
        quad_pid_node,
        map_generator_node,
        rviz_node,
        # Delay everything that consumes/produces drone state until the
        # controller is up and publishing.
        TimerAction(period=2.0, actions=[
            quadrotor_dynamics_node,
            test_interface_node,
            lidar_node,
            odom_visualization_node,
        ]),
    ])