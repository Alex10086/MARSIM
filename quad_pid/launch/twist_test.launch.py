#!/usr/bin/env python3
"""MARSIM + quad_pid + twist_pub -- verifies Twist mode with NO Nav2.

This exists to decouple two failure domains: when a Nav2 end-to-end run
misbehaves, running this against the same controller tells you whether the
controller or the Nav2 configuration is at fault.

Examples:
    # straight line, then stop publishing (must brake and hold)
    ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 duration:=5

    # pre-rotate to -54 deg, then drive forward -- the orbit-bug regression
    ros2 launch quad_pid twist_test.launch.py pattern:=forward vx:=1.0 yaw:=-54

    # side slip at yaw=0
    ros2 launch quad_pid twist_test.launch.py pattern:=strafe vy:=1.0

    # square circuit in the world frame
    ros2 launch quad_pid twist_test.launch.py pattern:=square duration:=16

    # drive /cmd_vel by hand instead
    ros2 launch quad_pid twist_test.launch.py run_twist_pub:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── twist_pub arguments ──
    run_twist_pub = LaunchConfiguration('run_twist_pub')
    pattern = LaunchConfiguration('pattern')
    vx = LaunchConfiguration('vx')
    vy = LaunchConfiguration('vy')
    wz = LaunchConfiguration('wz')
    duration = LaunchConfiguration('duration')
    linger = LaunchConfiguration('linger')
    yaw = LaunchConfiguration('yaw')

    declared = [
        DeclareLaunchArgument('run_twist_pub', default_value='true',
                              description='launch the Twist injector'),
        DeclareLaunchArgument('pattern', default_value='forward',
                              description='zero|forward|backward|strafe|spin|square'),
        DeclareLaunchArgument('vx', default_value='1.0'),
        DeclareLaunchArgument('vy', default_value='1.0'),
        DeclareLaunchArgument('wz', default_value='0.5'),
        DeclareLaunchArgument('duration', default_value='5.0',
                              description='seconds to publish; then publishing STOPS'),
        DeclareLaunchArgument('linger', default_value='6.0',
                              description='seconds to keep observing after the stop'),
        DeclareLaunchArgument('yaw', default_value='',
                              description='pre-rotate to this yaw (deg) before injecting'),
    ]

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('quad_pid'), 'launch',
            'single_drone_quad_pid.launch.py',
        ])),
    )

    # twist_pub maps an empty --yaw to None (no pre-rotation), so the flag can
    # always be passed and `yaw:=` simply means "don't rotate first".
    injector = Node(
        package='quad_pid',
        executable='twist_pub.py',
        name='twist_pub',
        output='screen',
        condition=IfCondition(run_twist_pub),
        arguments=['--pattern', pattern,
                   '--vx', vx, '--vy', vy, '--wz', wz,
                   '--duration', duration, '--linger', linger,
                   '--yaw', yaw],
    )

    return LaunchDescription(declared + [
        sim,
        # Wait for odom to exist and the controller to be publishing before
        # injecting: twist_pub pre-rotates using the current pose.
        TimerAction(period=6.0, actions=[injector]),
    ])
