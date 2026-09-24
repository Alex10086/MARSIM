"""Full stack: MARSIM + quad_pid(velocity profile) + TF + static map + Nav2 + RViz.

The MARSIM/quad_pid side is deliberately NOT re-declared here. It reuses
quad_pid's own `single_drone_quad_pid.launch.py`, which is the profile
quad_pid/README.md documents.

Re-declaring those nodes once already cost a debug cycle: several parameters are
declared as INTEGERs by the C++ nodes, and passing Python floats/strings makes
them throw rclcpp::exceptions::InvalidParameterTypeException and DIE at startup.
  - opengl_render_node: `livox_linestep` must be int (1, not 1.4)
  - odom_visualization: `drone_id` must be int (0, not '0')
Both failures are silent from Nav2's point of view: opengl_render_node dying
means /cloud never exists, so the costmap obstacle layer simply has no input and
the drone flies on the static map alone. Including the tested launch inherits the
correct types instead of duplicating the chance to get them wrong.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav_pkg = get_package_share_directory('marsim_nav')
    quad_pid_pkg = get_package_share_directory('quad_pid')

    # Source-tree absolute path: maps/*.pgm is gitignored and therefore not
    # installed into share/, so the installed copy cannot be relied upon.
    src_map_yaml = os.path.join(
        os.path.dirname(nav_pkg), '..', '..', '..',
        'src', 'marsim_nav', 'maps', 'forest.yaml')

    # MARSIM + quad_pid in velocity mode (so quad_pid subscribes /cmd_vel_nav).
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            quad_pid_pkg, 'launch', 'single_drone_quad_pid.launch.py')),
        launch_arguments={
            'quad_pid_params': os.path.join(
                quad_pid_pkg, 'config', 'quad_pid_nav.yaml'),
        }.items())

    # TF + map_server + Nav2 (and its own lifecycle manager).
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav_pkg, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': src_map_yaml}.items())

    return LaunchDescription([
        sim,
        # Give the sim time to publish /odom and /cloud before Nav2 configures
        # its costmaps: they need the TF tree and a robot pose.
        TimerAction(period=8.0, actions=[nav]),
    ])
