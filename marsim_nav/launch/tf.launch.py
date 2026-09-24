from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('lidar_offset_z', default_value='0.1'),
        Node(package='marsim_nav', executable='tf_broadcaster',
             name='marsim_tf_broadcaster', output='screen',
             parameters=[{'odom_topic': LaunchConfiguration('odom_topic'),
                          'lidar_offset_z': LaunchConfiguration('lidar_offset_z')}]),
    ])
