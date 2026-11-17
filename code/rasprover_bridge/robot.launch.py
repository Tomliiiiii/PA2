"""
Starts the complete robot with one command:
  - robot_state_publisher (URDF: base_link -> laser, imu_link)
  - rasprover bridge node (cmd_vel, odom, imu, battery)
  - RPLIDAR A2M12 driver (/scan)
  - foxglove_bridge (optional, for visualisation on the laptop)

Usage:
  ros2 launch rasprover_bridge robot.launch.py
  ros2 launch rasprover_bridge robot.launch.py foxglove:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('rasprover_bridge')
    with open(os.path.join(pkg, 'urdf', 'rasprover.urdf'), 'r') as f:
        robot_description = f.read()

    lidar_port = LaunchConfiguration('lidar_port')
    foxglove = LaunchConfiguration('foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('foxglove', default_value='true'),

        # Robot model: publishes the fixed transforms from the URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),

        # Bridge to the ESP32 (keep the robot still for 2 s after start)
        Node(
            package='rasprover_bridge',
            executable='bridge',
            parameters=[{'odom_scale': 0.0097}],
            output='screen',
        ),

        # Lidar driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('sllidar_ros2'), 'launch', 'sllidar_a2m12_launch.py')),
            launch_arguments={'serial_port': lidar_port, 'frame_id': 'laser'}.items(),
        ),

        # Visualisation bridge for Foxglove
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(
                get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')),
            condition=IfCondition(foxglove),
        ),
    ])
