"""
Starts the whole system with one command:
  1. robot.launch.py   (robot_state_publisher, bridge node, lidar, foxglove_bridge)
  2. SLAM Toolbox      (after 5 s, when the robot and the gyro calibration are ready)
  3. Nav2              (after 15 s, when SLAM provides the transform map -> odom)

Usage:
  ros2 launch rasprover_bridge bringup.launch.py
  ros2 launch rasprover_bridge bringup.launch.py nav:=false        (only mapping)
  ros2 launch rasprover_bridge bringup.launch.py foxglove:=false

Keep the robot still during the first seconds (gyro calibration).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('rasprover_bridge')
    slam_pkg = get_package_share_directory('slam_toolbox')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    foxglove = LaunchConfiguration('foxglove')
    nav = LaunchConfiguration('nav')
    slam_params = LaunchConfiguration('slam_params')
    nav2_params = LaunchConfiguration('nav2_params')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', 'robot.launch.py')),
        launch_arguments={'foxglove': foxglove}.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={'slam_params_file': slam_params}.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav2_params}.items(),
        condition=IfCondition(nav),
    )

    return LaunchDescription([
        DeclareLaunchArgument('foxglove', default_value='true'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('slam_params', default_value='/ros2_ws/config/slam.yaml'),
        DeclareLaunchArgument('nav2_params', default_value='/ros2_ws/config/nav2.yaml'),

        robot,
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=15.0, actions=[nav2]),
    ])
