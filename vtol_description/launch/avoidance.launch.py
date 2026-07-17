"""
avoidance.launch.py — Launch Gazebo + obstacle avoidance node.

Includes the standard gazebo.launch.py and adds the obstacle_avoidance node.

Usage:
  ros2 launch vtol_description avoidance.launch.py
  ros2 launch vtol_description avoidance.launch.py gui:=false
  ros2 launch vtol_description avoidance.launch.py threshold:=0.12 roi:=0.5
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
    pkg = get_package_share_directory('vtol_description')

    # ── args ──────────────────────────────────────────────────────────────
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.08',
        description='Edge density threshold for obstacle detection',
    )
    roi_arg = DeclareLaunchArgument(
        'roi', default_value='0.6',
        description='Central ROI fraction (0.0–1.0)',
    )

    # ── Include gazebo.launch.py (starts Gazebo + bridge + spawn) ─────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'gazebo.launch.py')
        ),
    )

    # ── Obstacle avoidance node (delayed so cameras are ready) ────────────
    avoidance_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='vtol_description',
                executable='obstacle_avoidance.py',
                name='obstacle_avoidance',
                output='screen',
                arguments=[
                    '--threshold', LaunchConfiguration('threshold'),
                    '--roi', LaunchConfiguration('roi'),
                ],
                prefix='python3',
            )
        ],
    )

    # ── Bridge debug images back to Gazebo transport for GUI display ─────
    # ROS2 → GZ direction (] means ROS-to-GZ)
    cam_names = ['front', 'back', 'left', 'right', 'up', 'down']
    debug_bridge_args = [
        f'/obstacle_avoidance/cam_{n}@sensor_msgs/msg/Image]gz.msgs.Image'
        for n in cam_names
    ]
    debug_bridge = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='debug_image_bridge',
                output='screen',
                arguments=debug_bridge_args,
            )
        ],
    )

    return LaunchDescription([
        threshold_arg,
        roi_arg,
        gazebo_launch,
        avoidance_node,
        debug_bridge,
    ])
