"""
display.launch.py  —  visualise vtol_description URDF in RViz.

Usage:
  ros2 launch vtol_description display.launch.py
  ros2 launch vtol_description display.launch.py rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vtol_description')
    urdf_file = os.path.join(pkg, 'urdf', 'vtol.urdf')
    rviz_cfg  = os.path.join(pkg, 'launch', 'vtol.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Revolute / continuous joint names — published at position 0 so
    # robot_state_publisher can broadcast their TF frames.
    joint_names = [
        'aileron_link_joint',
        'ruddervator_left_link_joint',
        'ruddervator_right_link_joint',
        'prop_1_joint',
        'prop_2_joint',
        'prop_3_joint',
        'prop_4_joint',
        'pusher_joint',
    ]
    js_yaml = (
        "{header: {stamp: {sec: 0}}, "
        "name: [" + ", ".join(f"'{j}'" for j in joint_names) + "], "
        "position: [" + ", ".join(["0.0"] * len(joint_names)) + "]}"
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 for visualisation'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': False}]
    )

    # Publish a latched zero joint-state so RSP can compute all TF frames
    joint_state_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--qos-durability', 'transient_local',
            '/joint_states', 'sensor_msgs/msg/JointState', js_yaml,
        ],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        joint_state_pub,
        rviz2,
    ])
