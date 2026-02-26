"""
gazebo.launch.py  —  spawn the vtol_description URDF in Gazebo Harmonic.

Usage:
  ros2 launch vtol_description gazebo.launch.py
  ros2 launch vtol_description gazebo.launch.py gui:=false   # headless
  ros2 launch vtol_description gazebo.launch.py rviz:=true   # also open RViz

Optional (for interactive joint sliders):
  sudo apt install ros-humble-joint-state-publisher-gui
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('vtol_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo resolves package:// URIs to model:// when ingesting URDF.
    # Add the install share parent so it can find model://vtol_description/...
    gz_resource_path = os.path.join(get_package_prefix('vtol_description'), 'share')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
    # sdformat_urdf also uses SDF_PATH for the same resolution
    if 'SDF_PATH' in os.environ:
        os.environ['SDF_PATH'] += ':' + gz_resource_path
    else:
        os.environ['SDF_PATH'] = gz_resource_path

    urdf_file  = os.path.join(pkg, 'urdf', 'vtol.urdf')
    world_file = os.path.join(pkg, 'worlds', 'vtol_empty.sdf')
    rviz_cfg   = os.path.join(pkg, 'launch', 'vtol.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # Joint names that need a published position so RSP can broadcast TF.
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

    # ── args ────────────────────────────────────────────────────────────────
    gui_arg  = DeclareLaunchArgument('gui',  default_value='true',
                                     description='Launch Gazebo GUI')
    rviz_arg = DeclareLaunchArgument('rviz', default_value='false',
                                     description='Launch RViz2')

    # ── Kill any leftover Gazebo processes first ─────────────────────────────
    # Without this, the GUI connects to a previously-running Gazebo server
    # (e.g. from a prior quadplane session) and the vtol_world is invisible.
    kill_gz = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f "gz sim" 2>/dev/null; pkill -9 gzserver 2>/dev/null; sleep 1.5; true'],
        output='screen',
    )

    # ── Gazebo server (delayed to let the kill complete) ─────────────────────
    gz_server = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(pkg_ros_gz_sim) / 'launch' / 'gz_sim.launch.py')
                ),
                launch_arguments={
                    'gz_args': f'-v4 -s -r {world_file}'
                }.items(),
            )
        ],
    )

    # ── Gazebo GUI (delayed to start after the server is up) ─────────────────
    gz_gui = TimerAction(
        period=2.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(Path(pkg_ros_gz_sim) / 'launch' / 'gz_sim.launch.py')
                ),
                launch_arguments={'gz_args': '-v4 -g'}.items(),
                condition=IfCondition(LaunchConfiguration('gui')),
            )
        ],
    )

    # ── robot_state_publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
        }],
    )

    # Publish a latched zero joint-state so RSP can compute all TF frames.
    joint_state_pub = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--qos-durability', 'transient_local',
            '/joint_states', 'sensor_msgs/msg/JointState', js_yaml,
        ],
        output='screen',
    )

    # ── Spawn URDF into Gazebo (delayed so the server is ready) ──────────
    spawn_vtol = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_vtol',
                output='screen',
                arguments=[
                    '-world', 'vtol_world',
                    '-name',  'vtol',
                    '-topic', 'robot_description',
                    # Roll=+90° (π/2): model Y-up → world Z-up → belly-down.
                    # Gravity disabled in URDF so model floats at spawn height.
                    # z=0.3223: belly sits exactly on the ground plane (z=0).
                    '-x', '0', '-y', '0', '-z', '0.333',
                    '-R', '1.5708',
                ],
            )
        ],
    )

    # ── RViz (optional) ──────────────────────────────────────────────────
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        kill_gz,          # kills any existing gz sim before starting fresh
        gz_server,
        gz_gui,
        robot_state_publisher,
        joint_state_pub,
        spawn_vtol,
        rviz2,
    ])
