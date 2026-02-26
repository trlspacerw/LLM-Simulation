"""
mission.launch.py — Launch Gazebo with the VTOL model.

The mission is now interactive — run run_mission.py in a separate terminal
after Gazebo has loaded:

  Terminal 1:
    source ~/ardu_ws/install/setup.bash
    ros2 launch vtol_description gazebo.launch.py

  Terminal 2 (after Gazebo is up):
    source ~/ardu_ws/install/setup.bash
    python3 ~/ardu_ws/install/vtol_description/share/vtol_description/scripts/run_mission.py

  Then type commands at the prompt:
    arm       — spool up propellers
    takeoff   — fly to 1 m hover altitude
    land      — descend and spool down after 2.5 s

Usage:
  ros2 launch vtol_description mission.launch.py
  ros2 launch vtol_description mission.launch.py gui:=false   # headless
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory('vtol_description')

    gz_resource_path = os.path.join(get_package_prefix('vtol_description'), 'share')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += ':' + gz_resource_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_resource_path
    if 'SDF_PATH' in os.environ:
        os.environ['SDF_PATH'] += ':' + gz_resource_path
    else:
        os.environ['SDF_PATH'] = gz_resource_path

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='true', description='Launch Gazebo GUI'
    )

    # ── Gazebo + model spawn ─────────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pkg) / 'launch' / 'gazebo.launch.py')
        ),
        launch_arguments={
            'gui':  LaunchConfiguration('gui'),
            'rviz': 'false',
        }.items(),
    )

    # ── Remind user how to start the interactive mission script ─────────────
    reminder = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=(
                '\n'
                '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
                '  Gazebo is ready.  Run the interactive mission in a\n'
                '  NEW terminal:\n\n'
                '    source ~/ardu_ws/install/setup.bash\n'
                '    python3 ~/ardu_ws/install/vtol_description/share/'
                'vtol_description/scripts/run_mission.py\n\n'
                '  Commands: arm → takeoff → land\n'
                '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
            )),
        ],
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        reminder,
    ])
