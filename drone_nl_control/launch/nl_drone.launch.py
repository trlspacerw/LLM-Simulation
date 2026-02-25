"""Launch file: natural language drone control on top of iris_tracking simulation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # ── arguments ─────────────────────────────────────────────────────────
    mission_arg = DeclareLaunchArgument(
        'mission',
        default_value='',
        description='Natural language mission description, e.g. "find the walking person"',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz alongside the simulation.',
    )

    # ── include iris_tracking simulation ─────────────────────────────────
    iris_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ardupilot_gz_bringup'),
                'launch',
                'iris_tracking.launch.py',
            ])
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )

    # ── mission controller node ───────────────────────────────────────────
    mission_controller = Node(
        package='drone_nl_control',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'mission': LaunchConfiguration('mission'),
            'hover_altitude_m': 8.0,
        }],
    )

    return LaunchDescription([
        mission_arg,
        rviz_arg,
        iris_tracking_launch,
        mission_controller,
    ])
