"""MS4 simulation keyboard teleop launch file for Team 23."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'Autonomous_Systems_Project_Team_23'
    pkg_share = get_package_share_directory(package_name)
    vehicle_sdf = os.path.join(
        pkg_share, 'models', 'team23_ackermann_ms4', 'model.sdf'
    )
    world_file = PathJoinSubstitution([
        pkg_share,
        'Worlds',
        LaunchConfiguration('world_file'),
    ])

    initial_x = '0.0'
    initial_y = '0.0'
    initial_z = '0.0'
    initial_yaw = '0.0'
    teleop_terminal_prefix = LaunchConfiguration('teleop_terminal_prefix')

    gazebo = ExecuteProcess(
        cmd=[
            'env',
            '-u', 'GTK_PATH',
            '-u', 'GTK_EXE_PREFIX',
            '-u', 'GIO_MODULE_DIR',
            '-u', 'GSETTINGS_SCHEMA_DIR',
            '-u', 'GTK_IM_MODULE_FILE',
            '-u', 'LOCPATH',
            '-u', 'XDG_DATA_HOME',
            '-u', 'XDG_DATA_DIRS',
            FindExecutable(name='gz'),
            'sim',
            '-r',
            world_file,
        ],
        output='screen',
        on_exit=Shutdown(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/vehicle/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/model/vehicle/cmd_vel'
            '@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/vehicle/odometry'
            '@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/world/empty/model/vehicle/joint_state', 'joint_states'),
            ('/model/vehicle/odometry', '/odom'),
        ],
        output='screen',
    )

    spawn_vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-file', vehicle_sdf,
            '-name', 'vehicle',
            '-x', initial_x,
            '-y', initial_y,
            '-z', initial_z,
            '-Y', initial_yaw,
        ],
    )

    teleop_node = Node(
        package=package_name,
        executable='ms2_teleop_team_23',
        name='autonomous_systems_ms_2_teleop_team_23',
        output='screen',
        emulate_tty=True,
        prefix=teleop_terminal_prefix,
        parameters=[{
            'command_topic': '/model/vehicle/cmd_vel',
            'state_topic': '/odom',
            'joint_state_topic': 'joint_states',
            'max_speed': 1.0,
            'max_turn_rate': 0.5,
            'serial_forwarding_enabled': False,
        }],
    )

    delayed_start = TimerAction(
        period=5.0,
        actions=[
            spawn_vehicle,
            bridge,
            teleop_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='Racing_Track.world',
            description='World file to load from the package Worlds folder.',
        ),
        DeclareLaunchArgument(
            'teleop_terminal_prefix',
            default_value='gnome-terminal --',
            description=(
                'Terminal prefix for teleop (opens its own window). '
                'Use "xterm -e" as an alternative.'
            ),
        ),
        gazebo,
        delayed_start,
    ])
