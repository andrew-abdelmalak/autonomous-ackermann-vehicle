"""MS4 simulation teleop launch file for Team 23."""

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

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    initial_yaw = LaunchConfiguration('initial_yaw')
    joystick_device = LaunchConfiguration('joystick_device')

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
        executable='joy_teleop_team_23',
        name='joy_teleop_team_23',
        output='screen',
        parameters=[{
            'joystick_device': joystick_device,
            'serial_forwarding_enabled': False,
            'publish_cmd_vel': True,
            'cmd_vel_topic': '/model/vehicle/cmd_vel',
            'estop_button': -1,
            'quit_button': -1,
            'max_speed_normal': 0.6,
            'max_speed_slow': 0.25,
            'max_speed_fast': 1.0,
            'max_turn_rate': 0.5,
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
            'initial_x',
            default_value='0.0',
            description='Initial vehicle x-position in meters.',
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial vehicle y-position in meters.',
        ),
        DeclareLaunchArgument(
            'initial_z',
            default_value='0.0',
            description='Initial vehicle z-position in meters.',
        ),
        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Initial vehicle yaw in radians.',
        ),
        DeclareLaunchArgument(
            'joystick_device',
            default_value='/dev/input/js0',
            description='Joystick device node.',
        ),
        gazebo,
        delayed_start,
    ])
