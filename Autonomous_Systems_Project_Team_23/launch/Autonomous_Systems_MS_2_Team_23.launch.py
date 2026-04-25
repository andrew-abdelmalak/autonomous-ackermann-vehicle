# Copyright 2026 Team 23
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""MS2 launch file for Team 23 – Gazebo vehicle with OLR or Teleop mode."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for the MS2 simulation."""
    package_name = 'Autonomous_Systems_Project_Team_23'
    pkg_share = get_package_share_directory(package_name)
    vehicle_sdf = os.path.join(
        pkg_share, 'models', 'prius_team23', 'model.sdf'
    )

    control_mode = LaunchConfiguration('control_mode')
    initial_speed = LaunchConfiguration('initial_speed')
    lane = LaunchConfiguration('lane')
    desired_speed = LaunchConfiguration('desired_speed')
    desired_lane = LaunchConfiguration('desired_lane')
    desired_steering = LaunchConfiguration('desired_steering')
    use_rqt_graph = LaunchConfiguration('use_rqt_graph')
    serial_forwarding_enabled = LaunchConfiguration(
        'serial_forwarding_enabled'
    )
    serial_port = LaunchConfiguration('serial_port')
    serial_baudrate = LaunchConfiguration('serial_baudrate')
    teleop_terminal_prefix = LaunchConfiguration('teleop_terminal_prefix')

    # Shared topic / limit parameters for both driving nodes.
    shared_params = {
        'command_topic': '/model/vehicle/cmd_vel',
        'state_topic': '/model/vehicle/odometry',
        'joint_state_topic': 'joint_states',
        'left_steering_joint_name': 'front_left_wheel_steering_joint',
        'right_steering_joint_name': 'front_right_wheel_steering_joint',
        'publish_rate_hz': 10.0,
        'wheel_base': 1.0,
        'max_speed': 1.0,
        'max_turn_rate': 0.5,
    }

    # --- Gazebo Harmonic simulator ---
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
            'empty.sdf',
        ],
        output='screen',
        on_exit=Shutdown(),
    )

    # --- ROS <-> Gazebo bridge ---
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
        ],
        output='screen',
    )

    # --- Spawn vehicle (z=0.325 so wheels sit on the ground plane) ---
    spawn_vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'empty',
            '-file', vehicle_sdf,
            '-name', 'vehicle',
            '-x', '0.0',
            '-y', lane,
            '-z', '0.325',
        ],
    )

    # --- OLR (Open-Loop Response) node ---
    olr_node = Node(
        package=package_name,
        executable='ms2_olr_team_23',
        name='autonomous_systems_ms_2_olr_team_23',
        output='screen',
        parameters=[{
            **shared_params,
            'desired_speed': desired_speed,
            'desired_steering': desired_steering,
        }],
        condition=IfCondition(
            PythonExpression(["'", control_mode, "' == 'olr'"])
        ),
    )

    # --- Teleop (Keyboard) node ---
    teleop_node = Node(
        package=package_name,
        executable='ms2_teleop_team_23',
        name='autonomous_systems_ms_2_teleop_team_23',
        output='screen',
        emulate_tty=True,
        prefix=teleop_terminal_prefix,
        parameters=[{
            **shared_params,
            'initial_speed': initial_speed,
            'lane': lane,
            'desired_speed': desired_speed,
            'desired_lane': desired_lane,
            'serial_forwarding_enabled': serial_forwarding_enabled,
            'serial_port': serial_port,
            'serial_baudrate': serial_baudrate,
        }],
        condition=IfCondition(
            PythonExpression(["'", control_mode, "' == 'teleop'"])
        ),
    )

    # --- rqt_graph ---
    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
        condition=IfCondition(use_rqt_graph),
    )

    delayed_start = TimerAction(
        period=5.0,
        actions=[spawn_vehicle, bridge, olr_node, teleop_node, rqt_graph],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode',
            default_value='teleop',
            description=(
                'Control mode: teleop (keyboard) or olr (open-loop response).'
            ),
        ),
        DeclareLaunchArgument(
            'initial_speed',
            default_value='0.0',
            description='Initial forward speed in m/s.',
        ),
        DeclareLaunchArgument(
            'lane',
            default_value='0.0',
            description='Lane value (also used as vehicle spawn Y offset).',
        ),
        DeclareLaunchArgument(
            'desired_speed',
            default_value='0.25',
            description='Desired forward speed in m/s (used by OLR).',
        ),
        DeclareLaunchArgument(
            'desired_lane',
            default_value='0.0',
            description='Desired lane value.',
        ),
        DeclareLaunchArgument(
            'desired_steering',
            default_value='0.0',
            description='Desired steering angular.z in rad/s (used by OLR).',
        ),
        DeclareLaunchArgument(
            'use_rqt_graph',
            default_value='true',
            description='Launch rqt_graph alongside the simulation.',
        ),
        DeclareLaunchArgument(
            'serial_forwarding_enabled',
            default_value='false',
            description='Forward teleop commands to an Arduino serial port.',
        ),
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino serial port used when serial forwarding is enabled.',
        ),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Arduino serial baud rate.',
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
