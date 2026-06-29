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

"""MS3 launch file for Team 23 - closed-loop speed and lateral control."""

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
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for MS3 simulation."""
    package_name = 'Autonomous_Systems_Project_Team_23'
    pkg_share = get_package_share_directory(package_name)
    vehicle_sdf = os.path.join(
        pkg_share, 'models', 'prius_team23', 'model.sdf'
    )

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    desired_speed = LaunchConfiguration('desired_speed')
    desired_lane = LaunchConfiguration('desired_lane')
    desired_heading = LaunchConfiguration('desired_heading')
    use_rqt_graph = LaunchConfiguration('use_rqt_graph')

    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    wheel_base = LaunchConfiguration('wheel_base')
    max_speed = LaunchConfiguration('max_speed')
    max_turn_rate = LaunchConfiguration('max_turn_rate')

    speed_command_topic = LaunchConfiguration('speed_command_topic')
    speed_kp = LaunchConfiguration('speed_kp')
    speed_ki = LaunchConfiguration('speed_ki')
    speed_kd = LaunchConfiguration('speed_kd')
    speed_integral_limit = LaunchConfiguration('speed_integral_limit')
    max_acceleration = LaunchConfiguration('max_acceleration')

    lateral_kp = LaunchConfiguration('lateral_kp')
    lateral_ki = LaunchConfiguration('lateral_ki')
    lateral_kd = LaunchConfiguration('lateral_kd')
    lateral_integral_limit = LaunchConfiguration('lateral_integral_limit')
    heading_kp = LaunchConfiguration('heading_kp')
    heading_kd = LaunchConfiguration('heading_kd')
    max_steering_angle = LaunchConfiguration('max_steering_angle')
    max_steering_rate = LaunchConfiguration('max_steering_rate')

    shared_params = {
        'command_topic': '/model/vehicle/cmd_vel',
        'state_topic': '/model/vehicle/odometry',
        'publish_rate_hz': publish_rate_hz,
        'wheel_base': wheel_base,
        'max_speed': max_speed,
        'max_turn_rate': max_turn_rate,
    }

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
            '-z', '0.325',
            '-Y', initial_yaw,
        ],
    )

    speed_controller_node = Node(
        package=package_name,
        executable='ms3_clr_alg_1_speed_team_23',
        name='autonomous_systems_ms_3_clr_alg_1_speed_team_23',
        output='screen',
        parameters=[{
            'state_topic': '/model/vehicle/odometry',
            'speed_command_topic': speed_command_topic,
            'publish_rate_hz': publish_rate_hz,
            'desired_speed': desired_speed,
            'max_speed': max_speed,
            'max_acceleration': max_acceleration,
            'speed_kp': speed_kp,
            'speed_ki': speed_ki,
            'speed_kd': speed_kd,
            'speed_integral_limit': speed_integral_limit,
        }],
    )

    lateral_controller_node = Node(
        package=package_name,
        executable='ms3_clr_alg_2_lateral_team_23',
        name='autonomous_systems_ms_3_clr_alg_2_lateral_team_23',
        output='screen',
        parameters=[{
            **shared_params,
            'speed_command_topic': speed_command_topic,
            'desired_lane': desired_lane,
            'desired_heading': desired_heading,
            'lateral_kp': lateral_kp,
            'lateral_ki': lateral_ki,
            'lateral_kd': lateral_kd,
            'lateral_integral_limit': lateral_integral_limit,
            'heading_kp': heading_kp,
            'heading_kd': heading_kd,
            'max_steering_angle': max_steering_angle,
            'max_steering_rate': max_steering_rate,
        }],
    )

    rqt_graph = Node(
        package='rqt_graph',
        executable='rqt_graph',
        output='screen',
        condition=IfCondition(use_rqt_graph),
    )

    delayed_start = TimerAction(
        period=5.0,
        actions=[
            spawn_vehicle,
            bridge,
            speed_controller_node,
            lateral_controller_node,
            rqt_graph,
        ],
    )

    return LaunchDescription([
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
            'initial_yaw',
            default_value='0.0',
            description='Initial vehicle yaw in radians.',
        ),
        DeclareLaunchArgument(
            'desired_speed',
            default_value='0.25',
            description='Desired speed setpoint in m/s.',
        ),
        DeclareLaunchArgument(
            'desired_lane',
            default_value='0.0',
            description='Desired lane setpoint in y-axis meters.',
        ),
        DeclareLaunchArgument(
            'desired_heading',
            default_value='0.0',
            description='Desired heading setpoint in radians.',
        ),
        DeclareLaunchArgument(
            'use_rqt_graph',
            default_value='true',
            description='Launch rqt_graph alongside simulation.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='Controller update rate in Hz.',
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='1.0',
            description='Vehicle wheelbase in meters.',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='1.0',
            description='Maximum absolute speed command in m/s.',
        ),
        DeclareLaunchArgument(
            'max_turn_rate',
            default_value='0.5',
            description='Maximum absolute yaw-rate command in rad/s.',
        ),
        DeclareLaunchArgument(
            'speed_command_topic',
            default_value='/ms3/speed_command',
            description='Internal speed command topic between MS3 nodes.',
        ),
        DeclareLaunchArgument(
            'speed_kp',
            default_value='1.2',
            description='Speed controller proportional gain.',
        ),
        DeclareLaunchArgument(
            'speed_ki',
            default_value='0.35',
            description='Speed controller integral gain.',
        ),
        DeclareLaunchArgument(
            'speed_kd',
            default_value='0.02',
            description='Speed controller derivative gain.',
        ),
        DeclareLaunchArgument(
            'speed_integral_limit',
            default_value='1.5',
            description='Absolute cap for speed controller integral term.',
        ),
        DeclareLaunchArgument(
            'max_acceleration',
            default_value='0.75',
            description='Maximum speed command slew in m/s^2.',
        ),
        DeclareLaunchArgument(
            'lateral_kp',
            default_value='0.95',
            description='Lateral controller proportional gain.',
        ),
        DeclareLaunchArgument(
            'lateral_ki',
            default_value='0.02',
            description='Lateral controller integral gain.',
        ),
        DeclareLaunchArgument(
            'lateral_kd',
            default_value='0.18',
            description='Lateral controller derivative gain.',
        ),
        DeclareLaunchArgument(
            'lateral_integral_limit',
            default_value='2.0',
            description='Absolute cap for lateral controller integral term.',
        ),
        DeclareLaunchArgument(
            'heading_kp',
            default_value='1.2',
            description='Heading controller proportional gain.',
        ),
        DeclareLaunchArgument(
            'heading_kd',
            default_value='0.25',
            description='Heading-rate damping gain.',
        ),
        DeclareLaunchArgument(
            'max_steering_angle',
            default_value='0.5',
            description='Maximum absolute steering angle in radians.',
        ),
        DeclareLaunchArgument(
            'max_steering_rate',
            default_value='1.0',
            description='Maximum steering-angle slew in rad/s.',
        ),
        gazebo,
        delayed_start,
    ])
