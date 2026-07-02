"""MS4 simulation launch file for Team 23."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import Shutdown
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import FindExecutable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for MS4 simulation."""
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
    planner_track_mode = LaunchConfiguration('planner_track_mode')
    use_rqt_graph = LaunchConfiguration('use_rqt_graph')
    headless = LaunchConfiguration('headless')

    empty_track_lane = LaunchConfiguration('empty_track_lane')
    empty_track_lane_transition_distance = LaunchConfiguration(
        'empty_track_lane_transition_distance'
    )
    empty_track_lane_second = LaunchConfiguration('empty_track_lane_second')
    empty_track_lane_second_start = LaunchConfiguration('empty_track_lane_second_start')
    empty_track_lane_second_transition_distance = LaunchConfiguration(
        'empty_track_lane_second_transition_distance'
    )

    racing_override_lane_profile = LaunchConfiguration('racing_override_lane_profile')
    racing_profile_lane_1 = LaunchConfiguration('racing_profile_lane_1')
    racing_profile_lane_2 = LaunchConfiguration('racing_profile_lane_2')
    racing_profile_lane_3 = LaunchConfiguration('racing_profile_lane_3')
    racing_profile_switch_1_start = LaunchConfiguration('racing_profile_switch_1_start')
    racing_profile_switch_1_distance = LaunchConfiguration('racing_profile_switch_1_distance')
    racing_profile_switch_2_start = LaunchConfiguration('racing_profile_switch_2_start')
    racing_profile_switch_2_distance = LaunchConfiguration('racing_profile_switch_2_distance')
    racing_straight_speed = LaunchConfiguration('racing_straight_speed')
    racing_lane_change_speed = LaunchConfiguration('racing_lane_change_speed')
    racing_vehicle_width = LaunchConfiguration('racing_vehicle_width')
    racing_wall_margin = LaunchConfiguration('racing_wall_margin')
    racing_obstacle_longitudinal_margin = LaunchConfiguration(
        'racing_obstacle_longitudinal_margin'
    )

    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    wheel_base = LaunchConfiguration('wheel_base')
    max_speed = LaunchConfiguration('max_speed')
    max_turn_rate = LaunchConfiguration('max_turn_rate')

    speed_command_topic = LaunchConfiguration('speed_command_topic')
    steering_command_topic = LaunchConfiguration('steering_command_topic')
    command_topic = LaunchConfiguration('command_topic')
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

    gazebo_environment = [
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
    ]

    gazebo_gui = ExecuteProcess(
        cmd=gazebo_environment + [world_file],
        output='screen',
        on_exit=Shutdown(),
        condition=UnlessCondition(headless),
    )
    gazebo_headless = ExecuteProcess(
        cmd=gazebo_environment + ['-s', world_file],
        output='screen',
        on_exit=Shutdown(),
        condition=IfCondition(headless),
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
            '/vehicle/chassis_contacts'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/vehicle/front_left_wheel_contacts'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/vehicle/front_right_wheel_contacts'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/vehicle/rear_left_wheel_contacts'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/vehicle/rear_right_wheel_contacts'
            '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
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

    planning_node = Node(
        package=package_name,
        executable='ms4_planning_team_23',
        name='autonomous_systems_ms_4_planning_team_23',
        output='screen',
        parameters=[{
            'state_topic': '/odom',
            'planner_track_mode': planner_track_mode,
            'publish_rate_hz': publish_rate_hz,
            'empty_track_lane': empty_track_lane,
            'empty_track_lane_transition_distance': empty_track_lane_transition_distance,
            'empty_track_lane_second': empty_track_lane_second,
            'empty_track_lane_second_start': empty_track_lane_second_start,
            'empty_track_lane_second_transition_distance': empty_track_lane_second_transition_distance,
            'racing_override_lane_profile': racing_override_lane_profile,
            'racing_profile_lane_1': racing_profile_lane_1,
            'racing_profile_lane_2': racing_profile_lane_2,
            'racing_profile_lane_3': racing_profile_lane_3,
            'racing_profile_switch_1_start': racing_profile_switch_1_start,
            'racing_profile_switch_1_distance': racing_profile_switch_1_distance,
            'racing_profile_switch_2_start': racing_profile_switch_2_start,
            'racing_profile_switch_2_distance': racing_profile_switch_2_distance,
            'racing_straight_speed': racing_straight_speed,
            'racing_lane_change_speed': racing_lane_change_speed,
            'racing_vehicle_width': racing_vehicle_width,
            'racing_wall_margin': racing_wall_margin,
            'racing_obstacle_longitudinal_margin': racing_obstacle_longitudinal_margin,
        }],
    )

    speed_controller_node = Node(
        package=package_name,
        executable='ms3_clr_alg_1_speed_team_23',
        name='autonomous_systems_ms_3_clr_alg_1_speed_team_23',
        output='screen',
        parameters=[{
            'state_topic': '/odom',
            'speed_command_topic': speed_command_topic,
            'desired_speed_topic': '/ms4/desired_speed',
            'publish_rate_hz': publish_rate_hz,
            'desired_speed': 0.25,
            'max_speed': max_speed,
            'max_acceleration': max_acceleration,
            'speed_kp': speed_kp,
            'speed_ki': speed_ki,
            'speed_kd': speed_kd,
            'speed_integral_limit': speed_integral_limit,
            'desired_speed': 0.0,
        }],
    )

    lateral_controller_node = Node(
        package=package_name,
        executable='ms3_clr_alg_2_lateral_team_23',
        name='autonomous_systems_ms_3_clr_alg_2_lateral_team_23',
        output='screen',
        parameters=[{
            'command_topic': command_topic,
            'state_topic': '/odom',
            'speed_command_topic': speed_command_topic,
            'desired_lane_topic': '/ms4/desired_lane',
            'desired_heading_topic': '/ms4/desired_heading',
            'steering_command_topic': steering_command_topic,
            'publish_rate_hz': publish_rate_hz,
            'desired_lane': 0.0,
            'desired_heading': 0.0,
            'wheel_base': wheel_base,
            'max_speed': max_speed,
            'max_turn_rate': max_turn_rate,
            'max_steering_angle': max_steering_angle,
            'max_steering_rate': max_steering_rate,
            'lateral_kp': lateral_kp,
            'lateral_ki': lateral_ki,
            'lateral_kd': lateral_kd,
            'lateral_integral_limit': lateral_integral_limit,
            'heading_kp': heading_kp,
            'heading_kd': heading_kd,
        }],
    )

    collision_monitor_node = Node(
        package=package_name,
        executable='ms4_collision_monitor_team_23',
        name='autonomous_systems_ms_4_collision_monitor_team_23',
        output='screen',
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
            planning_node,
            speed_controller_node,
            lateral_controller_node,
            collision_monitor_node,
            rqt_graph,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='Empty_Track.world',
            description='World file to load from the package Worlds folder.',
        ),
        DeclareLaunchArgument(
            'planner_track_mode',
            default_value='empty_track',
            description='Planner mode: empty_track, racing_track, or city_track.',
        ),
        DeclareLaunchArgument(
            'empty_track_lane',
            default_value='0.0',
            description='Empty track lane target (c in -sin(h)x+cos(h)y=c). Negative is right for heading=0. Empty track walls are at y=+/-0.35 so +/-0.1625 is lane center.',
        ),
        DeclareLaunchArgument(
            'empty_track_lane_transition_distance',
            default_value='0.0',
            description='Meters over which to smoothly shift from 0.0 to empty_track_lane (0 disables).',
        ),
        DeclareLaunchArgument(
            'empty_track_lane_second',
            default_value='0.0',
            description='Optional second lane target for empty track (same units as empty_track_lane).',
        ),
        DeclareLaunchArgument(
            'empty_track_lane_second_start',
            default_value='1000000000.0',
            description='X position in meters at which to start transitioning to empty_track_lane_second.',
        ),
        DeclareLaunchArgument(
            'empty_track_lane_second_transition_distance',
            default_value='0.0',
            description='Meters over which to transition from first lane to second lane (0 switches immediately).',
        ),
        DeclareLaunchArgument(
            'racing_override_lane_profile',
            default_value='false',
            description='Override racing track lane plan with a fixed lane profile.',
        ),
        DeclareLaunchArgument(
            'racing_profile_lane_1',
            default_value='0.1875',
            description='Racing override lane 1 target.',
        ),
        DeclareLaunchArgument(
            'racing_profile_lane_2',
            default_value='-0.1875',
            description='Racing override lane 2 target (right lane).',
        ),
        DeclareLaunchArgument(
            'racing_profile_lane_3',
            default_value='0.1875',
            description='Racing override lane 3 target (left lane).',
        ),
        DeclareLaunchArgument(
            'racing_profile_switch_1_start',
            default_value='1.6',
            description='X position to start switching from lane 1 to lane 2.',
        ),
        DeclareLaunchArgument(
            'racing_profile_switch_1_distance',
            default_value='2.0',
            description='Meters to complete switch from lane 1 to lane 2.',
        ),
        DeclareLaunchArgument(
            'racing_profile_switch_2_start',
            default_value='5.6',
            description='X position to start switching from lane 2 to lane 3.',
        ),
        DeclareLaunchArgument(
            'racing_profile_switch_2_distance',
            default_value='2.0',
            description='Meters to complete switch from lane 2 to lane 3.',
        ),
        DeclareLaunchArgument(
            'racing_straight_speed',
            default_value='0.26',
            description='Target speed on Racing Track straights in m/s.',
        ),
        DeclareLaunchArgument(
            'racing_lane_change_speed',
            default_value='0.14',
            description='Target speed during Racing Track lane changes in m/s.',
        ),
        DeclareLaunchArgument(
            'racing_vehicle_width',
            default_value='0.18',
            description='Conservative vehicle envelope width used for clearance planning.',
        ),
        DeclareLaunchArgument(
            'racing_wall_margin',
            default_value='0.08',
            description='Extra wall margin reserved on each side of the Racing Track.',
        ),
        DeclareLaunchArgument(
            'racing_obstacle_longitudinal_margin',
            default_value='0.05',
            description='Extra longitudinal clearance before each obstacle front face.',
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
            'use_rqt_graph',
            default_value='false',
            description='Launch rqt_graph alongside the simulation.',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo server-only for validation.',
        ),
        DeclareLaunchArgument(
            'publish_rate_hz',
            default_value='20.0',
            description='Controller and planner update rate in Hz.',
        ),
        DeclareLaunchArgument(
            'wheel_base',
            default_value='0.255',
            description='Vehicle wheelbase in meters.',
        ),
        DeclareLaunchArgument(
            'max_speed',
            default_value='0.40',
            description='Maximum absolute speed command in m/s.',
        ),
        DeclareLaunchArgument(
            'max_turn_rate',
            default_value='0.65',
            description='Maximum absolute yaw-rate command in rad/s.',
        ),
        DeclareLaunchArgument(
            'speed_command_topic',
            default_value='/ms3/speed_command',
            description='Internal speed command topic between MS4 nodes.',
        ),
        DeclareLaunchArgument(
            'steering_command_topic',
            default_value='/ms3/steering_command',
            description='Internal steering-angle command topic for hardware reuse.',
        ),
        DeclareLaunchArgument(
            'command_topic',
            default_value='/model/vehicle/cmd_vel',
            description='Twist topic consumed by the simulated vehicle.',
        ),
        DeclareLaunchArgument(
            'speed_kp',
            default_value='0.85',
            description='Speed controller proportional gain.',
        ),
        DeclareLaunchArgument(
            'speed_ki',
            default_value='0.10',
            description='Speed controller integral gain.',
        ),
        DeclareLaunchArgument(
            'speed_kd',
            default_value='0.03',
            description='Speed controller derivative gain.',
        ),
        DeclareLaunchArgument(
            'speed_integral_limit',
            default_value='1.5',
            description='Absolute cap for speed controller integral term.',
        ),
        DeclareLaunchArgument(
            'max_acceleration',
            default_value='0.30',
            description='Maximum speed command slew in m/s^2.',
        ),
        DeclareLaunchArgument(
            'lateral_kp',
            default_value='0.30',
            description='Stanley cross-track gain.',
        ),
        DeclareLaunchArgument(
            'lateral_ki',
            default_value='0.0',
            description='Steady-state lateral bias correction gain.',
        ),
        DeclareLaunchArgument(
            'lateral_kd',
            default_value='0.10',
            description='Low-speed softening term for Stanley control.',
        ),
        DeclareLaunchArgument(
            'lateral_integral_limit',
            default_value='2.0',
            description='Absolute cap for lateral bias correction integral term.',
        ),
        DeclareLaunchArgument(
            'heading_kp',
            default_value='0.50',
            description='Heading-error gain.',
        ),
        DeclareLaunchArgument(
            'heading_kd',
            default_value='0.15',
            description='Heading-rate damping gain.',
        ),
        DeclareLaunchArgument(
            'max_steering_angle',
            default_value='0.30',
            description='Maximum absolute steering angle in radians.',
        ),
        DeclareLaunchArgument(
            'max_steering_rate',
            default_value='0.22',
            description='Maximum steering-angle slew in rad/s.',
        ),
        gazebo_gui,
        gazebo_headless,
        delayed_start,
    ])
