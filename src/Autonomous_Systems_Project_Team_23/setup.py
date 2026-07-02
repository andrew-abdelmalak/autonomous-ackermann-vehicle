from glob import glob

from setuptools import find_packages, setup

package_name = 'Autonomous_Systems_Project_Team_23'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/Worlds', glob('Worlds/*.world')),
        ('share/' + package_name + '/Worlds/legacy_reference',
            glob('Worlds/legacy_reference/*.world')),
        ('share/' + package_name + '/Worlds/legacy_reference/Empty_Track',
            glob('Worlds/legacy_reference/Empty_Track/*')),
        ('share/' + package_name + '/Worlds/legacy_reference/Racing_Track',
            glob('Worlds/legacy_reference/Racing_Track/*')),
        ('share/' + package_name + '/models/prius_team23', [
            'models/prius_team23/model.config',
            'models/prius_team23/model.sdf',
        ]),
        ('share/' + package_name + '/models/team23_ackermann_ms4', [
            'models/team23_ackermann_ms4/model.config',
            'models/team23_ackermann_ms4/model.sdf',
        ]),
        ('share/' + package_name + '/models/prius_team23/meshes',
            glob('models/prius_team23/meshes/*')),
        ('share/' + package_name + '/models/prius_team23/materials/textures',
            glob('models/prius_team23/materials/textures/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team 23',
    maintainer_email='team23@student.guc.edu.eg',
    description='Team 23 ROS 2 package for Autonomous Systems milestones.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'validation_node = '
            'Autonomous_Systems_Project_Team_23.'
            'Validation_Printing_Node_Team_23:main',
            'pub_sub_node = '
            'Autonomous_Systems_Project_Team_23.'
            'Vehicle_Pub_Sub_Node_Team_23:main',
            'ms2_olr_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_2_OLR_Team_23:main',
            'ms2_teleop_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_2_Teleop_Team_23:main',
            'ms3_clr_alg_1_speed_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_23:main',
            'ms3_clr_alg_2_lateral_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_23:main',
            'joy_teleop_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_3_Joy_Teleop_Team_23:main',
            'ms4_planning_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_4_Planning_Team_23:main',
            'ms4_cmd_record_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_4_Cmd_Record_Team_23:main',
            'ms4_cmd_replay_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_4_Cmd_Replay_Team_23:main',
            'ms4_collision_monitor_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_4_Collision_Monitor_Team_23:main',
            'ms5_localization_team_23 = '
            'Autonomous_Systems_Project_Team_23.'
            'Autonomous_Systems_MS_5_Localization_Team_23:main',
        ],
    },
)
