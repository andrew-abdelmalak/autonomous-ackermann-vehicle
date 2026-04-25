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
        ('share/' + package_name + '/models/prius_team23', [
            'models/prius_team23/model.config',
            'models/prius_team23/model.sdf',
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
    description='Team 23 Autonomous Systems ROS 2 package',
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
        ],
    },
)
