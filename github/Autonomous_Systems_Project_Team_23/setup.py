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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrew Abdelmalak',
    maintainer_email='andrew.abdelmalak@student.guc.edu.eg',
    description='Task 1 Nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'validation_node = Autonomous_Systems_Project_Team_23.m1_validation_print_node:main',
            'pub_sub_node = Autonomous_Systems_Project_Team_23.m1_vehicle_pub_sub_node:main'
        ],
    },
)
