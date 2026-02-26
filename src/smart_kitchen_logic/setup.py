from setuptools import find_packages, setup

package_name = 'smart_kitchen_logic'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/waypoints.toml',
            'config/commands.toml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros2-jazzy',
    maintainer_email='ros2-jazzy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_executor = smart_kitchen_logic.command_executor:main',
            'command_sequence_client = smart_kitchen_logic.command_sequence_client:main',
            'robot_controller = smart_kitchen_logic.robot_controller_node:main',
            'safety_monitor = smart_kitchen_logic.safety_monitor_node:main',
            'planning_node = smart_kitchen_logic.planning_node:main',
            'moving_node = smart_kitchen_logic.moving_node:main',
            'human_detection_node = smart_kitchen_logic.human_detection_node:main',
        ],
    },
)
