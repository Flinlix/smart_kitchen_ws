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
            'controller = smart_kitchen_logic.robot_mover_node:main',
            'fake_human = smart_kitchen_logic.fake_human:main',
            'safety_monitor = smart_kitchen_logic.safety_monitor:main',
            'fake_rail_node = smart_kitchen_logic.fake_rail_node:main',
            'cup_node = smart_kitchen_logic.cup_node:main',
            'planning_node = smart_kitchen_logic.planning_node:main',
            'moving_node = smart_kitchen_logic.moving_node:main',
            'init_node = smart_kitchen_logic.init_node:main',
            'tf_transformation = smart_kitchen_logic.tf_transformation:main',
            'visual_marker_node = smart_kitchen_logic.visual_marker_node:main',
        ],
    },
)
