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
            'controller = smart_kitchen_logic.robot_mover_node:main',
            'fake_human = smart_kitchen_logic.fake_human:main',
            'safety_monitor = smart_kitchen_logic.safety_monitor:main',
        ],
    },
)
