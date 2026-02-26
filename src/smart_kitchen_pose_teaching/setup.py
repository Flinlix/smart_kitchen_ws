from setuptools import find_packages, setup

package_name = 'smart_kitchen_pose_teaching'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, [package_name + '/positions.toml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phri4',
    maintainer_email='phri4@todo.todo',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_place_node = smart_kitchen_pose_teaching.pick_place_node:main',
            'record_position = smart_kitchen_pose_teaching.utils.record_position:main',
            'goto_position = smart_kitchen_pose_teaching.utils.goto_position:main',
            'goto_carriage_lift = smart_kitchen_pose_teaching.utils.goto_carriage_lift:main'
        ],
    },
)
