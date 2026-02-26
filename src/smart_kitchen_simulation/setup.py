from setuptools import find_packages, setup

package_name = 'smart_kitchen_simulation'

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
    maintainer_email='felix.lindenmeier@campus.lmu.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cup_position_node = smart_kitchen_simulation.cup_position_node:main',
            'fake_human_node = smart_kitchen_simulation.fake_human_node:main',
            'rail_simulator_node = smart_kitchen_simulation.rail_simulator_node:main',
            'tf_transformation = smart_kitchen_simulation.tf_transformation:main',
            'visual_debug_node = smart_kitchen_simulation.visual_debug_node:main',
        ],
    },
)
