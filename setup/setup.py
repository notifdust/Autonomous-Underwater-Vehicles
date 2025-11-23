import os
from glob import glob
from setuptools import setup

package_name = 'auv_stack'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install all SDF models
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*.sdf')),
        # Install all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='notifdust',
    maintainer_email='gabriele.sepolvere@gmail.com',
    description='A modular AUV autonomy stack for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_controller = auv_stack.mpc_controller:main',
            'mission_planner = auv_stack.mission_planner:main',
            'thruster_allocator = auv_stack.thruster_allocator:main',
            'obstacle_avoidance = auv_stack.obstacle_avoidance:main',
        ],
    },
)