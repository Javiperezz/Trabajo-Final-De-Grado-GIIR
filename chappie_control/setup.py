from setuptools import find_packages, setup
from glob import glob

package_name = 'chappie_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Javi',
    maintainer_email='jpertru@ypv.edu.es',
    description='Chappie balance loop wrapped as a ROS 2 node. '
                'Subscribes to /cmd_vel and runs the PID balance loop, '
                'publishing /imu/data, /odom, /joint_states, /balance/state.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'balance_node = chappie_control.balance_node:main',
            'cascade_balance_node = chappie_control.cascade_balance_node:main',
            'joy_cmd_vel_node = chappie_control.joy_cmd_vel_node:main',
            'obstacle_guard_node = chappie_control.obstacle_guard_node:main',
            'pid_experiment_node = chappie_control.pid_experiment_node:main',
        ],
    },
)
