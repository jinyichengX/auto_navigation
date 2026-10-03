from setuptools import setup
import os
from glob import glob

package_name = 'px4_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jinyicheng',
    maintainer_email='jinyicheng@example.com',
    description='PX4 offboard 控制示例与键盘遥控',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_test = px4_control.arm_test:main',
            'offboard_takeoff = px4_control.offboard_takeoff:main',
            'offboard_move = px4_control.offboard_move:main',
            'keyboard_teleop = px4_control.keyboard_teleop:main',
            'tunnel_tour = px4_control.tunnel_tour:main',
            'tunnel_tour_avoid = px4_control.tunnel_tour_avoid:main',
            'gz_lidar_bridge = px4_control.gz_lidar_bridge:main',
            'px4_imu_bridge = px4_control.px4_imu_bridge:main',
            'fastlio_odom_bridge = px4_control.fastlio_odom_bridge:main',
        ],
    },
)
