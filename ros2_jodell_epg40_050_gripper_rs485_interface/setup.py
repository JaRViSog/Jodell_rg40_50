from setuptools import setup
import os
from glob import glob

package_name = 'ros2_jodell_epg40_050_gripper_rs485_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': ''},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Usama',
    maintainer_email='nrih.robotics@gmail.com',
    description='TF03 LiDAR CAN interface with custom message',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'ros2_jodell_epg40_050_gripper_rs485_interface = ros2_jodell_epg40_050_gripper_rs485_interface.ros2_jodell_epg40_050_gripper_rs485_interface:main'
        ],
    },
)
