from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'wsg50_driver_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',[f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/srv', glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bhumika Shree',
    maintainer_email='bhumikashree96@gmail.com',
    description='ROS2 driver for Weiss Robotics WSG-50 two-finger gripper.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper_node = wsg50_driver_pkg.wsg50_driver.gripper_node:main',
            'gripper_control = wsg50_driver_pkg.wsg50_driver.gripper_control:control_gripper',
        ],
    },
)
