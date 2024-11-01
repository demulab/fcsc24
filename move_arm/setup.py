import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'move_arm'

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
    maintainer='yusuke-wsl',
    maintainer_email='c1005073@planet.kanazawa-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_arm_node = move_arm.move_arm:main",
            "position_to_yaml = move_arm.position_to_yaml:main",
            "speed_arm_node = move_arm.speed_arm:main",
        ],
    },
)
