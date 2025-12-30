from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'openarm_servo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.'),  # ✅ 자동으로 openarm_servo 하위 탐색
    package_dir={'': '.'},              # ✅ 현재 디렉토리를 root로 인식
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='OpenArm User',
    maintainer_email='user@example.com',
    description='MoveIt Servo launch and config files for OpenArm robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_servo_left = openarm_servo.keyboard_servo_left:main',
            'keyboard_servo_right = openarm_servo.keyboard_servo_right:main',
            'keyboard_servo_both = openarm_servo.keyboard_servo_both:main',
            'quest_servo_left = openarm_servo.quest_servo_left:main',
            'quest_servo_right = openarm_servo.quest_servo_right:main',
            'quest_servo_both = openarm_servo.quest_servo_both:main',
            'homing_right = openarm_servo.homing_right:main',
            'twist_transformer = openarm_servo.twist_transformer:main',
            'servo_monitor = openarm_servo.servo_monitor:main',
        ],
    },
)
