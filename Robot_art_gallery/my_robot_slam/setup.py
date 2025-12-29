from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('maps/*.pgm')),
        (os.path.join('share', package_name), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vh5465s',
    maintainer_email='vh5465s@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'set_initial_pose = ' + package_name + '.set_initial_pose:main',
            'coverage = ' + package_name + '.coverage:main',
            'pose_saver = ' + package_name + '.pose_saver:main',
        ],
    },
)
