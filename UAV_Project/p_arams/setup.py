import os
from glob import glob
from setuptools import setup

package_name = 'p_arams'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sameer Tuteja',
    maintainer_email='sameer.tuteja@alumni.fh-aachen.de',
    description='ARAMS Project to fly through gates by detecting colors',
    license='CC',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project_script  =p_arams.project_script:main',        
        ],
    },
)
