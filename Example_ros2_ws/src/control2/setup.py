from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Include all launch files in the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anveshak',
    maintainer_email='akhandveergarg@gmail.com',
    description='ROS2 package for 4WS rover drive and steering control',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_node = control2.master_code:main',
        ],
    },
)
