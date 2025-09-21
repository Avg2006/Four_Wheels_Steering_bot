from setuptools import find_packages, setup

package_name = 'gps'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Akhand Veer Garg',
    maintainer_email='akhandveergarg@gmail.com',
    author='Akhand Veer Garg',
    author_email='akhandveergarg@gmail.com',
    description='A ROS2 package to convert Point messages to NavSatFix and GPS coordinates to XY frame',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_fix_node = gps.gps_fix:main',
            'gps_to_xy_node = gps.gps_to_xy:main',
        ],
    },
)
