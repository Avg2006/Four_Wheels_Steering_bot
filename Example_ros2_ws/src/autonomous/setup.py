from setuptools import find_packages, setup

package_name = 'autonomous'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy', 'tf-transformations'],
    zip_safe=True,
    maintainer='Akhand Veer Garg',
    maintainer_email='akhandveergarg@gmail.com',
    author='Akhand Veer Garg',
    author_email='akhandveergarg@gmail.com',
    description='A ROS2 node implementing a go-to-goal controller for 2D navigation using odometry feedback',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomous_node = autonomous.go_to_goal:main',
        ],
    },
)
