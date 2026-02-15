from setuptools import setup

package_name = 'webots_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Bridge between Webots and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = webots_bridge.sensor_publisher:main',
            'command_subscriber = webots_bridge.command_subscriber:main',
            'obstacle_avoider = webots_bridge.obstacle_avoider:main',
        ],
    },
)
