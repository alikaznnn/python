from setuptools import setup

package_name = 'ros2_pytorch_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/demo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Demo User',
    maintainer_email='demo@example.com',
    description='A simple demo package combining ROS2 with PyTorch for AI-powered robotics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_publisher = ros2_pytorch_demo.sensor_publisher:main',
            'ai_processor = ros2_pytorch_demo.ai_processor:main',
            'robot_controller = ros2_pytorch_demo.robot_controller:main',
        ],
    },
)