from setuptools import setup
import os
from glob import glob

package_name = 'robot_controller'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Optimized robot controller with Gemini AI integration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = robot_controller.motor_controller_node:main',
            'camera_publisher_node = robot_controller.camera_publisher_node:main',
        ],
    },
)


