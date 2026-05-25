from setuptools import setup
import os
from glob import glob

package_name = 'ros2_yolo_oak'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jw',
    maintainer_email='todo@todo.com',
    description='OAK-D S2 onboard YOLO spatial detector for solcam tracking',
    license='MIT',
    entry_points={
        'console_scripts': [
            'oak_detector = ros2_yolo_oak.oak_detector:main',
            'oak_viz = ros2_yolo_oak.oak_viz:main',
        ],
    },
)
