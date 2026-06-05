from setuptools import setup

package_name = 'ros2_gesture_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gesture.launch.py']),
        ('share/' + package_name + '/config', ['config/gesture_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwda15',
    maintainer_email='jwda15@users.noreply.github.com',
    description='손동작 supervisor (HaGRID 제스처 메뉴 + LCD UI)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_node = ros2_gesture_node.gesture_node:main',
            'ui_node = ros2_gesture_node.ui_node:main',
        ],
    },
)
