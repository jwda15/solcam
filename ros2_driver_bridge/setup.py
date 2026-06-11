from setuptools import setup

package_name = 'ros2_driver_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/driver.launch.py']),
        ('share/' + package_name + '/config', ['config/driver_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwda15',
    maintainer_email='jwda15@users.noreply.github.com',
    description='control_node ↔ STM32F407 드라이버 보드 UART 브리지',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_bridge = ros2_driver_bridge.driver_bridge:main',
        ],
    },
)
