from setuptools import setup

package_name = 'ros2_phone_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/phone.launch.py']),
        ('share/' + package_name + '/scripts',
         ['scripts/setup_v4l2loopback.sh', 'scripts/start_scrcpy_camera.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwda15',
    maintainer_email='jwda15@users.noreply.github.com',
    description='안드로이드 폰(촬영 카메라) ↔ Jetson USB 브리지 (scrcpy/adb)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phone_bridge = ros2_phone_bridge.phone_bridge:main',
        ],
    },
)
