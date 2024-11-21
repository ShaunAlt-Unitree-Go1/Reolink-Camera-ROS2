from setuptools import find_packages, setup # type: ignore

package_name = 'camera_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # include launch files
        (
            f'share/{package_name}/launch',
            ['launch/launch_camera_read_stream.py']
        ), 
        # (
        #     os.path.join('share', package_name, 'launch'),
        #     glob(os.path.join('launch', '*launch.[pxy][yma]*')),
        # ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_reader=camera_ros2.camera_reader:main',
            'camera_streamer=camera_ros2.camera_streamer:main',
        ],
    },
)
