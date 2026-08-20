from setuptools import find_packages, setup


package_name = 'foxglove_ros_worker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesac logistics',
    maintainer_email='maintainer@example.com',
    description='Republish vehicle Foxglove telemetry into the server ROS domain.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'foxglove_ros_worker = foxglove_ros_worker.main:main',
            'fleet_command_api = foxglove_ros_worker.api:main',
        ],
    },
)
