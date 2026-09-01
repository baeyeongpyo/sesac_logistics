from setuptools import find_packages, setup


package_name = 'fleet_bridge_worker'

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
    description='Bridge vehicle telemetry and command APIs into fleet operations.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fleet_bridge_worker = fleet_bridge_worker.main:main',
            'fleet_command_api = fleet_bridge_worker.api:main',
            'fleet_rosbag_recorder = fleet_bridge_worker.recording:main',
            'fleet_telemetry_writer = fleet_bridge_worker.telemetry:main',
        ],
    },
)
