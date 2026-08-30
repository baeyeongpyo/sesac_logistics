from setuptools import setup


package_name = 'vehicle_command_api'


setup(
    name=package_name,
    version='0.1.0',
    py_modules=['vehicle_command_api'],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            [f'resource/{package_name}'],
        ),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesac',
    maintainer_email='robotics@example.com',
    description='HTTP command gateway for a vehicle ROS 2 graph.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vehicle_command_api = vehicle_command_api:main',
        ],
    },
)
