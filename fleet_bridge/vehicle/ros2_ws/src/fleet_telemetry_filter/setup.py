from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'fleet_telemetry_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesac logistics',
    maintainer_email='devnull@example.com',
    description='Vehicle-side telemetry rate and change filter.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'telemetry_filter = fleet_telemetry_filter.node:main',
        ],
    },
)

