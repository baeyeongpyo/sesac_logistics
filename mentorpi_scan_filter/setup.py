import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mentorpi_scan_filter'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesac',
    maintainer_email='robotics@example.com',
    description='MentorPi fork self-reflection LaserScan filter.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'scan_filter = mentorpi_scan_filter.scan_filter_node:main',
    ]},
)
