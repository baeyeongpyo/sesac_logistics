from glob import glob
from setuptools import find_packages, setup


package_name = 'dofbot_gui'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='intelions',
    maintainer_email='intelions@example.com',
    description='A small ROS 2 GUI for publishing Yahboom DOFBOT joint poses.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dofbot_gui = dofbot_gui.gui:main',
            'dofbot_pick_place_gui = dofbot_gui.pick_place_gui:main',
            'dofbot_camera_coordinate_gui = dofbot_gui.camera_coordinate_gui:main',
            'dofbot_camera_coordinate_web = dofbot_gui.camera_coordinate_web:main',
        ],
    },
)
