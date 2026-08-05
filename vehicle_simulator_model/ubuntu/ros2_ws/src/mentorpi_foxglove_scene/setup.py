from setuptools import find_packages, setup


package_name = 'mentorpi_foxglove_scene'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesac',
    maintainer_email='robotics@example.com',
    description='Foxglove SceneUpdate publisher for the MentorPi warehouse.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sdf_scene_publisher = mentorpi_foxglove_scene.sdf_scene_publisher:main',
        ],
    },
)
