from setuptools import find_packages, setup

package_name = 'camera_gui'

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
    maintainer='intelions',
    maintainer_email='intelions@example.com',
    description='Desktop camera preview for DOFBOT.',
    license='MIT',
    entry_points={'console_scripts': ['camera_gui = camera_gui.gui:main']},
)
