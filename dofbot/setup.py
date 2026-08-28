from setuptools import find_packages, setup


package_name = 'dofbot'


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
    description='ROS 2 Humble interface for a Yahboom DOFBOT arm over I2C.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dofbot = dofbot.driver:main',
        ],
    },
)
