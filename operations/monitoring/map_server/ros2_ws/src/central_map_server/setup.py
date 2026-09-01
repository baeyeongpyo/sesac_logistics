from setuptools import find_packages, setup


package_name = 'central_map_server'


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
    description='Publish the central static map for operations monitoring.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'central_map_publisher = central_map_server.publisher:main',
        ],
    },
)
