from setuptools import find_packages, setup


package_name = 'warehouse_overlay_server'


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
    description='Publish warehouse zone overlays for operations monitoring.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'warehouse_zone_publisher = warehouse_overlay_server.publisher:main',
        ],
    },
)
