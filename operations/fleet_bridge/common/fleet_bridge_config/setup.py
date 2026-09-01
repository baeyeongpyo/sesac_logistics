from setuptools import find_packages, setup


package_name = 'fleet_bridge_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='sesac logistics',
    maintainer_email='devnull@example.com',
    description='Strict shared configuration contract for the Foxglove fleet bridge.',
    license='Apache-2.0',
)

