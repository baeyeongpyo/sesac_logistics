from setuptools import find_packages, setup

package_name = 'mentorpi_fleet_sim'
setup(name=package_name, version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml']),
                  ('share/' + package_name + '/launch', ['launch/fleet_demo.launch.py']),
                  ('share/' + package_name + '/config', ['config/tasks.yaml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='sesac',
      maintainer_email='robotics@example.com', description='MentorPi fleet simulation',
      license='Apache-2.0', entry_points={'console_scripts': [
          'dispatcher = mentorpi_fleet_sim.dispatcher:main']})
