from setuptools import setup

package_name = "fork_control"

setup(
    name=package_name,
    version="1.4.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/fork_control"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sesac",
    maintainer_email="robotics@example.com",
    description="Independent GPIO fork controller for pallet handling.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": ["fork_controller = fork_control.fork_controller:main"],
    },
)
