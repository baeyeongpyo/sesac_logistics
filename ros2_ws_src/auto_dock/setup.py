from setuptools import setup

package_name = "auto_dock"

setup(
    name=package_name,
    version="1.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/auto_dock"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/auto_dock.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sesac",
    maintainer_email="robotics@example.com",
    description="Headless pallet auto-docking launch package.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "auto_dock_node = auto_dock.auto_dock_node:main",
            "tag_entity_mapper = auto_dock.tag_entity_mapper:main",
        ],
    },
)
