from setuptools import setup

package_name = "mentorpi_apriltag_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", [
            "launch/apriltag_follow.launch.py",
            "launch/ascamera_loading.launch.py",
            "launch/usb_camera_follow.launch.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sesac",
    maintainer_email="robotics@example.com",
    description="AprilTag detection and simple visual-servo drive control for MentorPi.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "apriltag_detector = mentorpi_apriltag_control.apriltag_detector:main",
            "apriltag_follower = mentorpi_apriltag_control.apriltag_follower:main",
            "loading_controller = mentorpi_apriltag_control.loading_controller:main",
        ],
    },
)
