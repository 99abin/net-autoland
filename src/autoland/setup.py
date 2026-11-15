from setuptools import find_packages, setup

package_name = "autoland"
setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="abin",
    maintainer_email="bintanghidayatullah021@gmail.com",
    description="ROS 2 fixed-wing net landing prototype",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "parameter_manager = autoland.parameter_manager:main",
            "mission_manager = autoland.mission_manager:main",
            "landing_supervisor = autoland.landing_supervisor:main",
            "vision_controller = autoland.vision_controller:main",
            "fcu_interface = autoland.fcu_interface:main",
        ]
    },
)
