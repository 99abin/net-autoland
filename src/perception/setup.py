from setuptools import find_packages, setup

package_name = "perception"
setup(
    name=package_name,
    version="0.1.0",
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
            "net_det = perception.net_det:main",
            "net_pos = perception.net_pos:main",
        ]
    },
)
