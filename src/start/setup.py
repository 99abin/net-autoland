from setuptools import find_packages, setup
from glob import glob

package_name = "start"
setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="abin",
    maintainer_email="bintanghidayatullah021@gmail.com",
    description="ROS 2 fixed-wing net landing prototype",
    license="Apache-2.0",
    entry_points={"console_scripts": []},
)
