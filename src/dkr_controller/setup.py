from setuptools import find_packages, setup
import os
from glob import glob

package_name = "dkr_controller"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Melih",
    maintainer_email="melih@todo.todo",
    description="Dynamic Resource Reservation (DKR) traffic controller",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "dkr_traffic_manager = dkr_controller.traffic_manager_node:main",
        ],
    },
)
