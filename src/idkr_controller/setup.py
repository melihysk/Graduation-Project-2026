from setuptools import find_packages, setup
import os
from glob import glob

package_name = "idkr_controller"

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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Melih",
    maintainer_email="melih@todo.todo",
    description="Improved Dynamic Resource Reservation (IDKR) traffic controller",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "idkr_standalone_manager = idkr_controller.standalone_traffic_manager_idkr:main",
            "task_dispatcher_idkr = idkr_controller.task_dispatcher_dkr:main",
        ],
    },
)
