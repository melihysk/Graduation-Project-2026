from setuptools import find_packages, setup

package_name = "reservation_viz"

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
    maintainer="Melih",
    maintainer_email="melih@todo.todo",
    description="Shared RViz marker builders for reservation-based traffic controllers",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "graph_visualizer = reservation_viz.graph_visualizer:main",
        ],
    },
)
