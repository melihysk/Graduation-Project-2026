from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'metric_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'config', 'scenarios'),
            glob('config/scenarios/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Melih',
    maintainer_email='melih@todo.todo',
    description='Metric collection for multi-robot traffic management comparison',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'metric_logger_node = metric_logger.metric_logger_node:main',
            'task_dispatcher_node = metric_logger.task_dispatcher_node:main',
        ],
    },
)
