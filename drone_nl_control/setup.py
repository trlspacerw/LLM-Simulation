from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_nl_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Natural language drone control using LLM+VLM',
    license='GPL-3.0',
    entry_points={
        'console_scripts': [
            'mission_controller = drone_nl_control.mission_controller:main',
            'bench_llm     = drone_nl_control.bench_llm:main',
            'bench_vlm     = drone_nl_control.bench_vlm:main',
            'bench_mission = drone_nl_control.bench_mission:main',
            'bench_report  = drone_nl_control.report:main',
        ],
    },
)
