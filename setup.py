from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kaboat_autonomous'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='KABOAT Autonomous Navigation System',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'boat_controller = kaboat_autonomous.controllers.boat_controller:main',
            'gps_navigator = kaboat_autonomous.controllers.gps_navigator:main',
            'motor_controller = kaboat_autonomous.controllers.motor_controller:main',
            'keyboard_teleop = kaboat_autonomous.controllers.keyboard_teleop:main',
            'mission_runner = kaboat_autonomous.mission_runner:main',
            'visualize_local = kaboat_autonomous.visualize_local:main',
            'visualize_global = kaboat_autonomous.visualize_global:main',
            'integrated_visualizer = kaboat_autonomous.integrated_visualizer:main',
            'llm_interface = kaboat_autonomous.llm_interface:main',
            'action_dispatcher = kaboat_autonomous.action_dispatcher:main',
            'sensor_fusion = kaboat_autonomous.sensor_fusion:main',
        ],
    },
)
