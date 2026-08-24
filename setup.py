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
        ],
    },
)
