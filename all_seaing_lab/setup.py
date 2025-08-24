from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'all_seaing_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml')) + 
         glob(os.path.join('config', '*.rviz'))
         ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arcturus',
    maintainer_email='arcturus-logistics@mit.edu',
    description='Simulation workspace for arcturus onboarding lab',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_engine = all_seaing_lab.sim_engine:main',
            'teleop_controller = all_seaing_lab.teleop_controller:main',
            'waypoint_follower = all_seaing_lab.waypoint_follower:main',
            'buoy_course = all_seaing_lab.buoy_course:main'
        ],
    },
)
