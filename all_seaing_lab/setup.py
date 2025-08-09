from setuptools import find_packages, setup

package_name = 'all_seaing_lab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sim = all_seaing_lab.sim:main'
        ],
    },
)
