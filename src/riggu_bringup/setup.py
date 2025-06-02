from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'riggu_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'slam_toolbox'), glob('launch/slam_toolbox/*_launch.py')),
        # Include all world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'slam_toolbox'), glob('config/slam_toolbox/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tarun',
    maintainer_email='taruntom1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
