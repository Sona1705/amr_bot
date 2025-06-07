from setuptools import setup
import os
from glob import glob

package_name = 'amr_mapping'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('amr_mapping/launch/*.slam.launch.py')),
        # Install config files (optional, if you have them)
        (os.path.join('share', package_name, 'config'), glob('amr_mapping/config/*slam_toolbox.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='npd',
    maintainer_email='npd@example.com',
    description='AMR Mapping Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'slam.launch = amr_mapping slam.launch : main'
        ],
    },
)
