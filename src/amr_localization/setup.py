from setuptools import find_packages
from glob import glob
import os
import setup
package_name = 'amr_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='AMR localization with Kalman filter.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_filter = amr_localization.kalman_filter:main',
            'odometry_motion_model = amr_localization.odometry_motion_model:main'
        ],
    },
)


