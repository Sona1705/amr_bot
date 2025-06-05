from setuptools import setup

package_name = 'amr_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='npd',
    maintainer_email='npd@example.com',
    description='AMR Controller Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_controller = amr_controller.simple_controller:main',
            'noisy_controller = amr_controller.noisy_controller:main',
            'twist_relay = amr_controller.twist_relay:main'
        ],
    },
)
