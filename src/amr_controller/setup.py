from setuptools import setup

package_name = 'amr_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/' + package_name + '/launch', ['launch/teleop_joystick.launch.py', 'launch/controller_launch.py']),
    ('share/' + package_name + '/config', ['config/joy_config.yaml']),
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='AMR controller package',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_controller = amr_controller.simple_controller:main',
        ],
    },
)








