from setuptools import find_packages, setup

package_name = 'amr_py_examples'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='npd',
    maintainer_email='npd@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = amr_py_examples.simple_publisher:main',
            'simple_subscriber = amr_py_examples.simple_subscriber:main',
            'simple_parameter = amr_py_examples.simple_parameter:main',
            'simple_turtlesim_kinematics = amr_py_examples.simple_turtlesim_kinematics:main',
            'simple_tf_kinematics = amr_py_examples.simple_tf_kinematics:main',
            'simple_service_server = amr_py_examples.simple_service_server:main',
            'simple_service_client = amr_py_examples.simple_service_client:main',
            'simple_action_server = amr_py_examples.simple_action_server:main',
            'simple_action_client = amr_py_examples.simple_action_client:main',
            'simple_lifecycle_node = amr_py_examples.simple_lifecycle_node:main',
            'simple_qos_publisher = amr_py_examples.simple_qos_publisher:main',
            'simple_qos_subscriber = amr_py_examples.simple_qos_subscriber:main',
        ],
    },
)
