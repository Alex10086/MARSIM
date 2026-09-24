from setuptools import find_packages, setup

package_name = 'quad_pid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/quad_pid_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/quad_pid.launch.py', 'launch/single_drone_quad_pid.launch.py', 'launch/twist_test.launch.py']),
    ],
    scripts=['scripts/twist_pub.py'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Quadrotor PID controller for MARSIM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quad_pid_node = quad_pid.quad_pid_node:main',
        ],
    },
)