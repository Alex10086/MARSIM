from glob import glob

from setuptools import find_packages, setup

package_name = 'marsim_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Nav2 integration for MARSIM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster = marsim_nav.tf_broadcaster:main',
            'occupancy_grid_node = marsim_nav.occupancy_grid_node:main',
            'cloud_reframe_node = marsim_nav.cloud_reframe_node:main',
            'nav_diag = marsim_nav.nav_diag:main',
            'nav_live = marsim_nav.live_state:main',
        ],
    },
)
