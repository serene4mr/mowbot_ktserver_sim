from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kt_server_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # add config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='nguyenhatrung411@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "kt_server_client_node = kt_server_bridge.kt_server_client_node:main",
        ],
    },
)
