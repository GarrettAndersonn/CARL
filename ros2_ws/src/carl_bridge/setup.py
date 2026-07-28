from setuptools import find_packages, setup
from glob import glob

package_name = 'carl_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Caleb Hylkema',
    maintainer_email='calebhylkema6@gmail.com',
    description='Bridge between ROS2 and the CARL NUCLEO master',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = carl_bridge.bridge_node:main',
        ],
    },
)
