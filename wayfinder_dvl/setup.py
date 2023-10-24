import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'wayfinder_dvl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'config_name'),glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noel',
    maintainer_email='noel@todo.todo',
    description='Wayfinder DVL Node for ROS2',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['wayfinder_dvl = wayfinder_dvl.wayfinder_dvl:main'],
    },
)
