import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lbot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hk',
    maintainer_email='hk@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
          'talker = lbot_control.publisher_member_function:main',
          'listener = lbot_control.subscriber_member_function:main',
        ],

    },
)
