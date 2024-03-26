import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'trial_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaky',
    maintainer_email='puentek@oregonstate.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record=trial_control.record:main',
            'record_client=trial_control.record_client:main',
            'linear_actuator=trial_control.linear_actuator:main',
            'data_collection=trial_control.data_collection:main',
            'cable_pull_trial=trial_control.cable_pull_trial:main',
        ],
    },
)
