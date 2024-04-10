from setuptools import find_packages, setup

package_name = 'cable_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marcus',
    maintainer_email='rosettem@oregonstate.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_server = rob599_hw2.move_server:main',
            'server_test = rob599_hw2.server_test:main',
        ],
    },
)
