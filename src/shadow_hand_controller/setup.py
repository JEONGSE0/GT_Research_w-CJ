from setuptools import setup
from glob import glob
import os

package_name = 'shadow_hand_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/follow_shadow_hand.launch.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meow',
    maintainer_email='tj6774@gmail.com',
    description='Relay PoseStamped from controller to Isaac Sim consumer topic.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shadow_hand_relay = shadow_hand_controller.shadow_hand_relay:main',
            'shadow_hand_keyctl = shadow_hand_controller.shadow_hand_keyctl:main',
        ],
    },
)
