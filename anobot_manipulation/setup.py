from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'anobot_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "config"),glob("config/*.yaml"),),
        (os.path.join("share", package_name, "launch"),glob("launch/*.launch.py"),),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khkoh',
    maintainer_email='khkoh23@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'workstation_grasp_pose = anobot_manipulation.workstation_grasp_pose:main',
          'plan_to_pregrasp = anobot_manipulation.plan_to_pregrasp:main',
        ],
    },
)
