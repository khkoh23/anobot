from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'anobot_scene'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name, "meshes",),glob("meshes/*"),),
        (os.path.join("share", package_name, "config"),glob("config/*.yaml"),),
    ],
    install_requires=[
      'setuptools',
      'trimesh',
    ],
    zip_safe=True,
    maintainer='khkoh',
    maintainer_email='khkoh23@gmail.com',
    description='This is the environmental scene description package for Anodizing Line Automation Robot',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
          'apply_factory_scene = anobot_scene.apply_factory_scene:main',
          'rod_scene_manager = anobot_scene.rod_scene_manager:main',
          'dummy_tank_marker = anobot_scene.dummy_tank_marker:main',
          'dummy_tank_scene = anobot_scene.dummy_tank_scene:main',
          'workstation_grasp_pose = anobot_scene.workstation_grasp_pose:main',
        ],
    },
)
