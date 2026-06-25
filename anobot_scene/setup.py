from setuptools import find_packages, setup

package_name = 'anobot_scene'

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
        ],
    },
)
