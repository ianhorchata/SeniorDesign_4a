from glob import glob

from setuptools import setup

package_name = 'bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/resource', ['resource/me480_webots.urdf']),
        # Symlinks into the Webots project (SeniorDesign_4a/worlds and SeniorDesign_4a/protos).
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),
        ('share/' + package_name + '/protos', glob('protos/*.proto')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SeniorDesign_4a',
    maintainer_email='user@todo.todo',
    description='Webots bringup for the ME480 robot: ROS 2 driver, localization, OctoMap mapping and RViz.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'localization = bringup.localization:main',
        ],
    },
)
