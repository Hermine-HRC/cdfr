import glob
import os

from setuptools import find_packages, setup

package_name = 'herminebot_head'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'sequences'), glob.glob('sequences/*.json')),
    (os.path.join('share', package_name, 'params'), glob.glob('params/*.yaml')),
    (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py'))
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='axel',
    maintainer_email='axelt.hrc@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_node = herminebot_head.head_node:main',
            'get_team_color_service = herminebot_head.sim_color_team_service:main',
            'map_modifier = herminebot_head.map_modifier:main'
        ],
    },
)
