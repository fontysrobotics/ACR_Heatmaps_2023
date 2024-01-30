import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot4_heat_map'

setup(
    name=package_name,
    version='1.0.1',
    packages=[package_name],
    data_files=[
    	('share/ament_index/resource_index/packages',
            ['resources/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resources', ['resources/colorbar.png']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shananda Surjadi',
    maintainer_email='shanandasurjadi@gmail.com',
    description='TurtleBot 4 Heat Map package',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_plan = turtlebot4_heat_map.path_plan:main',
            'map_generator = turtlebot4_heat_map.map_generator:main',
        ],
    },
)
