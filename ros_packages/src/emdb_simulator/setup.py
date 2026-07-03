import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'emdb_simulator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test', 'tests']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            os.path.join('share', package_name),
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'worlds'),
            glob('config/worlds/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'robots'),
            glob('config/robots/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'tasks'),
            glob('config/tasks/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'scenarios'),
            glob('config/scenarios/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'arenas'),
            glob('config/arenas/*.yaml')
        ),
        (
            os.path.join('share', package_name, 'config', 'objects'),
            glob('config/objects/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fabian Alvarez',
    maintainer_email='alvarez.fabian@outlook.com',
    description='EMDB simulator',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_robocasa = emdb_simulator.core.robocasa_node:main',
            'test_scene_loader = emdb_simulator.core.scene_loader:main',
            'test_keyboard_client = emdb_simulator.core.keyboard_client:main',
            'test_position_server = emdb_simulator.core.position_server:main',
            'test_keyboard_teleop = emdb_simulator.core.keyboard_client:main',
        ],
    },
)