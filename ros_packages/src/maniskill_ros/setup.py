from setuptools import find_packages, setup

package_name = 'maniskill_ros'

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
    maintainer='fabian',
    maintainer_email='alvarez.fabian@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maniskill_bridge = maniskill_ros.maniskill_bridge_node:main',
            'test_maniskill = maniskill_ros.test_maniskill:main',
        ],
    },
)
