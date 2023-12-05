from setuptools import find_packages, setup
from glob import glob

package_name = 't14_ros_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs', glob('docs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carlostojal',
    maintainer_email='carlos.c.tojal@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge = t14_ros_bridge.bridge:main',
        ],
    },
)
