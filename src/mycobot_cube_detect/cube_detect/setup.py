from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cube_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minha',
    maintainer_email='alsgk0404@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'cube_test = cube_detect.cubetf:main',
          'cube_pixel = cube_detect.cube_pixel:main',
          'cube_world = cube_detect.cube_world_pub:main',
        ],
    },
)
