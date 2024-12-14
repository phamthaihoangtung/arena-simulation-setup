from glob import glob
import os

from setuptools import setup, find_packages

package_name = 'simulation-setup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(
        where='.',
        include=[f'{package_name}*']
    ),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Will recursively track all .yaml files in the entities/robots
        # directory and its subdirectories.
        *[
            (
                os.path.join('share', package_name, base),
                [os.path.join(base, file)]
            )
            for dir in ['configs', 'entities', 'launch', 'resource', 'worlds', 'gazebo_models', 'common']
            for base, dirs, files in os.walk(dir)
            for file in files
        ]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NamTruongTran',
    maintainer_email='trannamtruong98@gmail.com',
    description='simulation-setup.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
