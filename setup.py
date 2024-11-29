from glob import glob
import os

from setuptools import setup

package_name = 'simulation-setup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],  # No Python modules
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
            for dir in ['configs', 'entities', 'launch', 'resource', 'worlds']
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
