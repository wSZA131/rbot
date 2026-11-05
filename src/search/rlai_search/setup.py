from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rlai_search'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robolabs AI',
    maintainer_email='contact@robolabs.ai',
    description='Search task management and object localisation for rbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'search_manager = rlai_search.search_manager:main',
            'target_mapper = rlai_search.target_mapper:main',
        ],
    },
)
