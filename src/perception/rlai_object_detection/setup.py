from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'rlai_object_detection'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robolabs AI',
    maintainer_email='contact@robolabs.ai',
    description='Open-vocabulary object detection node for rbot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = rlai_object_detection.object_detector:main',
        ],
    },
)
