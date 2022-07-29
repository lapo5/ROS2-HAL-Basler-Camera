import os
from glob import glob
from setuptools import setup

package_name = 'basler_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'calibration'), glob('calibration/*.json')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
       
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='HAL for AV Camera',
    license='BSD',
    entry_points={
        'console_scripts': [
        "basler_camera = basler_camera.basler_camera:main",       
        
        ],
    },
)
