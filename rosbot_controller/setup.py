import os
from glob import glob
from setuptools import setup

package_name = 'rosbot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('scripts/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='k.yamshanov@fastsense.tech',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_gen = rosbot_controller.control_generator:main',
            'spawn_rosbot = rosbot_controller.spawn_rosbot:main',
            'rosbot_teleop = rosbot_controller.rosbot_teleop:main',
        ],
    },
)
