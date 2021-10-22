
from setuptools import setup

package_name = 'rosbag_to_hdf5'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'oakd_data_recorder = rosbag_to_hdf5.oakd_data_recorder:main',
            'realsense_data_recorder = rosbag_to_hdf5.realsense_data_recorder:main',
            'rosbot_gazebo_data_recorder = rosbag_to_hdf5.rosbot_gazebo_data_recorder:main'
        ],
    },
)
