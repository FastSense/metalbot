from setuptools import setup

package_name = 'logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/logger_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='KostyaYamshanov',
    maintainer_email='k.yamshanov@fastsense.tech',
    description='A package for logging the state and control of the robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'logger = logger.logger:main'
        ],
    },
)
