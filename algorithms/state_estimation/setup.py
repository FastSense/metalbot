from setuptools import setup

package_name = 'state_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=['state_estimation', 
              'state_estimation.ekf', 
              'state_estimation.ekf.diff10',
              'state_estimation.ekf.space12', 
              'state_estimation.ekf.space15',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='paul.ev@fastsense.tech',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
