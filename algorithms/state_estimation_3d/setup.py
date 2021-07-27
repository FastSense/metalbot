from setuptools import setup

package_name = 'state_estimation_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=['state_estimation_3d', 'state_estimation_3d.spacekf'],
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
            'ekf_node = state_estimation_3d.ekf_node:main'
        ],
    },
)
