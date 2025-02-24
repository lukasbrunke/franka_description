from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'franka_vis_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Tools for visualizing additional elements with Franka robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sphere_publisher = franka_vis_tools.sphere_publisher:main',
            'custom_marker_publisher = franka_vis_tools.custom_marker_publisher:main',
        ],
    },
)