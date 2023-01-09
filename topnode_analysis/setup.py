import glob

from setuptools import find_packages
from setuptools import setup

package_name = 'topnode_analysis'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    maintainer=(
        'Michael Carroll'
    ),
    maintainer_email=(
        'michael@openrobotics.org'
    ),
    author=(
        'Michael Carroll'
    ),
    author_email=(
        'michael@openrobotics.org'
    ),
    url='https://github.com/safe-ros/topnode',
    keywords=[],
    description='Tools for analysing topnode data.',
    long_description=(
        'This package provides tools for analysing topnode data, from '
        'building a model of the performance data to providing plotting utilities.'
    ),
    entry_points={
        'console_scripts': [
            f'convert = {package_name}.convert:main',
        ],
    },
    license='Apache 2.0',
    tests_require=['pytest'],
)
