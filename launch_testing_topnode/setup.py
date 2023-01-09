
import glob

from setuptools import find_packages
from setuptools import setup


setup(
    name='launch_testing_topnode',
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/launch_testing_topnode']),
        ('share/launch_testing_topnode', ['package.xml']),
    ],
    entry_points={
        'console_scripts': ['launch_test=launch_testing.launch_test:main'],
        'pytest11': ['launch_testing = launch_testing.pytest.hooks'],
    },
    install_requires=['setuptools'],
    zip_safe=True,
    author='Michael Carroll',
    author_email='michael@openrobotics.org',
    maintainer='Michael Carroll',
    maintainer_email='michael@openrobotics.org',  # noqa: E501
    url='https://github.com/safe-ros/topnode',
    download_url='https://github.com/safe-ros/topnode/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Extends launch_testing to assert about performance',
    long_description=('A package to create tests which involve'
                      ' asserting performance with launch files and multiple processes.'),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
)
