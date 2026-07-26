from glob import glob

from setuptools import find_packages, setup

package_name = 'runner_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        (
            'share/' + package_name + '/launch/include',
            glob('launch/include/*.py'),
        ),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='Runner launch files',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rf2o_scan_canonicalizer = '
            'runner_bringup.rf2o_scan_canonicalizer:main',
            'scan_rebinner = runner_bringup.scan_rebinner:main',
        ],
    },
)
