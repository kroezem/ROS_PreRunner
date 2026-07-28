from glob import glob

from setuptools import find_packages, setup

package_name = 'runner_drive_adapter'

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
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='Nav2 Twist to normalized Runner drive-command adapter',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'drive_adapter = runner_drive_adapter.drive_adapter_node:main',
        ],
    },
)
