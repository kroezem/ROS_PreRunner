from setuptools import find_packages, setup


package_name = 'runner_paddock'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['static/*']},
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['fastapi', 'setuptools', 'uvicorn', 'websockets'],
    zip_safe=True,
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='Paddock mode and command-authority supervision',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'command_authority = '
            'runner_paddock.command_authority_node:main',
            'mode_launcher = runner_paddock.mode_launcher:main',
            'mode_supervisor = runner_paddock.mode_supervisor_node:main',
            'web = runner_paddock.web_app:main',
        ],
    },
)
