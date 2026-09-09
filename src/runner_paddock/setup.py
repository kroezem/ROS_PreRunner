from setuptools import find_packages, setup


package_name = 'runner_paddock'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        f'{package_name}.static': [
            '*.css', '*.html', '*.js', '*.webmanifest',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['fastapi', 'setuptools', 'uvicorn', 'websockets'],
    # The web server passes the packaged static directory to Starlette, which
    # requires ordinary filesystem paths rather than a zipped egg resource.
    zip_safe=False,
    maintainer='matti',
    maintainer_email='matti@todo.todo',
    description='Paddock mode and command-authority supervision',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'stop_enforcer = runner_paddock.stop_enforcer:main',
            'command_authority = '
            'runner_paddock.command_authority_node:main',
            'mode_launcher = runner_paddock.mode_launcher:main',
            'mode_supervisor = runner_paddock.mode_supervisor_node:main',
            'map_executor = runner_paddock.map_session_node:main',
            'web = runner_paddock.web_app:main',
        ],
    },
)
