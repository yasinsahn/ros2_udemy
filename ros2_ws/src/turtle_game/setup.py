from setuptools import setup

package_name = 'turtle_game'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yasin',
    maintainer_email='yasin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawner_node = turtle_game.turtle_spawner:main',
            'turtle_controller_node = turtle_game.turtle_controller:main'
        ],
    },
)
