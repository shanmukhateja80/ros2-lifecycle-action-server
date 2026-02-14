from setuptools import find_packages, setup

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chinta-nsr-shanmukha-teja',
    maintainer_email='shanmukhateja80@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "CountUntilServer = actions_py.count_until_server:main",
            "CountUntilClient = actions_py.count_until_client:main",
            "count_until_server = actions_py.count_until_server_queue:main",
            "move_robot_server = actions_py.move_robot_server:main",
            "move_robot_client = actions_py.move_robot_client:main",
            "move_turtle = actions_py.move_turtle:main",
            "turtlebot = actions_py.turtlebot:main"
            "turtlebot = actions_py.turtlebottest:main"
        ],
    },
)
