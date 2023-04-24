from setuptools import setup

package_name = 'turtlebot3_custom_navigation'

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
    maintainer='nstre',
    maintainer_email='nstre@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "drive_circle = turtlebot3_custom_navigation.drive_circle:main",
            "drive_square = turtlebot3_custom_navigation.drive_square:main",
            "drive_to_point_odom = turtlebot3_custom_navigation.drive_to_point_odom:main",
        ],
    },
)
