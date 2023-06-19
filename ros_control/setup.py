from setuptools import setup

package_name = 'ros_control'
submodules = "ros_control/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica-dev',
    maintainer_email='robotica-dev@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "servo_control = ros_control.servo_control:main",
            "serial_input = ros_control.serial_input:main",
            "Telemetric = ros_control.Telemetric:main",
            "RandomData = ros_control.RandomData:main",
            "positioningshim = ros_control.positioningshim:main",
            "serial_output = ros_control.serial_output:main",
            "ax12_control = ros_control.ax12_control:main",
            "command_publisher = ros_control.command_publisher:main"

        ],
    },
)
