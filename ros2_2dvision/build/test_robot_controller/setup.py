from setuptools import setup

package_name = 'test_robot_controller'

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
    maintainer='vboxuser',
    maintainer_email='vboxuser@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "cam_loop = test_robot_controller.test_cam_loop:main",
            "cam_obj_detect = test_robot_controller.test_cam_detect:main",
            "cam_gs1 = test_robot_controller.test_cam_GS1:main",
            "cam_merge = test_robot_controller.test_cam_merge:main"
        ],
    },
)
