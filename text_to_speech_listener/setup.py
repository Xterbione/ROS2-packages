from setuptools import setup

package_name = 'text_to_speech_listener'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='jetson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'text_to_speech_listener = text_to_speech_listener.text_to_speech_listener:main',
            'emoteaudioplayer = text_to_speech_listener.emoteaudioplayer:main'
        ],
    },
)
