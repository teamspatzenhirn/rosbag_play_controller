from setuptools import setup

package_name = 'rosbag_play_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'getkey'],
    zip_safe=True,
    maintainer='Team Spatzenhirn',
    maintainer_email='team-spatzenhirn@uni-ulm.de',
    description='Utility node for controlling the playback of rosbags',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rosbag_controller = rosbag_play_controller.rosbag_play_controller:main',
        ],
    },
)
