from setuptools import setup
import os
from glob import glob

package_name = 'object_following_robot'

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/bringup_launch.py', 'launch/localization_launch.py', 'launch/multi_robot_simulation_launch.py', 'launch/navigation_launch.py', 'launch/robot_gazebo_spawn_launch.py', 'launch/rviz_launch.py', 'launch/single_robot_simulation_launch.py', 'launch/slam_launch.py']))
#data_files.append(('share/' + package_name + '/launch', ['launch/robots_controller.py']))
data_files.append(('share/' + package_name + '/maps', ['maps/house_world.yaml']))
data_files.append(('share/' + package_name + '/maps', ['maps/house_world.pgm']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/house.world']))
data_files.append(('share/' + package_name + '/params', ['params/nav2_params.yaml']))
data_files.append(('share/' + package_name + '/params', ['params/nav2_robot1_params.yaml']))
data_files.append(('share/' + package_name + '/params', ['params/nav2_robot2_params.yaml']))
data_files.append(('share/' + package_name + '/params', ['params/follow_point_bt.xml']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/nav2_config.rviz']))
data_files.append(('share/' + package_name + '/rviz', ['rviz/nav2_config_multiple_robots.rviz']))
data_files.append(('share/' + package_name + '/urdf', ['urdf/two_wheeled_robot.urdf']))
data_files.append(('share/' + package_name + '/models/two_wheeled_robot_description', ['models/two_wheeled_robot_description/model.sdf']))
data_files.append(('share/' + package_name + '/models/two_wheeled_robot_description', ['models/two_wheeled_robot_description/model.config']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='emanuelerambaldi',
    maintainer_email='S5265998@studenti.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_following = object_following_robot.object_following:main',
            'publisher_subscriber = object_following_robot.publisher_subscriber:main',
            'basefootprint_frame_listener = object_following_robot.basefootprint_frame_listener:main',
            'robot2_pose_publisher = object_following_robot.robot2_pose_publisher:main'
        ],
    },
)
