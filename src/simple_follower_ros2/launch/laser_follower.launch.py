import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():

    bringup_dir = get_package_share_directory('turn_on_mercury_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    mercury_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_mercury_robot.launch.py')),
    )
    mercury_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mercury_lidar.launch.py')),
    )
    return LaunchDescription([
        mercury_robot,mercury_lidar,
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='lasertracker', 
            name='lasertracker',
             ),
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='laserfollower', 
            ),]
    )

