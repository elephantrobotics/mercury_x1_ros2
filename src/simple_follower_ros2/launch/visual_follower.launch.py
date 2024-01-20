import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,TextSubstitution
import launch_ros.actions
from launch.conditions import IfCondition

#def launch(launch_descriptor, argv):
def generate_launch_description():
    Width_arg = DeclareLaunchArgument(
      'Width', default_value=TextSubstitution(text='640')  
    )
    #if use DaBai or Gemini [Height = 400]
    Height_arg = DeclareLaunchArgument(
      'Height', default_value=TextSubstitution(text='480') 
    )

    bringup_dir = get_package_share_directory('turn_on_mercury_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    mercury_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_mercury_robot.launch.py')),
    )

    mercury_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mercury_camera.launch.py')),
    )
    return LaunchDescription([
        mercury_robot,mercury_camera,Width_arg,Height_arg,
                
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='visualtracker', 
            name='visualtracker',
            parameters=[
                {'Width': LaunchConfiguration('Width')},
                {'Height': LaunchConfiguration('Height')}
                ]
            ),
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='visualfollow', 
            name='visualfollow',
            output='screen',
            ),]
            
    )

