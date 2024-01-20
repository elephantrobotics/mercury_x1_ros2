import os
import launch
import launch.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from ament_index_python import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    mercury_slam_dir = get_package_share_directory('mercury_slam_toolbox')
    bringup_dir = get_package_share_directory('turn_on_mercury_robot')

    mercury_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_mercury_robot.launch.py')),
    )
    mercury_slam = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(mercury_slam_dir, 'launch', 'online_sync.launch.py')
          ),
     )
    mercury_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'mercury_lidar.launch.py')),
    )
    mercury_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'mercury_camera.launch.py')),
    )
    parameters=[{
          'queue_size':20,
          'frame_id':'camera_link',
          'use_sim_time':use_sim_time,
          'subscribe_scan':True,
          'subscribe_depth':True}]

    remappings=[
          ('odom', '/odom_combined'),
          ('scan', '/scan'),
          ('rgb/image', '/camera/color/image_raw'), 
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    return LaunchDescription([
        mercury_robot,mercury_lidar,mercury_camera,
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Nodes to launch
        Node(
            package='rtabmap_ros', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),


    ])
