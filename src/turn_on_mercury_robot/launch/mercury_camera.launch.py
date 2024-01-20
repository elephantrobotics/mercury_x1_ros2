import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	astra_dir = get_package_share_directory('astra_camera')
	astra_launch_dir = os.path.join(astra_dir,'launch')
 
	usbcam_dir=get_package_share_directory('usb_cam')
	usbcam_launch_dir = os.path.join(usbcam_dir,'launch')

	usbcam_arg = DeclareLaunchArgument(
		'video_device', default_value='/dev/video0',
		description='video device serial number.')
 
	Astra_S = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_mini.launch.py')),)

	Astra_Pro = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'astra_pro.launch.py')),)

	Dabai = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'dabai.launch.py')),)

	Gemini = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(astra_launch_dir,'gemini.launch.py')),)

	#Select the UVC device {Rgbcam(C70)、Astra_Gemini、Astra_Dabai}
	Wheeltec_Usbcam = IncludeLaunchDescription(
	PythonLaunchDescriptionSource(os.path.join(usbcam_launch_dir,'demo.launch.py')),
	launch_arguments={'video_device': '/dev/RgbCam'}.items())

	# Create the launch description and populate
	ld = LaunchDescription()
	
	#Select your camera here, options include:
	#Astra_S、Astra_Pro、Dabai、Gemini、Wheeltec_Usbcam
	ld.add_action(Astra_S)

	ld.add_action(usbcam_arg)

	return ld
