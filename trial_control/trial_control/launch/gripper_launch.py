#  launch file for the linear actuator, dynamixels, and tactile sensors 
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Start up the Papillarray Tactile Sensors
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource([
				os.path.join(
					get_package_share_directory('papillarray_ros2_v2'),
					'launch/papillarray.launch.py'
				)
			]),
		),

        # Start the dual dynamixel motor interface
        Node(
            package='dynamixel_control',
            executable='motor_interface',
            name='motor_interface',
        ),

        # Start the recording node
        Node(
            package='trial_control',
            executable='record',
            name='trial_control',
        ),

        # Start the linear actuator control
        Node(
            package='trial_control',
            executable='linear_actuator',
            name='linear_actuator',
        ),

        # Start the slip check node
        Node(
            package='trial_control',
            executable='slip_check',
            name='slip_check',
        ),

        # Start the global check node
        Node(
            package='trial_control',
            executable='global_check',
            name='global_check',
        ),
])
