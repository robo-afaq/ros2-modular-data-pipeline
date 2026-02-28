from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(package='research_robotics', executable='sensor_publisher', name='sensor_publisher'),
		Node(package='research_robotics', executable='processor_node', name='processor_node', parameters=[{'scale_factor': 3.0}]),
	])
