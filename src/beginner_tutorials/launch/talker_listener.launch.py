"""
Launch file for talker and listener nodes.

This launch file starts both the talker (publisher) and listener (subscriber) nodes
with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for talker and listener nodes.
    
    Returns:
        LaunchDescription: Launch description with nodes and arguments
    """
    # Declare launch argument for frequency
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='500',
        description='Publishing frequency in milliseconds for the talker node'
    )
    
    # Get launch configuration
    frequency = LaunchConfiguration('frequency')
    
    # Create talker node
    # ROS 2 parameter system automatically converts string to int
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='minimal_publisher',
        parameters=[{'frequency': frequency}],
        output='screen'
    )
    
    # Create listener node
    listener_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen'
    )
    
    return LaunchDescription([
        frequency_arg,
        talker_node,
        listener_node,
    ])

