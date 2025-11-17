"""
Launch file for generating TF frames PDF.

This launch file starts the talker node, waits for TF frames to be published,
generates a TF frames PDF using view_frames, and saves it to the results directory.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
import pathlib


def generate_launch_description():
    """
    Generate launch description for TF frames PDF generation.
    
    Returns:
        LaunchDescription: Launch description with talker node and TF frame generation
    """
    # Declare launch argument for frequency
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='500',
        description='Publishing frequency in milliseconds for the talker node'
    )
    
    # Get launch configurations
    frequency = LaunchConfiguration('frequency')
    
    # Get workspace root and results directory
    launch_file_dir = pathlib.Path(__file__).parent.parent.parent.parent
    workspace_root = str(launch_file_dir)
    results_dir = os.path.join(workspace_root, 'results')
    
    # Create results directory if it doesn't exist
    os.makedirs(results_dir, exist_ok=True)
    
    # Create talker node
    talker_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='minimal_publisher',
        parameters=[{'frequency': frequency}],
        output='screen'
    )
    
    # Generate TF frames PDF after talker has been running
    # Wait 5 seconds for TF frames to accumulate, then run view_frames
    generate_frames = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen',
                cwd=workspace_root
            )
        ]
    )
    
    # Move PDF to results directory after generation
    # view_frames creates frames.pdf in current directory
    move_pdf = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 
                     f'if [ -f {workspace_root}/frames.pdf ]; then '
                     f'mv {workspace_root}/frames.pdf {results_dir}/tf_frames.pdf && '
                     f'rm -f {workspace_root}/frames_*.gv && '
                     f'echo "TF frames PDF saved to {results_dir}/tf_frames.pdf"; '
                     f'else echo "Error: frames.pdf not generated"; fi'],
                output='screen'
            )
        ]
    )
    
    # Shutdown after PDF is generated
    shutdown_action = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=['bash', '-c', 'echo "TF frames PDF generation complete. Shutting down..."'],
                output='screen',
                on_exit=Shutdown()
            )
        ]
    )
    
    return LaunchDescription([
        frequency_arg,
        talker_node,
        generate_frames,
        move_pdf,
        shutdown_action,
    ])

