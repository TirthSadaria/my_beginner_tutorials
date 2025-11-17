"""
Launch file for talker and listener nodes with optional bag recording.

This launch file starts both the talker (publisher) and listener (subscriber) nodes
and optionally records all topics to a ROS 2 bag file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import os
import pathlib


def generate_launch_description():
    """
    Generate launch description for talker and listener nodes with bag recording.
    
    Returns:
        LaunchDescription: Launch description with nodes, arguments, and optional bag recording
    """
    # Declare launch argument for frequency
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='500',
        description='Publishing frequency in milliseconds for the talker node'
    )
    
    # Declare launch argument for bag recording
    record_arg = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='Enable/disable bag recording (true/false)'
    )
    
    # Declare launch argument for TF frames generation
    generate_tf_frames_arg = DeclareLaunchArgument(
        'generate_tf_frames',
        default_value='false',
        description='Enable/disable TF frames PDF generation (true/false)'
    )
    
    # Get launch configurations
    frequency = LaunchConfiguration('frequency')
    record = LaunchConfiguration('record')
    generate_tf_frames = LaunchConfiguration('generate_tf_frames')
    
    # Create talker node
    # RPATH is set in CMakeLists.txt so LD_LIBRARY_PATH is not needed
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
    
    # Convert record string to boolean for condition evaluation
    record_condition = PythonExpression(["'", record, "' == 'true'"])
    generate_tf_condition = PythonExpression(["'", generate_tf_frames, "' == 'true'"])
    
    # Get workspace root and results directory for TF frames
    launch_file_dir = pathlib.Path(__file__).parent.parent.parent.parent
    workspace_root = str(launch_file_dir)
    results_dir = os.path.join(workspace_root, 'results')
    os.makedirs(results_dir, exist_ok=True)
    
    # Bag recording process (only runs if record is true)
    bag_record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        condition=IfCondition(record_condition)
    )
    
    # Generate TF frames PDF after talker has been running (only if generate_tf_frames is true)
    # Wait 5 seconds for TF frames to accumulate, then run view_frames
    generate_frames = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
                output='screen',
                cwd=workspace_root,
                condition=IfCondition(generate_tf_condition)
            )
        ],
        condition=IfCondition(generate_tf_condition)
    )
    
    # Move PDF to results directory after generation
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
                output='screen',
                condition=IfCondition(generate_tf_condition)
            )
        ],
        condition=IfCondition(generate_tf_condition)
    )
    
    # Log message when recording is enabled
    record_info = LogInfo(
        msg=['Bag recording is enabled. Recording all topics to bag file.'],
        condition=IfCondition(record_condition)
    )
    
    # Log message when recording is disabled
    no_record_info = LogInfo(
        msg=['Bag recording is disabled. Set record:=true to enable bag recording.'],
        condition=IfCondition(PythonExpression(["'", record, "' != 'true'"]))
    )
    
    # Log message when TF frames generation is enabled
    tf_frames_info = LogInfo(
        msg=['TF frames PDF generation is enabled. Will generate PDF after 5 seconds.'],
        condition=IfCondition(generate_tf_condition)
    )
    
    return LaunchDescription([
        frequency_arg,
        record_arg,
        generate_tf_frames_arg,
        talker_node,
        listener_node,
        bag_record_process,
        generate_frames,
        move_pdf,
        record_info,
        no_record_info,
        tf_frames_info,
    ])

