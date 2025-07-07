#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """
    Launch file to start the complete ROS2 + PyTorch demo system.
    
    This launch file starts:
    1. Sensor Publisher - Simulates sensor data
    2. AI Processor - Uses PyTorch to process sensor data
    3. Robot Controller - Manages robot safety and commands
    """
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # Node configurations
    sensor_publisher_node = Node(
        package='ros2_pytorch_demo',
        executable='sensor_publisher',
        name='sensor_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    ai_processor_node = Node(
        package='ros2_pytorch_demo',
        executable='ai_processor',
        name='ai_processor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    robot_controller_node = Node(
        package='ros2_pytorch_demo',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    # Optional: Start RQT for monitoring (commented out by default)
    # rqt_node = ExecuteProcess(
    #     cmd=['rqt'],
    #     output='screen'
    # )
    
    # Optional: Start RViz2 for visualization (commented out by default)
    # rviz_node = ExecuteProcess(
    #     cmd=['rviz2'],
    #     output='screen'
    # )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        log_level_arg,
        
        # Start sensor publisher first
        sensor_publisher_node,
        
        # Start AI processor after a small delay to ensure sensor publisher is ready
        TimerAction(
            period=2.0,
            actions=[ai_processor_node]
        ),
        
        # Start robot controller after AI processor
        TimerAction(
            period=4.0,
            actions=[robot_controller_node]
        ),
        
        # Optionally start monitoring tools
        # TimerAction(
        #     period=6.0,
        #     actions=[rqt_node]
        # ),
    ])