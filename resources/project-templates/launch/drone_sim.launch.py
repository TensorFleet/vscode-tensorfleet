#!/usr/bin/env python3
"""
TensorFleet Drone Simulation Launch File
Launches Gazebo simulation with PX4 SITL
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for drone simulation"""
    
    # Declare arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='iris',
        description='Drone model (iris, typhoon_h480, plane)'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty',
        description='Gazebo world (empty, warehouse, outdoor)'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )
    
    # Get launch configurations
    model = LaunchConfiguration('model')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    
    # PX4 SITL
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gazebo'],
        cwd='~/PX4-Autopilot',  # Update to your PX4 path
        output='screen'
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn drone model
    spawn_drone = ExecuteProcess(
        cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
             '-entity', 'drone',
             '-model', model,
             '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )
    
    # ROS 2 bridge for PX4
    px4_ros_bridge = Node(
        package='px4_ros_com',
        executable='sensor_combined_listener',
        name='px4_ros_bridge',
        output='screen'
    )
    
    return LaunchDescription([
        model_arg,
        world_arg,
        headless_arg,
        px4_sitl,
        gazebo,
        spawn_drone,
        px4_ros_bridge,
    ])

