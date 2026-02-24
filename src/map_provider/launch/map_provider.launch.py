from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess
import os


def generate_launch_description():
    # Declare the `map_file` argument
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value = os.path.expanduser("~/swarm_robots_ws/src/map_provider/maps/updated_maze_map.yaml"),
        description='Path to the map file'
    )

    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_file')}]
    )
    
    
    # Lifecycle Manager node
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server']}]
    )
    
    

    # Configure transition for the lifecycle node
    configure_transition = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
        output='screen'
    )

    # Activate transition (delayed to ensure configure completes)
    activate_transition = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
        output='screen'
    )

    # Delay activation to allow configuration to complete
    delayed_activate_transition = TimerAction(
        period=2.0,  # Delay in seconds
        actions=[activate_transition]
    )

    return LaunchDescription([
        map_file_arg,
        map_server_node,
        configure_transition,
        delayed_activate_transition,
    ])
