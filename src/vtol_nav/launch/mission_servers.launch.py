"""
Launch file for mission servers.

Starts:
- drone_slave_controller for each drone (from vtol_controller)
- leader_follower_server
- vtol_navigate_server
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    nodes = []

    # Launch drone_slave_controller for each drone
    for i in range(num_drones):
        drone_name = f"vtol_{i+1}"
        nodes.append(
            Node(
                package="vtol_controller",
                executable="drone_slave_controller",
                namespace=drone_name,
                name="controller",
                output="screen"
            )
        )

    # Launch leader_follower_server
    nodes.append(
        Node(
            package="vtol_nav",
            executable="leader_follower_server",
            name="leader_follower_server",
            output="screen"
        )
    )

    # Launch vtol_navigate_server
    drone_names = [f'vtol_{i+1}' for i in range(num_drones)]
    nodes.append(
        Node(
            package="vtol_nav",
            executable="vtol_navigate_server",
            name="vtol_navigate_server",
            output="screen",
            parameters=[{'drone_names': drone_names}]
        )
    )

    return nodes


def generate_launch_description():

    declare_num = DeclareLaunchArgument(
        'num_drones',
        default_value='3'
    )

    return LaunchDescription([
        declare_num,
        OpaqueFunction(function=launch_setup)
    ])
