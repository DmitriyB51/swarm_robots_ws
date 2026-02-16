from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    num_drones = LaunchConfiguration('num_drones')

    declare_num = DeclareLaunchArgument(
        'num_drones',
        default_value='3'
    )

    nodes = []

    for i in range(3):   # static loop required by launch
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

    return LaunchDescription([declare_num] + nodes)
