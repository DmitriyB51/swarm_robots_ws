from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    nodes = []

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
