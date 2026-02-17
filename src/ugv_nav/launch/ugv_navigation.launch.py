#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    num_ugvs = int(LaunchConfiguration('num_ugvs').perform(context))

    nodes = []

    for i in range(num_ugvs):
        ugv_name = f"ugv_{i+1}"

        nodes.append(
            Node(
                package="ugv_swarm_path_planner",
                executable="ugv_astar_planner",
                namespace=ugv_name,
                name="ugv_astar_planner",
                output="screen"
            )
        )

        nodes.append(
            Node(
                package="ugv_swarm_path_planner",
                executable="ugv_controller",
                namespace=ugv_name,
                name="ugv_controller",
                output="screen"
            )
        )

        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"{ugv_name}_map_to_odom_tf",
                arguments=[
                    "0", "0", "0",
                    "0", "0", "0",
                    "map",
                    f"{ugv_name}/odom"
                ],
                output="screen"
            )
        )

    return nodes


def generate_launch_description():

    declare_num = DeclareLaunchArgument(
        'num_ugvs',
        default_value='3'
    )

    rviz_config_path = os.path.expanduser(
        "~/swarm_robots_ws/src/ugv_nav/rviz/ugv_path_planning.rviz"
    )

    map_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("map_provider"),
                "launch",
                "map_provider.launch.py"
            )
        )
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        declare_num,
        map_provider,
        OpaqueFunction(function=launch_setup),
        rviz2
    ])
