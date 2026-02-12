#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # === RViz Config Path ===
    rviz_config_path = os.path.expanduser(
        "~/swarm_robots_ws/src/ugv_nav/rviz/ugv_path_planning.rviz"
    )

    # === Map Provider Launch ===
    map_provider = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("map_provider"),
                "launch",
                "map_provider.launch.py"
            )
        )
    )

    # === Path Planner Node ===
    path_planner = Node(
        package="ugv_swarm_path_planner",
        executable="astar_planner_fixed_pose",
        name="astar_planner_fixed_pose",
        output="screen"
    )

    # === RViz2 ===
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_path],
        output="screen"
    )

    return LaunchDescription([
        map_provider,
        path_planner,
        rviz2
    ])
