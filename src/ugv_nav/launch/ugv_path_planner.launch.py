import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

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

    path_planner = Node(
        package="ugv_swarm_path_planner",
        executable="astar_planner_fixed_pose",
        name="astar_planner_fixed_pose",
        output="screen"
    )

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
