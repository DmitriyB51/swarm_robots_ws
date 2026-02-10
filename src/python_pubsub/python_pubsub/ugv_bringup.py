# Copyright (c) 2020-2024, NVIDIA CORPORATION.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

import sys
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.storage.native import get_assets_root_path


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist



class JetbotROSNode(Node):
    def __init__(self):
        super().__init__("jetbot_ros_node")

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseStamped,
            "ugv/pose",
            10
        )

        # Subscriber
        self.cmd_sub = self.create_subscription(
            Twist,
            "ugv/cmd_vel",
            self.cmd_vel_callback,
            10
        )

        self.linear_x = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg: Twist):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z


assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit(1)

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()


jetbot_usd = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
add_reference_to_stage(jetbot_usd, "/World/Jetbot")

jetbot = Articulation(
    prim_paths_expr="/World/Jetbot",
    name="my_jetbot"
)

# jetbot.set_world_poses(
#     position=np.array([3.5, -6.5, 0.0]),
#     orientation=np.array([0.0, 0.0, 0.0, 0.1])
# )

jetbot.set_world_poses(positions=np.array([[3.5, -6.5, 0.0]]) / get_stage_units())


my_world.reset()


rclpy.init()
ros_node = JetbotROSNode()

print("Simulation running...")


WHEEL_RADIUS = 0.03    # meters
WHEEL_BASE = 0.1125  


while simulation_app.is_running():

    my_world.step(render=True)
    rclpy.spin_once(ros_node, timeout_sec=0.0)

    if not my_world.is_playing():
        continue


    positions, orientations = jetbot.get_world_poses()

    # Root link pose
    pos = positions[0]        # shape (3,)
    rot = orientations[0]     # shape (4,)


    # publisher
    pose_msg = PoseStamped()
    pose_msg.header.stamp = ros_node.get_clock().now().to_msg()
    pose_msg.header.frame_id = "world"

    pose_msg.pose.position.x = float(pos[0])
    pose_msg.pose.position.y = float(pos[1])
    pose_msg.pose.position.z = float(pos[2])

    pose_msg.pose.orientation.x = float(rot[0])
    pose_msg.pose.orientation.y = float(rot[1])
    pose_msg.pose.orientation.z = float(rot[2])
    pose_msg.pose.orientation.w = float(rot[3])

    ros_node.pose_pub.publish(pose_msg)

    # subscriber
    v = ros_node.linear_x
    w = ros_node.angular_z

    left_wheel_vel = (v - w * WHEEL_BASE / 2.0) / WHEEL_RADIUS
    right_wheel_vel = (v + w * WHEEL_BASE / 2.0) / WHEEL_RADIUS

    wheel_vels = np.array([[left_wheel_vel, right_wheel_vel]])
    jetbot.set_joint_velocities(wheel_vels)

# Cleanup
ros_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
