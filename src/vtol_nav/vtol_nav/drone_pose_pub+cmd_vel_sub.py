from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys

import carb
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension



# ROS2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist


# Enable extensions
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("omni.physx.supportui")
enable_extension("omni.physx.ui")
simulation_app.update()


# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()



my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[0.5, 0.0, 0.5], target=[0.00, 0.00, 0.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view




asset_path = assets_root_path + "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd"



add_reference_to_stage(usd_path=asset_path, prim_path="/World/drone")

# Add DriveAPI to all joints before reset
from pxr import UsdPhysics, Usd

stage = my_world.stage
drone_prim = stage.GetPrimAtPath("/World/drone")
for prim in Usd.PrimRange(drone_prim):
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1e2)
        drive.CreateMaxForceAttr(1e6)
        print(f"DriveAPI added to: {prim.GetPath()}")




drone = Articulation(prim_paths_expr="/World/drone", name="my_drone")





drone.set_world_poses(positions=np.array([[0.0, 0, 0.0]]) / get_stage_units())


# initialize the world
my_world.reset()


num_dofs = drone.num_dof


# Motor speeds — updated by cmd_vel callback
target_velocities = np.array([0.0, 0.0, 0.0, 0.0])


# cmd_vel callback: converts Twist message to 4 motor speeds
def cmd_vel_callback(msg):
    global target_velocities
    thrust = msg.linear.z     
    roll   = msg.angular.x    
    pitch  = msg.angular.y    
    yaw    = msg.angular.z    

    # convert Twist to individual motor speeds
    
    m1 =  (thrust + pitch + roll + yaw)
    m2 = -(thrust - pitch + roll - yaw)
    m3 =  (thrust + pitch - roll - yaw)
    m4 = -(thrust - pitch - roll + yaw)

    target_velocities = np.array([m1, m2, m3, m4])


# ros2 pub sub
rclpy.init()
ros_node = rclpy.create_node("drone_controller")
pose_publisher = ros_node.create_publisher(PoseStamped, "/robot_pose", 10)
cmd_vel_subscriber = ros_node.create_subscription(Twist, "/cmd_vel", cmd_vel_callback, 10)





while simulation_app.is_running():
    drone.set_joint_velocity_targets(target_velocities)
    my_world.step(render=True)

    
    positions, orientations = drone.get_world_poses()

    # rounding for readability
    pos = np.round(positions[0], 4)
    ori = np.round(orientations[0], 4)

    # Publish PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "world"
    pose_msg.header.stamp = ros_node.get_clock().now().to_msg()
    pose_msg.pose.position.x = float(pos[0])
    pose_msg.pose.position.y = float(pos[1])
    pose_msg.pose.position.z = float(pos[2])
    pose_msg.pose.orientation.w = float(ori[0])
    pose_msg.pose.orientation.x = float(ori[1])
    pose_msg.pose.orientation.y = float(ori[2])
    pose_msg.pose.orientation.z = float(ori[3])
    pose_publisher.publish(pose_msg)

   
    rclpy.spin_once(ros_node, timeout_sec=0.0)


ros_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
