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

import os

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
    eye=[-7, 0.0, 7], target=[0.00, 0.00, 0.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view




# asset_path = assets_root_path + "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd"
# asset_path = "/home/qasob/swarm_robots_ws/src/robots/vtol_swarm_config.usd"
asset_path = os.path.expanduser("~/swarm_robots_ws/src/vtol_dron_description/urdf/vtol_simple/vtol_simple.usd")



add_reference_to_stage(usd_path=asset_path, prim_path="/World/vtol_drone")

# Add DriveAPI to all joints before reset
from pxr import UsdPhysics, Usd

stage = my_world.stage

# Add scene lighting
from pxr import UsdLux
dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(500)


drone_prim = stage.GetPrimAtPath("/World/vtol_drone")






# Disable physics on the base - движем дрон сами, не физикой
if drone_prim and drone_prim.HasAPI(UsdPhysics.RigidBodyAPI):
    rigid_body = UsdPhysics.RigidBodyAPI(drone_prim)
    rigid_body.CreateKinematicEnabledAttr(True)
    print("Base set to kinematic mode (physics disabled)")






for prim in Usd.PrimRange(drone_prim):
    if prim.IsA(UsdPhysics.RevoluteJoint):
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(1e2)
        drive.CreateMaxForceAttr(1e6)
        print(f"DriveAPI added to: {prim.GetPath()}")




drone = Articulation(prim_paths_expr="/World/vtol_drone", name="my_drone")





drone.set_world_poses(positions=np.array([[0.0, 0, 0.0]]) / get_stage_units())


# initialize the world
my_world.reset()

# Remove default SphereLight after world is initialized
stage.RemovePrim("/World/defaultGroundPlane/SphereLight")

num_dofs = drone.num_dof

# Current drone position (you control this)
position = np.array([0.0, 0.0, 0.0], dtype=float)  # Start at 0.5m height



# Velocity commands
cmd_vel_x = 0.0
cmd_vel_y = 0.0
cmd_vel_z = 0.0  # m/s

# Current orientation (Euler angles in radians)
roll = 0.0   # Rotation around X-axis (tilt left/right)
pitch = 0.0  # Rotation around Y-axis (tilt forward/back)
yaw = 0.0    # Rotation around Z-axis (spin left/right)

# Angular velocities (rad/s)
cmd_angular_x = 0.0  # Roll
cmd_angular_y = 0.0  # Pitch
cmd_angular_z = 0.0  # Yaw

dt = 1.0 / 60.0  # Time





# Propeller spinning (visual effect)
base_spin_vel = np.array([80.0, -80.0, 80.0, -80.0], dtype=np.float32)
num_rotors = min(4, num_dofs)  # 4 rotors
rotor_angles = np.zeros(num_rotors, dtype=np.float32)  # Current angles

def euler_to_quat(roll, pitch, yaw):
    """Convert Euler angles to quaternion [w, x, y, z]"""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qw, qx, qy, qz





def cmd_vel_callback(msg):
    global cmd_vel_x, cmd_vel_y, cmd_vel_z
    global cmd_angular_x, cmd_angular_y, cmd_angular_z

    # Linear velocities
    cmd_vel_x = msg.linear.x
    cmd_vel_y = msg.linear.y
    cmd_vel_z = msg.linear.z

    # Angular velocities
    cmd_angular_x = msg.angular.x  # Roll 
    cmd_angular_y = msg.angular.y  # Pitch 
    cmd_angular_z = msg.angular.z  # Yaw   

    


# ros2 pub sub
rclpy.init()
ros_node = rclpy.create_node("drone_controller")
pose_publisher = ros_node.create_publisher(PoseStamped, "/drone/pose", 10)
cmd_vel_subscriber = ros_node.create_subscription(Twist, "/cmd_vel", cmd_vel_callback, 10)





stage_units = get_stage_units()

while simulation_app.is_running():

    # Update position
    position[0] += cmd_vel_x * dt
    position[1] += cmd_vel_y * dt
    position[2] += cmd_vel_z * dt

    # Update orientation
    roll += cmd_angular_x * dt   # new roll
    pitch += cmd_angular_y * dt  # new pitch
    yaw += cmd_angular_z * dt    # new yaw

    # Floor limit
    if position[2] < 0.0:
        position[2] = 0.0

    # Convert Euler to quaternion
    qw, qx, qy, qz = euler_to_quat(roll, pitch, yaw)
    orientation = np.array([[qw, qx, qy, qz]], dtype=np.float32)

    # Apply pose
    positions = (position / stage_units)[None, :]
    drone.set_world_poses(positions=positions, orientations=orientation)

    # Spin propellers continuously (visual only)
    rotor_omega = base_spin_vel[:num_rotors]  # Constant speed
    rotor_angles += rotor_omega * dt  # Update angles
    rotor_angles = (rotor_angles + np.pi) % (2 * np.pi) - np.pi  # Wrap to [-π, π]

    # Set propeller positions and velocities
    joint_pos = np.zeros((1, num_dofs), dtype=np.float32)
    joint_pos[0, :num_rotors] = rotor_angles
    drone.set_joint_positions(joint_pos)

    joint_vel = np.zeros((1, num_dofs), dtype=np.float32)
    joint_vel[0, :num_rotors] = rotor_omega
    drone.set_joint_velocities(joint_vel)

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
