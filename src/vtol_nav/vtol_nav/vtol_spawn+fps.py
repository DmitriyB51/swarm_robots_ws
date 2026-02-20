from isaacsim import SimulationApp

# [OPT 1] RaytracedLighting + lower resolution for better FPS
simulation_app = SimulationApp({
    "headless": False,
    "renderer": "RaytracedLighting",
    "width": 1280,
    "height": 720,
})

import sys
import os
import time
import numpy as np
import carb

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension

from pxr import UsdPhysics, Usd, UsdLux

import rclpy
from geometry_msgs.msg import PoseStamped, Twist


enable_extension("isaacsim.asset.importer.urdf")
# [OPT 4] Removed omni.physx.supportui and omni.physx.ui (debug UI extensions that cost resources)
simulation_app.update()

# [OPT 2] DLSS Performance mode (AI upscaling) + [OPT 6] FXAA (cheapest anti-aliasing)
carb.settings.get_settings().set("/rtx/post/dlss/execMode", 0)  # 0=Performance
carb.settings.get_settings().set("/rtx/post/aa/op", 2)  # 2=FXAA


# world setup
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# [OPT 3] Render at 30 FPS, physics at 60Hz
my_world = World(stage_units_in_meters=1.0, physics_dt=1.0/60.0, rendering_dt=1.0/30.0)
my_world.scene.add_default_ground_plane()

set_camera_view(
    eye=[-10, 0.0, 10],
    target=[0.0, 0.0, 0.0],
    camera_prim_path="/OmniverseKit_Persp"
)

stage = my_world.stage

dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome_light.CreateIntensityAttr(500)

asset_path = os.path.expanduser(
    "~/swarm_robots_ws/src/vtol_dron_description/urdf/vtol_simple/vtol_simple.usd"
)


def euler_to_quat(roll, pitch, yaw):
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class DroneInstance:

    def __init__(self, world, ros_node, name, index, asset_path):

        self.name = name
        self.namespace = f"/{name}"

        self.position = np.array([index * 3.0, 0.0, 0.0], dtype=float)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        self.cmd_vel = np.zeros(3)
        self.cmd_ang = np.zeros(3)

        # Spawn USD
        self.prim_path = f"/World/{name}"
        add_reference_to_stage(asset_path, self.prim_path)

        self.articulation = Articulation(
            prim_paths_expr=self.prim_path,
            name=name
        )

        prim = stage.GetPrimAtPath(self.prim_path)

        # Kinematic base
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rigid_body = UsdPhysics.RigidBodyAPI(prim)
            rigid_body.CreateKinematicEnabledAttr(True)

        # Apply DriveAPI
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(p, "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(1e2)
                drive.CreateMaxForceAttr(1e6)

        self.pose_pub = ros_node.create_publisher(
            PoseStamped,
            f"{self.namespace}/pose",
            10
        )

        ros_node.create_subscription(
            Twist,
            f"{self.namespace}/cmd_vel",
            self.cmd_callback,
            10
        )

        self.num_dofs = None
        self.num_rotors = None
        self.base_spin_vel = None
        self.rotor_angles = None

    def initialize_after_reset(self):

        self.num_dofs = self.articulation.num_dof
        self.num_rotors = min(4, self.num_dofs)

        self.base_spin_vel = np.array(
            [80.0, -80.0, 80.0, -80.0],
            dtype=np.float32
        )

        self.rotor_angles = np.zeros(
            self.num_rotors,
            dtype=np.float32
        )

        self.articulation.set_world_poses(
            positions=np.array([self.position]) / get_stage_units()
        )

    def cmd_callback(self, msg):
        self.cmd_vel[:] = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_ang[:] = [msg.angular.x, msg.angular.y, msg.angular.z]

    def update(self, dt, stage_units):

        # Update pose
        self.position += self.cmd_vel * dt
        self.roll  += self.cmd_ang[0] * dt
        self.pitch += self.cmd_ang[1] * dt
        self.yaw   += self.cmd_ang[2] * dt

        qw, qx, qy, qz = euler_to_quat(self.roll, self.pitch, self.yaw)
        orientation = np.array([[qw, qx, qy, qz]], dtype=np.float32)
        positions = (self.position / stage_units)[None, :]

        self.articulation.set_world_poses(
            positions=positions,
            orientations=orientation
        )

        # propeller spinning viz effect
        rotor_omega = self.base_spin_vel[:self.num_rotors]
        self.rotor_angles += rotor_omega * dt
        self.rotor_angles = (self.rotor_angles + np.pi) % (2*np.pi) - np.pi

        joint_pos = np.zeros((1, self.num_dofs), dtype=np.float32)
        joint_pos[0, :self.num_rotors] = self.rotor_angles
        self.articulation.set_joint_positions(joint_pos)

        joint_vel = np.zeros((1, self.num_dofs), dtype=np.float32)
        joint_vel[0, :self.num_rotors] = rotor_omega
        self.articulation.set_joint_velocities(joint_vel)

    def publish_pose(self, ros_node):
        pos, ori = self.articulation.get_world_poses()

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = ros_node.get_clock().now().to_msg()

        msg.pose.position.x = float(pos[0][0])
        msg.pose.position.y = float(pos[0][1])
        msg.pose.position.z = float(pos[0][2])
        msg.pose.orientation.w = float(ori[0][0])
        msg.pose.orientation.x = float(ori[0][1])
        msg.pose.orientation.y = float(ori[0][2])
        msg.pose.orientation.z = float(ori[0][3])

        self.pose_pub.publish(msg)


rclpy.init()
ros_node = rclpy.create_node("swarm_controller")

NUM_DRONES = 3
drones = []

for i in range(NUM_DRONES):
    drones.append(
        DroneInstance(
            my_world,
            ros_node,
            f"vtol_{i+1}",
            i,
            asset_path
        )
    )

my_world.reset()

for drone in drones:
    drone.initialize_after_reset()

stage_units = get_stage_units()
last_time = time.time()

from omni.kit.viewport.utility import get_active_viewport
viewport = get_active_viewport()
fps_print_time = time.time()

while simulation_app.is_running():

    now = time.time()
    dt = now - last_time
    last_time = now

    for drone in drones:
        drone.update(dt, stage_units)

    my_world.step(render=True)

    for drone in drones:
        drone.publish_pose(ros_node)

    rclpy.spin_once(ros_node, timeout_sec=0.0)

    # Print simulation FPS once per second
    if time.time() - fps_print_time >= 1.0:
        print(f"SIM FPS: {viewport.fps:.1f}")
        fps_print_time = time.time()


ros_node.destroy_node()
rclpy.shutdown()
simulation_app.close()
