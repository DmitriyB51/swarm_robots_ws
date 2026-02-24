# Drone Instance class - import after SimulationApp is created
import numpy as np

from pxr import UsdPhysics, Usd

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

from geometry_msgs.msg import PoseStamped, Twist


def euler_to_quat(roll, pitch, yaw):
    """Convert Euler angles to quaternion (w, x, y, z)."""
    cr, sr = np.cos(roll/2), np.sin(roll/2)
    cp, sp = np.cos(pitch/2), np.sin(pitch/2)
    cy, sy = np.cos(yaw/2), np.sin(yaw/2)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class DroneInstance:
    """VTOL Drone instance with kinematic control."""

    def __init__(self, world, ros_node, name, initial_position, asset_path, stage):

        self.name = name
        self.namespace = f"/{name}"
        self.stage = stage

        self.position = np.array(initial_position, dtype=float)
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

        # Apply DriveAPI for propellers
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
        """Initialize drone state after world reset."""
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
        """Update drone position and propeller animation."""
        # Update pose
        self.position += self.cmd_vel * dt
        self.roll += self.cmd_ang[0] * dt
        self.pitch += self.cmd_ang[1] * dt
        self.yaw += self.cmd_ang[2] * dt

        qw, qx, qy, qz = euler_to_quat(self.roll, self.pitch, self.yaw)
        orientation = np.array([[qw, qx, qy, qz]], dtype=np.float32)
        positions = (self.position / stage_units)[None, :]

        self.articulation.set_world_poses(
            positions=positions,
            orientations=orientation
        )

        # Propeller spinning visualization
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
        """Publish drone pose."""
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
