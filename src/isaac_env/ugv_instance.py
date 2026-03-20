# UGV Instance class - import after SimulationApp is created
import os
import numpy as np
from scipy.spatial.transform import Rotation as R

from pxr import UsdPhysics, UsdGeom, Usd

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros

from drone_instance import euler_to_quat


class UGVInstance:
    """UGV (ground vehicle) instance with differential drive and carry/drop support."""

    def __init__(self, world, ros_node, name, initial_position, initial_orientation, asset_path, stage):

        self.name = name
        self.namespace = f"/{name}"
        self.stage = stage
        self.ros_node = ros_node

        self.scale = 1.0
        self.wheel_radius = 0.04
        self.wheel_base = 0.12

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # Carry/drop state
        self.attached = True
        self.carrier_drone = None
        self.carry_offset = np.array([0.6462, 0.0400, 0.0441])
        self.released = False

        # Spawn USD
        self.prim_path = f"/World/{name}"
        add_reference_to_stage(asset_path, self.prim_path)

        prim = stage.GetPrimAtPath(self.prim_path)
        prim.GetAttribute("xformOp:scale").Set((self.scale, self.scale, self.scale))

        self.articulation = Articulation(
            prim_paths_expr=self.prim_path,
            name=name
        )

        # Disable collision while carried (re-enabled on drop)
        self._set_collision_enabled(False)

        # Apply DriveAPI for wheel joints
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(p, "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(1e4)
                drive.CreateMaxForceAttr(1e6)

        self.odom_pub = ros_node.create_publisher(
            Odometry,
            f"{self.namespace}/odom",
            10
        )

        ros_node.create_subscription(
            Twist,
            f"{self.namespace}/cmd_vel",
            self.cmd_callback,
            10
        )

        # Drop command subscription
        ros_node.create_subscription(
            Bool,
            f"{self.namespace}/drop",
            self.drop_callback,
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)

        self._dof_resolved = False
        self._right_indices = []
        self._left_indices = []

    def _set_collision_enabled(self, enabled):
        """Enable/disable all collision shapes in the UGV hierarchy."""
        root = self.stage.GetPrimAtPath(self.prim_path)
        for p in Usd.PrimRange(root):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                col = UsdPhysics.CollisionAPI(p)
                col.GetCollisionEnabledAttr().Set(enabled)

    def drop_callback(self, msg):
        """Handle drop command from mission client."""
        if msg.data and self.attached:
            self.release()

    def release(self):
        """Transition from carried to independent physics-driven mode."""
        self.attached = False
        self.released = True
        self.carrier_drone = None
        self._set_collision_enabled(True)
        self.ros_node.get_logger().info(f"[{self.name}] RELEASED from drone")

    def initialize_after_reset(self, initial_position, initial_orientation):
        """Initialize UGV state after world reset."""
        pos = np.array([initial_position], dtype=np.float32) / get_stage_units()
        # initial_orientation is [qx, qy, qz, qw] → Isaac wants [qw, qx, qy, qz]
        ori = np.array([[
            initial_orientation[3],
            initial_orientation[0],
            initial_orientation[1],
            initial_orientation[2],
        ]], dtype=np.float32)
        self.articulation.set_world_poses(positions=pos, orientations=ori)

        # No kinematic toggle needed — carried UGVs are teleported before physics step

    def cmd_callback(self, msg):
        if self.attached:
            return
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def compute_wheel_velocities(self):
        v = self.linear_velocity
        w = self.angular_velocity

        v_left = (v - (self.wheel_base / 2.0) * w) / self.wheel_radius
        v_right = (v + (self.wheel_base / 2.0) * w) / self.wheel_radius
        return v_left, v_right

    def _resolve_dof_mapping(self):
        """Resolve DOF indices for left/right wheels from DOF names."""
        dof_names = list(self.articulation.dof_names)
        self.ros_node.get_logger().info(
            f"[{self.name}] DOF names: {dof_names}"
        )

        # From URDF joint origins:
        #   Revolute_1 (y<0) = Front Right
        #   Revolute_2 (y<0) = Rear Right
        #   Revolute_3 (y>0) = Front Left
        #   Revolute_4 (y>0) = Rear Left
        right_keywords = ['Revolute_1', 'Revolute_2']
        left_keywords = ['Revolute_3', 'Revolute_4']

        for i, name in enumerate(dof_names):
            if any(k in name for k in right_keywords):
                self._right_indices.append(i)
            elif any(k in name for k in left_keywords):
                self._left_indices.append(i)

        self.ros_node.get_logger().info(
            f"[{self.name}] Right DOFs (idx {self._right_indices}), "
            f"Left DOFs (idx {self._left_indices})"
        )

    def update_carried(self):
        """Update UGV position to track underneath the carrier drone."""
        if self.carrier_drone is None:
            return

        drone_pos = self.carrier_drone.position.copy()
        # Use drone's visual yaw (including offset) for correct positioning
        drone_visual_yaw = self.carrier_drone.yaw + self.carrier_drone.yaw_offset

        # Rotate carry offset by drone's visual yaw into world frame
        cos_y = np.cos(drone_visual_yaw)
        sin_y = np.sin(drone_visual_yaw)
        rotated_offset = np.array([
            self.carry_offset[0] * cos_y - self.carry_offset[1] * sin_y,
            self.carry_offset[0] * sin_y + self.carry_offset[1] * cos_y,
            self.carry_offset[2]
        ])

        carried_pos = drone_pos + rotated_offset

        # Match drone visual orientation
        qw, qx, qy, qz = euler_to_quat(0.0, 0.0, drone_visual_yaw)

        positions = np.array([carried_pos], dtype=np.float32) / get_stage_units()
        orientations = np.array([[qw, qx, qy, qz]], dtype=np.float32)
        self.articulation.set_world_poses(positions=positions, orientations=orientations)

        # Zero out velocities to prevent accumulation from gravity
        self.articulation.set_linear_velocities(np.zeros((1, 3), dtype=np.float32))
        self.articulation.set_angular_velocities(np.zeros((1, 3), dtype=np.float32))

    def update(self):
        """Dispatch to carried or ground update based on state."""
        if self.attached:
            self.update_carried()
            return

        if self.released:
            # One-time transition: just clear the flag and zero velocities
            self.released = False
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            return

        self._update_ground()

    def _update_ground(self):
        """Apply wheel velocities for 4-wheel rover (differential drive)."""
        if not self._dof_resolved:
            self._resolve_dof_mapping()
            self._dof_resolved = True

        v_left, v_right = self.compute_wheel_velocities()

        num_dofs = self.articulation.num_dof
        joint_vel = np.zeros((1, num_dofs), dtype=np.float32)
        for i in self._right_indices:
            joint_vel[0, i] = v_right
        for i in self._left_indices:
            joint_vel[0, i] = v_left
        self.articulation.set_joint_velocity_targets(joint_vel)

    def publish_odometry(self):
        """Publish odometry and TF."""
        positions, orientations = self.articulation.get_world_poses()
        pos, quat_isaac = positions[0], orientations[0]

        lin_vel_world = self.articulation.get_linear_velocities()[0]
        ang_vel_world = self.articulation.get_angular_velocities()[0]

        # Isaac (w,x,y,z) → ROS (x,y,z,w)
        quat_ros = [
            quat_isaac[1],
            quat_isaac[2],
            quat_isaac[3],
            quat_isaac[0],
        ]

        # Guard against zero-norm quaternion (uninitialized articulation)
        quat_norm = np.linalg.norm(quat_ros)
        if quat_norm < 1e-6:
            quat_ros = [0.0, 0.0, 0.0, 1.0]

        rot = R.from_quat(quat_ros)
        lin_vel_local = rot.inv().apply(lin_vel_world)
        ang_vel_local = rot.inv().apply(ang_vel_world)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        odom_msg.header.frame_id = f"{self.name}/odom"
        odom_msg.child_frame_id = f"{self.name}/base_link"

        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])

        odom_msg.pose.pose.orientation.x = float(quat_ros[0])
        odom_msg.pose.pose.orientation.y = float(quat_ros[1])
        odom_msg.pose.pose.orientation.z = float(quat_ros[2])
        odom_msg.pose.pose.orientation.w = float(quat_ros[3])

        odom_msg.twist.twist.linear.x = float(lin_vel_local[0])
        odom_msg.twist.twist.angular.z = float(ang_vel_local[2])

        self.odom_pub.publish(odom_msg)

        # TF broadcast
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)
