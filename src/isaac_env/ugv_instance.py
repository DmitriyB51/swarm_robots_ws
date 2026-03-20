# UGV Instance class - import after SimulationApp is created
import numpy as np
from scipy.spatial.transform import Rotation as R

from pxr import UsdPhysics, UsdGeom, Usd, Gf

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import tf2_ros

from drone_instance import euler_to_quat


class UGVInstance:
    """UGV instance: visual child of drone while carried, real articulation after drop."""

    def __init__(self, world, ros_node, name, initial_position, initial_orientation,
                 asset_path, stage):

        self.name = name
        self.namespace = f"/{name}"
        self.stage = stage
        self.ros_node = ros_node
        self.asset_path = asset_path

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

        # ---- Real physics UGV (hidden while carried) ----
        self.prim_path = f"/World/{name}"
        add_reference_to_stage(asset_path, self.prim_path)

        prim = stage.GetPrimAtPath(self.prim_path)
        prim.GetAttribute("xformOp:scale").Set((self.scale, self.scale, self.scale))

        self.articulation = Articulation(
            prim_paths_expr=self.prim_path,
            name=name
        )

        # Apply DriveAPI for wheel joints
        for p in Usd.PrimRange(prim):
            if p.IsA(UsdPhysics.RevoluteJoint):
                drive = UsdPhysics.DriveAPI.Apply(p, "angular")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(1e4)
                drive.CreateMaxForceAttr(1e6)

        # Hide real UGV — it stays hidden until drop
        UsdGeom.Imageable(prim).MakeInvisible()

        # Disable collision on hidden UGV so it doesn't affect anything
        for p in Usd.PrimRange(prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

        # ---- Visual-only carried UGV (created in setup_carry_visual) ----
        self._visual_path = None

        # ---- ROS ----
        self.odom_pub = ros_node.create_publisher(
            Odometry, f"{self.namespace}/odom", 10
        )
        ros_node.create_subscription(
            Twist, f"{self.namespace}/cmd_vel", self.cmd_callback, 10
        )
        ros_node.create_subscription(
            Bool, f"{self.namespace}/drop", self.drop_callback, 10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)

        self._dof_resolved = False
        self._right_indices = []
        self._left_indices = []

    def setup_carry_visual(self, drone):
        """Create a visual-only UGV as a child of the drone prim.

        Because it's a child in the USD scene graph, it inherits the
        drone's world transform automatically. No teleporting needed.
        """
        self.carrier_drone = drone
        self._visual_path = f"{drone.prim_path}/base_link/carried_{self.name}"
        add_reference_to_stage(self.asset_path, self._visual_path)

        vis_prim = self.stage.GetPrimAtPath(self._visual_path)
        vis_prim.GetAttribute("xformOp:scale").Set(
            (self.scale, self.scale, self.scale)
        )

        # Set local offset relative to drone
        vis_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(float(self.carry_offset[0]),
                      float(self.carry_offset[1]),
                      float(self.carry_offset[2]))
        )

        # Strip ALL physics from the visual copy
        # First collect joint prims to delete (can't delete while iterating)
        joints_to_remove = []
        for p in Usd.PrimRange(vis_prim):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                p.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if p.HasAPI(UsdPhysics.CollisionAPI):
                p.RemoveAPI(UsdPhysics.CollisionAPI)
            if p.IsA(UsdPhysics.Joint):
                joints_to_remove.append(p.GetPath())
        # Delete joint prims entirely so PhysX doesn't try to create them
        for joint_path in joints_to_remove:
            self.stage.RemovePrim(joint_path)

    def drop_callback(self, msg):
        if msg.data and self.attached:
            self.release()

    def release(self):
        """Drop: hide visual, show real UGV at visual's current world position."""
        self.attached = False
        self.released = True

        # Read the visual prim's computed world transform BEFORE hiding it
        vis_prim = self.stage.GetPrimAtPath(self._visual_path)
        xform_cache = UsdGeom.XformCache()
        world_xform = xform_cache.GetLocalToWorldTransform(vis_prim)
        drop_pos = world_xform.ExtractTranslation()
        drop_rot = world_xform.ExtractRotationQuat()
        # Gf.Quatd → (w, x, y, z) for Isaac
        img = drop_rot.GetImaginary()
        qw, qx, qy, qz = drop_rot.GetReal(), img[0], img[1], img[2]

        self.ros_node.get_logger().info(
            f"[{self.name}] RELEASING at pos={drop_pos}, quat=({qw},{qx},{qy},{qz})"
        )

        self.carrier_drone = None

        # Hide visual
        UsdGeom.Imageable(vis_prim).MakeInvisible()

        # Show real UGV
        real_prim = self.stage.GetPrimAtPath(self.prim_path)
        UsdGeom.Imageable(real_prim).MakeVisible()

        # Re-enable collision
        for p in Usd.PrimRange(real_prim):
            if p.HasAPI(UsdPhysics.CollisionAPI):
                UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(True)

        # Teleport real UGV to where the visual was
        stage_units = get_stage_units()
        positions = np.array([[drop_pos[0], drop_pos[1], drop_pos[2]]], dtype=np.float32) / stage_units
        orientations = np.array([[qw, qx, qy, qz]], dtype=np.float32)
        self.articulation.set_world_poses(positions=positions, orientations=orientations)
        self.articulation.set_linear_velocities(np.zeros((1, 3), dtype=np.float32))
        self.articulation.set_angular_velocities(np.zeros((1, 3), dtype=np.float32))

        self.ros_node.get_logger().info(f"[{self.name}] RELEASED at {drop_pos}")

    def initialize_after_reset(self, initial_position, initial_orientation):
        """Initialize real UGV after world reset (stays hidden)."""
        pos = np.array([initial_position], dtype=np.float32) / get_stage_units()
        ori = np.array([[
            initial_orientation[3],
            initial_orientation[0],
            initial_orientation[1],
            initial_orientation[2],
        ]], dtype=np.float32)
        self.articulation.set_world_poses(positions=pos, orientations=ori)

        # Re-hide after reset (reset may restore visibility)
        if self.attached:
            real_prim = self.stage.GetPrimAtPath(self.prim_path)
            UsdGeom.Imageable(real_prim).MakeInvisible()
            for p in Usd.PrimRange(real_prim):
                if p.HasAPI(UsdPhysics.CollisionAPI):
                    UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Set(False)

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
        dof_names = list(self.articulation.dof_names)
        self.ros_node.get_logger().info(f"[{self.name}] DOF names: {dof_names}")

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

    def update(self):
        """Only used after drop for ground driving."""
        if self.attached:
            return  # Visual child handles position automatically

        if self.released:
            self.released = False
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            return

        self._update_ground()

    def _update_ground(self):
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
        if self.attached:
            # While carried, report drone position + offset
            if self.carrier_drone is None:
                return
            # Read world transform from visual prim (child of drone base_link)
            vis_prim = self.stage.GetPrimAtPath(self._visual_path)
            xform_cache = UsdGeom.XformCache()
            world_xform = xform_cache.GetLocalToWorldTransform(vis_prim)
            pos = world_xform.ExtractTranslation()
            drop_rot = world_xform.ExtractRotationQuat()
            img = drop_rot.GetImaginary()
            qw = drop_rot.GetReal()
            quat_ros = [img[0], img[1], img[2], qw]
            lin_vel_local = np.zeros(3)
            ang_vel_local = np.zeros(3)
        else:
            positions, orientations = self.articulation.get_world_poses()
            pos, quat_isaac = positions[0], orientations[0]
            lin_vel_world = self.articulation.get_linear_velocities()[0]
            ang_vel_world = self.articulation.get_angular_velocities()[0]
            quat_ros = [quat_isaac[1], quat_isaac[2], quat_isaac[3], quat_isaac[0]]
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

        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)
