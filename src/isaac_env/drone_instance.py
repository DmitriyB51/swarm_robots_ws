# Drone Instance class - import after SimulationApp is created
import numpy as np

from pxr import UsdPhysics, Usd, UsdGeom, UsdShade, Sdf, Gf

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool


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


FOV_RADIUS = 3.0   # meters — matches detection_radius in mission scripts
FOV_NOSE_OFFSET = 0.3  # meters — shift apex backward toward drone body


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
        self.yaw_offset = np.pi  # Rotate mesh 180° so nose faces forward

        self.cmd_vel = np.zeros(3)
        self.cmd_ang = np.zeros(3)

        # X/Y-axis velocity dynamics (lighter lag than Z)
        self.actual_vxy = np.zeros(2)  # current actual horizontal velocity (m/s)
        self.tau_xy = 0.35             # horizontal lag time constant (seconds)
        self.max_xy_accel = 2.0        # max horizontal acceleration (m/s^2)

        # Z-axis first-order velocity dynamics (from real flight data)
        self.actual_vz = 0.0           # current actual vertical velocity (m/s)
        self.tau_z = 0.6               # first-order lag time constant (seconds)
        self.max_climb_rate = 3.3      # max upward velocity (m/s)
        self.max_descent_rate = 2.6    # max downward velocity (m/s, stored positive)
        self.max_z_accel = 0.85        # max vertical acceleration (m/s^2)
        self.ground_z = 0.25           # minimum altitude (meters)

        # Angular rate smoothing (simulates rotational inertia)
        self.smooth_ang = np.zeros(3)
        self.tau_ang = 0.2             # angular rate filter time constant (seconds)

        self.carried_ugv = None  # Reference to carried UGV, set by combined_spawn
        self._fov_curves = None  # BasisCurves prim for FOV visualization
        self._fov_visible = False

        # Spawn USD
        self.prim_path = f"/World/{name}"
        add_reference_to_stage(asset_path, self.prim_path)

        self.articulation = Articulation(
            prim_paths_expr=self.prim_path,
            name=name
        )

        prim = stage.GetPrimAtPath(self.prim_path)

        # Kinematic base (original logic — root prim check)
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

        ros_node.create_subscription(
            Bool, '/show_fov',
            self._show_fov_callback,
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

        # Tail propeller (5th joint) — spins only during horizontal movement
        self.has_tail_prop = self.num_dofs > 4
        self.tail_angle = 0.0
        self.tail_max_spin = 80.0      # max spin speed (rad/s), same as main rotors

        self.articulation.set_world_poses(
            positions=np.array([self.position]) / get_stage_units()
        )

    def setup_fov_lines(self):
        """Create BasisCurves pyramid showing the drone's detection cone."""
        R = FOV_RADIUS
        cx, cy, cz = self.position

        # 4 ground corners
        corners = [
            (cx + R, cy + R, 0.0),
            (cx + R, cy - R, 0.0),
            (cx - R, cy - R, 0.0),
            (cx - R, cy + R, 0.0),
        ]

        # 8 line segments: 4 center→corner + 4 corner→corner
        points = []
        for corner in corners:
            points.append(Gf.Vec3f(cx, cy, cz))
            points.append(Gf.Vec3f(*corner))
        for i in range(4):
            points.append(Gf.Vec3f(*corners[i]))
            points.append(Gf.Vec3f(*corners[(i + 1) % 4]))

        fov_path = f"/World/FOV_{self.name}"
        curves = UsdGeom.BasisCurves.Define(self.stage, fov_path)
        curves.GetTypeAttr().Set("linear")
        curves.GetWrapAttr().Set("nonperiodic")
        curves.GetCurveVertexCountsAttr().Set([2] * 8)
        curves.GetPointsAttr().Set(points)
        curves.GetWidthsAttr().Set([0.04] * 16)

        # Cyan emissive material
        mat_path = f"/World/Looks/FOV_{self.name}_Mat"
        mat = UsdShade.Material.Define(self.stage, mat_path)
        sh = UsdShade.Shader.Define(self.stage, f"{mat_path}/Shader")
        sh.SetSourceAsset("OmniPBR.mdl", "mdl")
        sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
        sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(0.0, 0.9, 1.0)
        )
        sh.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
        sh.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI(curves.GetPrim()).Bind(mat)

        self._fov_curves = curves
        UsdGeom.Imageable(curves.GetPrim()).MakeInvisible()

    def _show_fov_callback(self, msg):
        if msg.data and self._fov_curves is not None and not self._fov_visible:
            UsdGeom.Imageable(self._fov_curves.GetPrim()).MakeVisible()
            self._fov_visible = True

    def _update_fov_points(self):
        """Recompute FOV pyramid points from current drone position."""
        if self._fov_curves is None:
            return
        R = FOV_RADIUS
        # Shift apex backward along heading so it sits at the drone nose
        cx = float(self.position[0]) - FOV_NOSE_OFFSET * np.cos(self.yaw)
        cy = float(self.position[1]) - FOV_NOSE_OFFSET * np.sin(self.yaw)
        cz = float(self.position[2])

        corners = [
            (cx + R, cy + R, 0.0),
            (cx + R, cy - R, 0.0),
            (cx - R, cy - R, 0.0),
            (cx - R, cy + R, 0.0),
        ]

        points = []
        for corner in corners:
            points.append(Gf.Vec3f(cx, cy, cz))
            points.append(Gf.Vec3f(*corner))
        for i in range(4):
            points.append(Gf.Vec3f(*corners[i]))
            points.append(Gf.Vec3f(*corners[(i + 1) % 4]))

        self._fov_curves.GetPointsAttr().Set(points)

    def cmd_callback(self, msg):
        self.cmd_vel[:] = [msg.linear.x, msg.linear.y, msg.linear.z]
        self.cmd_ang[:] = [msg.angular.x, msg.angular.y, msg.angular.z]

    def update(self, dt, stage_units):
        """Update drone position and propeller animation."""
        # X/Y: first-order velocity lag (lighter than Z)
        for i in range(2):
            vxy_error = self.cmd_vel[i] - self.actual_vxy[i]
            vxy_dot = vxy_error / self.tau_xy
            vxy_dot = max(-self.max_xy_accel, min(self.max_xy_accel, vxy_dot))
            self.actual_vxy[i] += vxy_dot * dt
        self.position[0] += self.actual_vxy[0] * dt
        self.position[1] += self.actual_vxy[1] * dt

        # Z: first-order velocity lag with acceleration and velocity limits
        vz_error = self.cmd_vel[2] - self.actual_vz
        vz_dot = vz_error / self.tau_z
        vz_dot = max(-self.max_z_accel, min(self.max_z_accel, vz_dot))
        self.actual_vz += vz_dot * dt
        self.actual_vz = max(-self.max_descent_rate, min(self.max_climb_rate, self.actual_vz))

        # Ground collision: stop descending at ground level
        if self.position[2] <= self.ground_z and self.actual_vz < 0.0:
            self.actual_vz = 0.0

        self.position[2] += self.actual_vz * dt

        if self.position[2] < self.ground_z:
            self.position[2] = self.ground_z
            self.actual_vz = 0.0
        # Smooth angular rates (low-pass filter to reduce oscillation/jitter)
        alpha_ang = min(dt / self.tau_ang, 1.0)
        self.smooth_ang += (self.cmd_ang - self.smooth_ang) * alpha_ang
        self.roll += self.smooth_ang[0] * dt
        self.pitch += self.smooth_ang[1] * dt
        self.yaw += self.smooth_ang[2] * dt

        qw, qx, qy, qz = euler_to_quat(self.roll, self.pitch, self.yaw + self.yaw_offset)
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
        joint_vel = np.zeros((1, self.num_dofs), dtype=np.float32)
        joint_vel[0, :self.num_rotors] = rotor_omega

        # Tail propeller: spin proportional to horizontal speed
        if self.has_tail_prop:
            xy_speed = np.sqrt(self.actual_vxy[0]**2 + self.actual_vxy[1]**2)
            max_xy_speed = 2.0  # PID velocity limit
            tail_omega = self.tail_max_spin * min(xy_speed / max_xy_speed, 1.0)
            self.tail_angle += tail_omega * dt
            self.tail_angle = (self.tail_angle + np.pi) % (2*np.pi) - np.pi
            joint_pos[0, 4] = self.tail_angle
            joint_vel[0, 4] = tail_omega

        self.articulation.set_joint_positions(joint_pos)
        self.articulation.set_joint_velocities(joint_vel)

        self._update_fov_points()

    def publish_pose(self, ros_node):
        """Publish drone pose using logical orientation (without yaw_offset).

        The yaw_offset only rotates the visual mesh so the nose faces forward.
        The PID controller needs the logical yaw (without offset) so that
        body-frame velocity commands aren't reversed.
        """
        qw, qx, qy, qz = euler_to_quat(self.roll, self.pitch, self.yaw)

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = ros_node.get_clock().now().to_msg()

        msg.pose.position.x = float(self.position[0])
        msg.pose.position.y = float(self.position[1])
        msg.pose.position.z = float(self.position[2])
        msg.pose.orientation.w = float(qw)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)

        self.pose_pub.publish(msg)
