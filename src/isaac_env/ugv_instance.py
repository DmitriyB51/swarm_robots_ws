# UGV Instance class - import after SimulationApp is created
import numpy as np
from scipy.spatial.transform import Rotation as R

import omni
from pxr import Gf, UsdGeom

from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class UGVInstance:
    """UGV (ground vehicle) instance with differential drive."""

    def __init__(self, world, ros_node, name, initial_position, initial_orientation):

        self.name = name
        self.namespace = f"/{name}"
        self.world = world
        self.ros_node = ros_node

        self.wheel_radius = 0.0325
        self.wheel_base = 0.1125

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

        self.prim_path = f"/World/{name}"
        add_reference_to_stage(jetbot_asset_path, self.prim_path)

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self.prim_path)
        xform = UsdGeom.Xformable(prim)

        # Create rotation from quaternion (Gf.Quatd takes real part first: w, x, y, z)
        quat = Gf.Quatd(
            float(initial_orientation[3]),  # w (real part)
            float(initial_orientation[0]),  # x
            float(initial_orientation[1]),  # y
            float(initial_orientation[2])   # z
        )
        rotation = Gf.Rotation(quat)

        # Create the transform matrix with rotation and translation
        transform = Gf.Matrix4d()
        transform.SetTransform(rotation, Gf.Vec3d(
            float(initial_position[0]),
            float(initial_position[1]),
            float(initial_position[2])
        ))

        xform.AddTransformOp().Set(transform)

        scale = 5.0  # increase to make UGVs bigger
        xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))

        self.robot = world.scene.add(
            Articulation(self.prim_path, name=name)
        )

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

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)

    def cmd_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def compute_wheel_velocities(self):
        v = self.linear_velocity
        w = self.angular_velocity

        v_left = (v - (self.wheel_base / 2.0) * w) / self.wheel_radius
        v_right = (v + (self.wheel_base / 2.0) * w) / self.wheel_radius
        return v_left, v_right

    def update(self):
        """Apply wheel velocities for differential drive."""
        v_l, v_r = self.compute_wheel_velocities()

        self.robot.set_joint_velocities(
            np.array([[v_l, v_r]], dtype=np.float32)
        )

    def publish_odometry(self):
        """Publish odometry and TF."""
        positions, orientations = self.robot.get_world_poses()
        pos, quat_isaac = positions[0], orientations[0]

        lin_vel_world = self.robot.get_linear_velocities()[0]
        ang_vel_world = self.robot.get_angular_velocities()[0]

        # Isaac (w,x,y,z) → ROS (x,y,z,w)
        quat_ros = [
            quat_isaac[1],
            quat_isaac[2],
            quat_isaac[3],
            quat_isaac[0],
        ]

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
