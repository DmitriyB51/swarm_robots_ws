# # Copyright (c) 2020-2024, NVIDIA CORPORATION.
# from isaacsim import SimulationApp
# simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

# import omni
# import numpy as np
# from scipy.spatial.transform import Rotation as R
# import rclpy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# import tf2_ros

# from pxr import Gf, UsdGeom

# from isaacsim.core.api import World
# from isaacsim.core.prims import Articulation
# from isaacsim.core.utils.stage import add_reference_to_stage
# from isaacsim.core.utils.extensions import enable_extension
# from isaacsim.storage.native import get_assets_root_path

# enable_extension("isaacsim.ros2.bridge")
# simulation_app.update()


# class UGVInstance:

#     def __init__(self, world, ros_node, name, initial_pose):

#         self.name = name
#         self.namespace = f"/{name}"
#         self.world = world
#         self.ros_node = ros_node

#         self.wheel_radius = 0.0325
#         self.wheel_base = 0.1125

#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0


#         # Spawn robot with correct transform before physics
#         assets_root_path = get_assets_root_path()
#         jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

#         self.prim_path = f"/World/{name}"
#         add_reference_to_stage(jetbot_asset_path, self.prim_path)

#         # Apply transform directly in USD stage
#         stage = omni.usd.get_context().get_stage()
#         prim = stage.GetPrimAtPath(self.prim_path)
#         xform = UsdGeom.Xformable(prim)

#         transform = Gf.Matrix4d().SetTranslate(
#             Gf.Vec3d(
#                 float(initial_pose[0]),
#                 float(initial_pose[1]),
#                 float(initial_pose[2])
#             )
#         )

#         xform.AddTransformOp().Set(transform)

#         # Add articulation AFTER transform is applied
#         self.robot = world.scene.add(
#             Articulation(self.prim_path, name=name)
#         )


#         # ROS Interfaces

#         self.odom_pub = ros_node.create_publisher(
#             Odometry,
#             f"{self.namespace}/odom",
#             10
#         )

#         ros_node.create_subscription(
#             Twist,
#             f"{self.namespace}/cmd_vel",
#             self.cmd_callback,
#             10
#         )

#         self.tf_broadcaster = tf2_ros.TransformBroadcaster(ros_node)

#     # ROS cmd_vel callback
#     def cmd_callback(self, msg):
#         self.linear_velocity = msg.linear.x
#         self.angular_velocity = msg.angular.z

#     # Differential Drive
#     def compute_wheel_velocities(self):
#         v = self.linear_velocity
#         w = self.angular_velocity

#         v_left = (v - (self.wheel_base / 2.0) * w) / self.wheel_radius
#         v_right = (v + (self.wheel_base / 2.0) * w) / self.wheel_radius
#         return v_left, v_right

#     # Apply wheel velocities
#     def update(self):

#         v_l, v_r = self.compute_wheel_velocities()

#         self.robot.set_joint_velocities(
#             np.array([[v_l, v_r]], dtype=np.float32)
#         )

#     # Publish Odometry + TF
#     def publish_odometry(self):

#         positions, orientations = self.robot.get_world_poses()
#         pos, quat_isaac = positions[0], orientations[0]

#         lin_vel_world = self.robot.get_linear_velocities()[0]
#         ang_vel_world = self.robot.get_angular_velocities()[0]

#         # Isaac (w,x,y,z) → ROS (x,y,z,w)
#         quat_ros = [
#             quat_isaac[1],
#             quat_isaac[2],
#             quat_isaac[3],
#             quat_isaac[0],
#         ]

#         rot = R.from_quat(quat_ros)
#         lin_vel_local = rot.inv().apply(lin_vel_world)
#         ang_vel_local = rot.inv().apply(ang_vel_world)

#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.ros_node.get_clock().now().to_msg()
#         odom_msg.header.frame_id = f"{self.name}/odom"
#         odom_msg.child_frame_id = f"{self.name}/base_link"

#         odom_msg.pose.pose.position.x = float(pos[0])
#         odom_msg.pose.pose.position.y = float(pos[1])
#         odom_msg.pose.pose.position.z = float(pos[2])

#         odom_msg.pose.pose.orientation.x = float(quat_ros[0])
#         odom_msg.pose.pose.orientation.y = float(quat_ros[1])
#         odom_msg.pose.pose.orientation.z = float(quat_ros[2])
#         odom_msg.pose.pose.orientation.w = float(quat_ros[3])

#         odom_msg.twist.twist.linear.x = float(lin_vel_local[0])
#         odom_msg.twist.twist.angular.z = float(ang_vel_local[2])

#         self.odom_pub.publish(odom_msg)

#         # TF
#         t = TransformStamped()
#         t.header = odom_msg.header
#         t.child_frame_id = odom_msg.child_frame_id
#         t.transform.translation.x = odom_msg.pose.pose.position.x
#         t.transform.translation.y = odom_msg.pose.pose.position.y
#         t.transform.translation.z = odom_msg.pose.pose.position.z
#         t.transform.rotation = odom_msg.pose.pose.orientation

#         self.tf_broadcaster.sendTransform(t)



# class UGVSwarmManager:

#     def __init__(self):

#         self.world = World(stage_units_in_meters=1.0)
#         self.world.scene.add_default_ground_plane()

#         rclpy.init()
#         self.ros_node = rclpy.create_node("ugv_swarm_controller")

#         # Define spawn positions
#         robot_configs = [
#             ("ugv_1", [0.0, 0.0, 0.05]),
#             ("ugv_2", [9.8, 9.7, 0.05]),
#             ("ugv_3", [-8.6, 9.7, 0.05]),
#         ]

#         self.robots = [
#             UGVInstance(self.world, self.ros_node, name, pose)
#             for name, pose in robot_configs
#         ]

#         self.world.reset()

#         self.timeline = omni.timeline.get_timeline_interface()

#     def run(self):

#         self.timeline.play()

#         while simulation_app.is_running():

#             self.world.step(render=True)

#             if self.world.is_playing():
#                 for robot in self.robots:
#                     robot.update()

#             for robot in self.robots:
#                 robot.publish_odometry()

#             rclpy.spin_once(self.ros_node, timeout_sec=0.0)

#         self.shutdown()

#     def shutdown(self):
#         self.ros_node.destroy_node()
#         rclpy.shutdown()
#         simulation_app.close()


# if __name__ == "__main__":
#     manager = UGVSwarmManager()
#     manager.run()













# Copyright (c) 2020-2024, NVIDIA CORPORATION.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

from pxr import Gf, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


class UGVInstance:

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

        v_l, v_r = self.compute_wheel_velocities()

        self.robot.set_joint_velocities(
            np.array([[v_l, v_r]], dtype=np.float32)
        )

    def publish_odometry(self):

        positions, orientations = self.robot.get_world_poses()
        pos, quat_isaac = positions[0], orientations[0]

        lin_vel_world = self.robot.get_linear_velocities()[0]
        ang_vel_world = self.robot.get_angular_velocities()[0]

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

        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


class UGVSwarmManager:

    def __init__(self):

        self.world = World(stage_units_in_meters=1.0)

        env_usd_path = "/home/qasob/swarm_robots_ws/src/robots/scene/maze_small_with_ground.usd"
        add_reference_to_stage(env_usd_path, "/World/Environment")

        rclpy.init()
        self.ros_node = rclpy.create_node("ugv_swarm_controller")

        # Robot configs: (name, [x, y, z], [qx, qy, qz, qw])
        robot_configs = [
            ("ugv_1", [-1.0795470476150513, -18.356243133544922, 0.05], [0.0, 0.0, 0.7239233431599225, 0.6898804195135279]),
            ("ugv_2", [15.534133911132812, -0.7533082962036133, 0.05], [0.0, 0.0, -0.9999898369146042, 0.004508444022421742]),
            ("ugv_3", [-17.56472396850586, -0.7498464584350586, 0.05], [0.0, 0.0, 0.021339575169731635, 0.9997722853388042]),
        ]

        self.robots = [
            UGVInstance(self.world, self.ros_node, name, position, orientation)
            for name, position, orientation in robot_configs
        ]

        self.world.reset()

        self.timeline = omni.timeline.get_timeline_interface()

    def run(self):

        self.timeline.play()

        while simulation_app.is_running():

            self.world.step(render=True)

            if self.world.is_playing():
                for robot in self.robots:
                    robot.update()

            for robot in self.robots:
                robot.publish_odometry()

            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

        self.shutdown()

    def shutdown(self):
        self.ros_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    manager = UGVSwarmManager()
    manager.run()
