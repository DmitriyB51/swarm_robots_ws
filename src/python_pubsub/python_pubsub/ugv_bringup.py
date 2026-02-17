# # Copyright (c) 2020-2024, NVIDIA CORPORATION.

# from isaacsim import SimulationApp

# # Start Isaac Sim
# simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

# import sys
# import omni
# import carb
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion

# from isaacsim.core.api import World
# from isaacsim.core.prims import Articulation
# from isaacsim.core.utils.stage import add_reference_to_stage
# from isaacsim.core.utils.extensions import enable_extension
# from isaacsim.storage.native import get_assets_root_path

# # Enable ROS2 bridge
# enable_extension("isaacsim.ros2.bridge")
# simulation_app.update()


# class JetbotCmdVel(Node):
#     def __init__(self):
#         super().__init__("jetbot_cmdvel_node")

#         self.timeline = omni.timeline.get_timeline_interface()

#         # Create world
#         self.world = World(stage_units_in_meters=1.0)
#         self.world.scene.add_default_ground_plane()

#         # Load JetBot
#         assets_root_path = get_assets_root_path()
#         if assets_root_path is None:
#             carb.log_error("Could not find Isaac Sim assets folder")
#             simulation_app.close()
#             sys.exit()

#         jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
#         add_reference_to_stage(jetbot_asset_path, "/World/Jetbot")

#         self.jetbot = Articulation("/World/Jetbot", name="jetbot")

#         self.world.reset()

#         # Robot parameters
#         self.wheel_radius = 0.0325
#         self.wheel_base = 0.1125

#         self.linear_velocity = 0.0
#         self.angular_velocity = 0.0

#         # Subscriber
#         self.subscription = self.create_subscription(
#             Twist,
#             "/ugv_1/cmd_vel",
#             self.cmd_vel_callback,
#             10
#         )

#         # Odom publisher
#         self.odom_pub = self.create_publisher(
#             Odometry,
#             "/ugv_1/odom",
#             10
#         )

#     def cmd_vel_callback(self, msg: Twist):
#         self.linear_velocity = msg.linear.x
#         self.angular_velocity = msg.angular.z

#     def compute_wheel_velocities(self):
#         v = self.linear_velocity
#         w = self.angular_velocity

#         v_left = (v - (self.wheel_base / 2.0) * w) / self.wheel_radius
#         v_right = (v + (self.wheel_base / 2.0) * w) / self.wheel_radius

#         return v_left, v_right

#     def publish_odometry(self):
#         # Get articulation pose (batched)
#         positions, orientations = self.jetbot.get_world_poses()

#         position = positions[0]
#         orientation = orientations[0]

#         lin_vels = self.jetbot.get_linear_velocities()
#         ang_vels = self.jetbot.get_angular_velocities()

#         lin_vel = lin_vels[0]
#         ang_vel = ang_vels[0]

#         odom_msg = Odometry()
#         odom_msg.header.stamp = self.get_clock().now().to_msg()
#         odom_msg.header.frame_id = "odom"
#         odom_msg.child_frame_id = "base_link"

#         # Pose
#         odom_msg.pose.pose.position.x = float(position[0])
#         odom_msg.pose.pose.position.y = float(position[1])
#         odom_msg.pose.pose.position.z = float(position[2])

#         odom_msg.pose.pose.orientation.x = float(orientation[0])
#         odom_msg.pose.pose.orientation.y = float(orientation[1])
#         odom_msg.pose.pose.orientation.z = float(orientation[2])
#         odom_msg.pose.pose.orientation.w = float(orientation[3])

#         # Twist
#         odom_msg.twist.twist.linear.x = float(lin_vel[0])
#         odom_msg.twist.twist.linear.y = float(lin_vel[1])
#         odom_msg.twist.twist.linear.z = float(lin_vel[2])

#         odom_msg.twist.twist.angular.x = float(ang_vel[0])
#         odom_msg.twist.twist.angular.y = float(ang_vel[1])
#         odom_msg.twist.twist.angular.z = float(ang_vel[2])

#         self.odom_pub.publish(odom_msg)

#     def run(self):
#         self.timeline.play()
#         reset_needed = False

#         while simulation_app.is_running():

#             self.world.step(render=True)
#             rclpy.spin_once(self, timeout_sec=0.0)

#             if self.world.is_stopped() and not reset_needed:
#                 reset_needed = True

#             if self.world.is_playing():

#                 if reset_needed:
#                     self.world.reset()
#                     reset_needed = False

#                 # Apply wheel velocities
#                 v_left, v_right = self.compute_wheel_velocities()
#                 self.jetbot.set_joint_velocities([[v_left, v_right]])

#                 # Publish odom
#                 self.publish_odometry()

#         self.timeline.stop()
#         self.destroy_node()
#         simulation_app.close()


# if __name__ == "__main__":
#     rclpy.init()
#     node = JetbotCmdVel()
#     node.run()




# Copyright (c) 2020-2024, NVIDIA CORPORATION.
from isaacsim import SimulationApp

# Start Isaac Sim
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import sys
import omni
import carb
import numpy as np
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path

# Enable ROS2 bridge
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

class JetbotCmdVel(Node):
    def __init__(self):
        super().__init__("jetbot_cmdvel_node")

        self.timeline = omni.timeline.get_timeline_interface()

        # Create world and add Jetbot
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        assets_root_path = get_assets_root_path()
        jetbot_asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
        add_reference_to_stage(jetbot_asset_path, "/World/Jetbot")

        # Initialize Articulation and add it to the scene (Critical for physics updates)
        self.jetbot = self.world.scene.add(Articulation("/World/Jetbot", name="jetbot"))

        self.world.reset()

        # Robot parameters
        self.wheel_radius = 0.0325
        self.wheel_base = 0.1125
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # ROS2 Communication
        self.subscription = self.create_subscription(Twist, "/ugv_1/cmd_vel", self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/ugv_1/odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) # For odom -> base_link transform

    def cmd_vel_callback(self, msg: Twist):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def compute_wheel_velocities(self):
        v, w = self.linear_velocity, self.angular_velocity
        v_left = (v - (self.wheel_base / 2.0) * w) / self.wheel_radius
        v_right = (v + (self.wheel_base / 2.0) * w) / self.wheel_radius
        return v_left, v_right

    def publish_odometry(self):
        # 1. Get state from Isaac Sim
        positions, orientations = self.jetbot.get_world_poses()
        pos, quat_isaac = positions[0], orientations[0] # Isaac: [W, X, Y, Z]

        lin_vel_world = self.jetbot.get_linear_velocities()[0]
        ang_vel_world = self.jetbot.get_angular_velocities()[0]

        # 2. Convert Isaac Quat [W, X, Y, Z] to SciPy [X, Y, Z, W]
        # SciPy and ROS use the same [X, Y, Z, W] format
        quat_scipy = [quat_isaac[1], quat_isaac[2], quat_isaac[3], quat_isaac[0]]
        rot = R.from_quat(quat_scipy)
        
        # 3. Transform World Velocity to Local Robot Frame
        # We need the inverse rotation to go from World -> Local
        lin_vel_local = rot.inv().apply(lin_vel_world)
        ang_vel_local = rot.inv().apply(ang_vel_world)

        # 4. Create Odometry Message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Pose
        odom_msg.pose.pose.position.x = float(pos[0])
        odom_msg.pose.pose.position.y = float(pos[1])
        odom_msg.pose.pose.position.z = float(pos[2])
        
        odom_msg.pose.pose.orientation.x = float(quat_scipy[0])
        odom_msg.pose.pose.orientation.y = float(quat_scipy[1])
        odom_msg.pose.pose.orientation.z = float(quat_scipy[2])
        odom_msg.pose.pose.orientation.w = float(quat_scipy[3])

        # Twist (In Local Frame)
        odom_msg.twist.twist.linear.x = float(lin_vel_local[0])
        odom_msg.twist.twist.linear.y = float(lin_vel_local[1])
        odom_msg.twist.twist.angular.z = float(ang_vel_local[2])

        self.odom_pub.publish(odom_msg)

        # 4. Broadcast Transform (Optional but recommended for RViz)
        t = TransformStamped()
        t.header = odom_msg.header
        t.child_frame_id = odom_msg.child_frame_id
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def run(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
            
            if self.world.is_playing():
                # Apply commands
                v_l, v_r = self.compute_wheel_velocities()
                self.jetbot.set_joint_velocities(np.array([[v_l, v_r]]))
                
                # Publish
                self.publish_odometry()
            
            # Allow ROS2 to process callbacks
            rclpy.spin_once(self, timeout_sec=0.0)

        self.timeline.stop()
        simulation_app.close()

if __name__ == "__main__":
    rclpy.init()
    node = JetbotCmdVel()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()