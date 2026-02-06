# SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  



import sys
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path
from isaacsim.core.utils.extensions import enable_extension

# ROS2 setup
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()


import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class CarController(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.subscription = self.create_subscription(
            String,
            'isaac_pub_topic',
            self.listener_callback,
            10)
        self.current_value = "0"
        
    def listener_callback(self, msg):
        self.current_value = msg.data
        self.get_logger().info(f'{self.current_value}')



# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    
    simulation_app.close()
    sys.exit()



my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view





# Add Carter
asset_path = assets_root_path + "/Isaac/Robots/NVIDIA/NovaCarter/nova_carter.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Car")
car = Articulation(prim_paths_expr="/World/Car", name="my_car")

# set the initial poses of the arm and the car so they don't collide BEFORE the simulation starts
#arm.set_world_poses(positions=np.array([[0.0, 1.0, 0.0]]) / get_stage_units())
car.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]) / get_stage_units())


# initialize the world
my_world.reset()


rclpy.init()
car_controller = CarController()



reset_needed = False
while simulation_app.is_running():
    
    rclpy.spin_once(car_controller, timeout_sec=0.0)
    
    
    my_world.step(render=True)
    
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
        
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            reset_needed = False
    
    
        
        if car_controller.current_value == "1":
            
            car.set_joint_velocities([[1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]])
        else:
            
            car.set_joint_velocities([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

# Cleanup
car_controller.destroy_node()
rclpy.shutdown()
simulation_app.close()




