# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.

from isaacsim import SimulationApp

# 1. Start the simulation app
simulation_app = SimulationApp({"headless": False}) 

import sys
import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

# 2. Preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

# 3. Add Jetbot
# The Jetbot is a two-wheeled differential drive robot
asset_path = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/Jetbot")
jetbot = Articulation(prim_paths_expr="/World/Jetbot", name="my_jetbot")

# 4. Initialize the world
my_world.reset()

print("Simulation started. Close the window to stop.")

# 5. The "While Logic" Loop
# This runs until the Simulation App is shut down by the user
while simulation_app.is_running():
    # step the simulation
    my_world.step(render=True)
    
    if my_world.is_playing():
        # Jetbot has 2 wheel joints. 
        # We apply equal velocity to both to move straight forward.
        # Format is [[left_wheel_speed, right_wheel_speed]]
        forward_velocity = np.array([[5.0, 5.0]])
        jetbot.set_joint_velocities(forward_velocity)
        
        # Optional: Print positions to console
        joint_positions = jetbot.get_joint_positions()
        # print(f"Current Wheel Positions: {joint_positions}")

# Cleanup
simulation_app.close()