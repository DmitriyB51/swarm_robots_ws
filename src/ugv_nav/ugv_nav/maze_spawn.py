from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "headless": False
})

import omni
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.api import World

import os
usd_path = os.path.expanduser("~/swarm_robots_ws/src/robots/scene/maze_small_with_ground.usd")

open_stage(usd_path)

world = World()
world.reset()

while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()



