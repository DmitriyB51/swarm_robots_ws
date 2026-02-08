from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})  # start the simulation app, with GUI open

import sys

import carb
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# preparing the scene
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()



my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()  # add ground plane
set_camera_view(
    eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp"
)  # set camera view






asset_path = assets_root_path + "/Isaac/Robots/Bitcraze/Crazyflie/cf2x.usd"



add_reference_to_stage(usd_path=asset_path, prim_path="/World/drone")
drone = Articulation(prim_paths_expr="/World/drone", name="my_drone")


drone.set_world_poses(positions=np.array([[0.0, -1.0, 0.0]]) / get_stage_units())


# initialize the world
my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)

simulation_app.close()
