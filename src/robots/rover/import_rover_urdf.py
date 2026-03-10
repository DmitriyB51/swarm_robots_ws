"""
Import rover URDF to USD using Isaac Sim's URDF importer.
Run with: /home/dmitriyb51/isaacsim/python.sh import_rover_urdf.py
"""
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.importer.urdf")
simulation_app.update()

import omni.kit.commands
from pxr import Usd
from isaacsim.asset.importer.urdf._urdf import UrdfJointTargetType

# Create import config
status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.fix_base = False
import_config.make_default_prim = True
import_config.create_physics_scene = False
import_config.default_drive_type = UrdfJointTargetType.JOINT_DRIVE_VELOCITY
import_config.default_drive_strength = 0.0
import_config.default_position_drive_damping = 10000.0

urdf_path = "/home/dmitriyb51/Downloads/urdf_description/urdf/urdf.xacro"
dest_path = "/home/dmitriyb51/swarm_robots_ws/src/robots/rover/rover.usd"

omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_path,
    import_config=import_config,
    dest_path=dest_path,
)

print("Import complete. Checking output...")

import os
rover_dir = os.path.dirname(dest_path)
for f in os.listdir(rover_dir):
    print(f"  {f}")

# Verify the USD
stage = Usd.Stage.Open(dest_path)
dp = stage.GetDefaultPrim()
print(f"\nUSD: {dest_path}")
print(f"Default prim: {dp.GetPath() if dp else 'None'}")
for prim in stage.Traverse():
    schemas = prim.GetAppliedSchemas()
    if schemas:
        print(f"  {prim.GetPath()} {prim.GetTypeName()} {schemas}")

simulation_app.close()
