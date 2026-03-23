# Copyright (c) 2020-2024, NVIDIA CORPORATION.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import os
import time
import carb
import omni

import rclpy

from pxr import UsdGeom, UsdLux, UsdShade, Sdf, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import get_assets_root_path

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
enable_extension("omni.physx.supportui")
enable_extension("omni.physx.ui")
simulation_app.update()

# Import instance classes after SimulationApp is created
from drone_instance import DroneInstance
from ugv_instance import UGVInstance


class CombinedSwarmManager:
    """Manager for combined VTOL and UGV swarm simulation."""

    def __init__(self):

        # Check assets
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder")
            simulation_app.close()
            return

        # Create world
        self.world = World(stage_units_in_meters=1.0)
        self.stage = self.world.stage

        # Load environment
        env_usd_path = os.path.expanduser("~/swarm_robots_ws/src/robots/scene/maze_small_with_ground.usd")
        if os.path.exists(env_usd_path):
            add_reference_to_stage(env_usd_path, "/World/Environment")
            # Apply 180 degree rotation around Z-axis to match ugv_spawn orientation
            env_prim = self.stage.GetPrimAtPath("/World/Environment")
            env_xform = UsdGeom.Xformable(env_prim)
            env_xform.AddRotateZOp().Set(0.0)
        else:
            self.world.scene.add_default_ground_plane()
            carb.log_warn(f"Environment USD not found at {env_usd_path}, using default ground plane")

        # Camera setup
        set_camera_view(
            eye=[-10, 0.0, 15],
            target=[0.0, 0.0, 0.0],
            camera_prim_path="/OmniverseKit_Persp"
        )

        # Add dome light
        dome_light = UsdLux.DomeLight.Define(self.stage, "/World/DomeLight")
        dome_light.CreateIntensityAttr(500)

        # Red dot marker at the target/goal point on the floor
        target_x, target_y = -12.0, 12.0
        sphere = UsdGeom.Sphere.Define(self.stage, "/World/TargetMarker")
        sphere.GetRadiusAttr().Set(0.3)
        sphere.AddTranslateOp().Set(Gf.Vec3d(target_x, target_y, 0.15))
        # Red emissive material so it's visible
        mat = UsdShade.Material.Define(self.stage, "/World/TargetMarker/RedMaterial")
        shader = UsdShade.Shader.Define(self.stage, "/World/TargetMarker/RedMaterial/Shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.0, 0.0))
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.8, 0.0, 0.0))
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        UsdShade.MaterialBindingAPI.Apply(sphere.GetPrim()).Bind(mat)

        # Initialize ROS
        rclpy.init()
        self.ros_node = rclpy.create_node("combined_swarm_controller")

        # VTOL drone asset path (new drone with drop mechanism)
        vtol_asset_path = os.path.expanduser(
            "~/swarm_robots_ws/src/robots/drone/drone_final.usd"
        )

        # Drone configs: (name, [x, y, z]) — on landing pads
        drone_configs = [
            ("vtol_1", [-30.0, 30.0, 0.25]),
            ("vtol_2", [-34.0, 37.0, 0.25]),
            ("vtol_3", [-34.0, 23.0, 0.25]),
        ]

        # Carry offset: UGV position relative to drone (from reference USD)
        self.carry_offset = [0.6462, 0.0400, 0.0441]

        # UGV configs: spawn at drone positions + carry offset (carried from start)
        # Drones start at yaw=0 but have yaw_offset=pi (visual rotation in code)
        # Rotate carry offset by yaw_offset so UGVs spawn at correct position
        import math
        yaw_offset = math.pi  # Must match drone_instance yaw_offset
        cos_y = math.cos(yaw_offset)
        sin_y = math.sin(yaw_offset)
        rotated_offset = [
            self.carry_offset[0] * cos_y - self.carry_offset[1] * sin_y,
            self.carry_offset[0] * sin_y + self.carry_offset[1] * cos_y,
            self.carry_offset[2],
        ]
        ugv_configs = [
            (
                f"ugv_{i+1}",
                [
                    drone_configs[i][1][0] + rotated_offset[0],
                    drone_configs[i][1][1] + rotated_offset[1],
                    drone_configs[i][1][2] + rotated_offset[2],
                ],
                [0.0, 0.0, 0.0, 1.0],
            )
            for i in range(len(drone_configs))
        ]

        # Create drones
        self.drones = []
        if os.path.exists(vtol_asset_path):
            for name, position in drone_configs:
                self.drones.append(
                    DroneInstance(
                        self.world,
                        self.ros_node,
                        name,
                        position,
                        vtol_asset_path,
                        self.stage
                    )
                )
        else:
            carb.log_warn(f"VTOL asset not found at {vtol_asset_path}, skipping drones")

        # UGV asset path
        ugv_asset_path = os.path.expanduser(
            "~/swarm_robots_ws/src/robots/rover/rover.usd"
        )

        # Create UGVs
        self.ugvs = []
        self.ugv_init_configs = []
        if os.path.exists(ugv_asset_path):
            for name, position, orientation in ugv_configs:
                self.ugvs.append(
                    UGVInstance(
                        self.world,
                        self.ros_node,
                        name,
                        position,
                        orientation,
                        ugv_asset_path,
                        self.stage
                    )
                )
                self.ugv_init_configs.append((position, orientation))
        else:
            carb.log_warn(f"UGV asset not found at {ugv_asset_path}, skipping UGVs")

        # Link drone-UGV pairs
        for drone, ugv in zip(self.drones, self.ugvs):
            drone.carried_ugv = ugv

        # Reset world
        self.world.reset()

        from scene.materials import apply_all_materials
        from scene.environment import populate_environment
        from scene.landing_pads import create_landing_pads
        apply_all_materials(self.stage)
        populate_environment(self.stage)
        create_landing_pads(self.stage)

        # Initialize drones after reset
        for drone in self.drones:
            drone.initialize_after_reset()

        # Initialize UGVs after reset
        for ugv, (pos, ori) in zip(self.ugvs, self.ugv_init_configs):
            ugv.initialize_after_reset(pos, ori)

        # Create visual carry attachments AFTER reset so scene graph is stable
        for drone, ugv in zip(self.drones, self.ugvs):
            ugv.setup_carry_visual(drone)

        self.timeline = omni.timeline.get_timeline_interface()
        self.stage_units = get_stage_units()
        self.last_time = time.time()

    def run(self):
        """Main simulation loop."""
        self.timeline.play()

        while simulation_app.is_running():

            now = time.time()
            dt = now - self.last_time
            self.last_time = now

            # Update drones
            for drone in self.drones:
                drone.update(dt, self.stage_units)

            # Step world (physics + render)
            self.world.step(render=True)

            # Update ground UGVs after physics (wheel drive)
            if self.world.is_playing():
                for ugv in self.ugvs:
                    if not ugv.attached:
                        ugv.update()

            # Publish poses/odometry
            for drone in self.drones:
                drone.publish_pose(self.ros_node)

            for ugv in self.ugvs:
                ugv.publish_odometry()

            # Process ROS callbacks
            rclpy.spin_once(self.ros_node, timeout_sec=0.0)

        self.shutdown()

    def shutdown(self):
        """Clean shutdown."""
        self.ros_node.destroy_node()
        rclpy.shutdown()
        simulation_app.close()


def main():
    manager = CombinedSwarmManager()
    manager.run()


if __name__ == "__main__":
    main()