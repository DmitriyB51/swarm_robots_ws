# Copyright (c) 2020-2024, NVIDIA CORPORATION.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import os
import time
import carb
import omni

import rclpy

from pxr import UsdGeom, UsdLux

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
        env_usd_path = "/home/qasob/swarm_robots_ws/src/robots/scene/maze_small_with_ground.usd"
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

        # Initialize ROS
        rclpy.init()
        self.ros_node = rclpy.create_node("combined_swarm_controller")

        # VTOL drone asset path
        vtol_asset_path = os.path.expanduser(
            "~/swarm_robots_ws/src/vtol_dron_description/urdf/vtol_simple/vtol_simple.usd"
        )

        # Drone configs: (name, [x, y, z])
        drone_configs = [
            ("vtol_1", [-30.0, 30.0, 9.0]),
            ("vtol_2", [-33.0, 35.0, 7.0]),
            ("vtol_3", [-33.0, 25.0, 5.0]),
        ]

        # UGV configs: (name, [x, y, z], [qx, qy, qz, qw])
        ugv_configs = [
            ("ugv_1", [-1.0795470476150513, -18.356243133544922, 0.05], [0.0, 0.0, 0.7239233431599225, 0.6898804195135279]),
            ("ugv_2", [15.534133911132812, -0.7533082962036133, 0.05], [0.0, 0.0, -0.9999898369146042, 0.004508444022421742]),
            ("ugv_3", [-17.56472396850586, -0.7498464584350586, 0.05], [0.0, 0.0, 0.021339575169731635, 0.9997722853388042]),
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

        # Create UGVs
        self.ugvs = [
            UGVInstance(self.world, self.ros_node, name, position, orientation)
            for name, position, orientation in ugv_configs
        ]

        # Reset world
        self.world.reset()

        # Initialize drones after reset
        for drone in self.drones:
            drone.initialize_after_reset()

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

            # Step world
            self.world.step(render=True)

            # Update UGVs (physics-based)
            if self.world.is_playing():
                for ugv in self.ugvs:
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
