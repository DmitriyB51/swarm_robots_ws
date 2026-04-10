# Person Instance class - import after SimulationApp + anim extensions are enabled
import time

import carb
import omni.kit.commands
import omni.anim.navigation.core as nav
import omni.anim.graph.core as ag

from pxr import Sdf, Usd, UsdGeom, Gf

from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.storage.native import get_assets_root_path

from omni.anim.people.scripts.navigation_manager import NavigationManager
from omni.anim.people.scripts.utils import Utils
from isaacsim.replicator.metropolis.utils.carb_util import CarbUtil

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


def _find_skel_root(stage, root_prim_path):
    root = stage.GetPrimAtPath(root_prim_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "SkelRoot":
            return prim.GetPath()
    return None


def _define_navmesh_volume(stage, prim_path, center, half_size):
    import NavSchema

    volume = NavSchema.NavMeshVolume.Define(stage, Sdf.Path(prim_path))
    prim = volume.GetPrim()

    UsdGeom.Boundable(prim).GetExtentAttr().Set(
        [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
    )

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*center))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    xformable.AddScaleOp().Set(
        Gf.Vec3f(2 * half_size[0], 2 * half_size[1], 2 * half_size[2])
    )

    omni.kit.commands.execute(
        "ApplyNavMeshAPICommand",
        prim_path=Sdf.Path(prim_path),
        api=NavSchema.NavMeshAreaAPI,
    )
    return prim


def _bake_navmesh_blocking(simulation_app):
    inav = nav.acquire_interface()
    bake_done = {"v": False}

    def _on_evt(evt):
        if evt.type == int(nav.EVENT_TYPE_NAVMESH_UPDATING):
            if evt.payload.get("progress", 0.0) >= 1.0:
                bake_done["v"] = True
        elif evt.type == int(nav.EVENT_TYPE_NAVMESH_UPDATED):
            bake_done["v"] = True

    sub = inav.get_navmesh_event_stream().create_subscription_to_pop(_on_evt)
    inav.start_navmesh_baking()

    for _ in range(600):
        if bake_done["v"]:
            break
        simulation_app.update()

    if not bake_done["v"]:
        carb.log_error("[person_instance] navmesh bake did not complete in time")
    sub = None


class PersonInstance:
    """Animated walking person driven by omni.anim.people NavigationManager."""

    def __init__(self, world, ros_node, name, initial_position, waypoints, stage,
                 simulation_app):
        self.name = name
        self.stage = stage
        self.world = world
        self.simulation_app = simulation_app
        self.waypoints = waypoints
        self.initial_position = initial_position

        self.walking = False
        self.done = False
        self.character = None
        self.nav_mgr = None
        self.skel_root_path = None

        self.desired_speed = 0.0
        self.actual_speed = 0.0

        asset_root = get_assets_root_path()
        biped_usd = f"{asset_root}/Isaac/People/Characters/Biped_Setup.usd"
        character_usd = f"{asset_root}/Isaac/People/Characters/F_Business_02/F_Business_02.usd"

        # Create container
        chars_path = "/World/Characters"
        if not stage.GetPrimAtPath(chars_path).IsValid():
            UsdGeom.Xform.Define(stage, chars_path)

        self.biped_prim_path = f"{chars_path}/{name}_Biped_Setup"
        self.character_prim_path = f"{chars_path}/{name}_F_Business_02"
        self.anim_graph_prim_path = f"{self.biped_prim_path}/CharacterAnimation/AnimationGraph"

        add_reference_to_stage(biped_usd, self.biped_prim_path)
        add_reference_to_stage(character_usd, self.character_prim_path)

        # Set initial position
        char_prim = stage.GetPrimAtPath(self.character_prim_path)
        char_prim.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(initial_position[0], initial_position[1], initial_position[2])
        )

        # Wait for references to load
        for _ in range(30):
            simulation_app.update()

        # Hide Biped_Setup (skeleton rig only, not the visible character)
        biped_prim = stage.GetPrimAtPath(self.biped_prim_path)
        if biped_prim.IsValid():
            UsdGeom.Imageable(biped_prim).MakeInvisible()

        # Find SkelRoot
        self.skel_root_path = _find_skel_root(stage, self.character_prim_path)
        if self.skel_root_path is None:
            carb.log_error(
                f"[person_instance] no SkelRoot under {self.character_prim_path}"
            )
            return

        # Verify AnimationGraph prim exists
        if not stage.GetPrimAtPath(self.anim_graph_prim_path).IsValid():
            carb.log_error(
                f"[person_instance] AnimationGraph not found at "
                f"{self.anim_graph_prim_path}"
            )
            return

        # Apply AnimationGraph to character's SkelRoot
        omni.kit.commands.execute(
            "RemoveAnimationGraphAPICommand",
            paths=[Sdf.Path(self.skel_root_path)],
        )
        omni.kit.commands.execute(
            "ApplyAnimationGraphAPICommand",
            paths=[Sdf.Path(self.skel_root_path)],
            animation_graph_path=Sdf.Path(self.anim_graph_prim_path),
        )

        # ROS subscriber for walk trigger
        ros_node.create_subscription(
            Bool,
            f"/{name}/start_walk",
            self._start_walk_callback,
            10,
        )

        # ROS subscriber for stop trigger (drone found the person)
        ros_node.create_subscription(
            Bool,
            f"/{name}/stop_walk",
            self._stop_walk_callback,
            10,
        )

        # ROS publisher for person pose (used by mission client for detection)
        self.pose_pub = ros_node.create_publisher(
            PoseStamped, f"/{name}/pose", 10
        )

    def setup_navmesh(self):
        """Define navmesh volume and bake. Call once before world.reset()."""
        _define_navmesh_volume(
            self.stage,
            prim_path="/World/NavMeshVolume",
            center=(0.0, 0.0, 1.0),
            half_size=(25.0, 25.0, 5.0),
        )
        for _ in range(5):
            self.simulation_app.update()

        _bake_navmesh_blocking(self.simulation_app)

    def initialize_after_reset(self):
        """Initialize NavigationManager after world.reset() + timeline.play()."""
        if self.skel_root_path is None:
            return

        # Poll for character binding
        self.character = None
        for _ in range(60):
            self.simulation_app.update()
            self.character = ag.get_character(str(self.skel_root_path))
            if self.character is not None:
                break

        if self.character is None:
            carb.log_error(
                f"[person_instance] ag.get_character returned None for "
                f"{self.skel_root_path}"
            )
            return

        self.nav_mgr = NavigationManager(
            str(self.skel_root_path),
            navmesh_enabled=True,
            dynamic_avoidance_enabled=False,
        )

        # Pre-generate the path so it's ready when start_walking is called
        coords = []
        for wx, wy, wz in self.waypoints:
            coords.extend([str(wx), str(wy), str(wz)])
        coords.append("_")
        self.nav_mgr.generate_goto_path(coords)

    def _start_walk_callback(self, msg):
        if msg.data:
            self.start_walking()

    def _stop_walk_callback(self, msg):
        if msg.data:
            self.stop_walking()

    def start_walking(self):
        if self.character is None or self.nav_mgr is None:
            carb.log_error("[person_instance] cannot start walking — not initialized")
            return
        self.walking = True
        self.character.set_variable("Action", "Walk")
        carb.log_warn(f"[person_instance] {self.name} started walking")

    def stop_walking(self):
        if self.character is None:
            return
        self.walking = False
        self.done = True
        self.desired_speed = 0.0
        self.character.set_variable("Action", "None")
        if self.nav_mgr:
            self.nav_mgr.set_path_points(None)
            self.nav_mgr.clean_path_targets()
        carb.log_warn(f"[person_instance] {self.name} stopped walking (found by drone)")

    def publish_pose(self, ros_node):
        """Publish person's current world position via animation graph transform."""
        if self.character is None:
            return
        import carb
        pos = carb.Float3()
        rot = carb.Float4()
        self.character.get_world_transform(pos, rot)

        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = ros_node.get_clock().now().to_msg()
        msg.pose.position.x = float(pos.x)
        msg.pose.position.y = float(pos.y)
        msg.pose.position.z = float(pos.z)
        self.pose_pub.publish(msg)

    def update(self, dt):
        if not self.walking or self.done:
            return
        if self.character is None or self.nav_mgr is None:
            return

        if self.nav_mgr.destination_reached():
            self.desired_speed = 0.0
            if self.actual_speed < 0.001:
                self.character.set_variable("Action", "None")
                self.nav_mgr.set_path_points(None)
                self.nav_mgr.clean_path_targets()
                self.done = True
                carb.log_warn(f"[person_instance] {self.name} reached destination")
                return
        else:
            self.desired_speed = 2.0

        self.character.set_variable("Action", "Walk")
        self.nav_mgr.update_path()
        self.character.set_variable("PathPoints", self.nav_mgr.get_path_points())

        max_change = dt / Utils.CONFIG["WalkBlendTime"]
        delta = CarbUtil.clamp(
            self.desired_speed - self.actual_speed, -max_change, max_change
        )
        self.actual_speed = CarbUtil.clamp(self.actual_speed + delta, 0.0, 1.0)
        self.character.set_variable("Walk", self.actual_speed)

    def destroy(self):
        if self.nav_mgr is not None:
            self.nav_mgr.destroy()
            self.nav_mgr = None
