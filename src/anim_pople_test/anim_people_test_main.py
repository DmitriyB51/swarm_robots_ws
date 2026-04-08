import os as _os

#hardcored imports from my pc 
_SCHEMA_PLUGIN_PATHS = [
    "/home/dmitriyb51/isaacsim/extscache/omni.anim.navigation.schema-106.4.0+106.4.0.lx64.r.cp310/plugins/NavSchema/resources",
    "/home/dmitriyb51/isaacsim/extscache/omni.anim.graph.schema-106.5.0+106.5.0.lx64.r.cp310/plugins/AnimGraphSchema/resources",
    "/home/dmitriyb51/isaacsim/extscache/omni.anim.behavior.schema-106.5.0+106.5.0.lx64.r.cp310/plugins/BehaviorSchema/resources",
    "/home/dmitriyb51/isaacsim/extscache/omni.usd.schema.anim-0.0.0+d02c707b.lx64.r.cp310/plugins/OmniSkelSchema/resources",
    "/home/dmitriyb51/isaacsim/extscache/omni.usd.schema.anim-0.0.0+d02c707b.lx64.r.cp310/plugins/AnimationSchema/resources",
    "/home/dmitriyb51/isaacsim/extscache/omni.usd.schema.anim-0.0.0+d02c707b.lx64.r.cp310/plugins/RetargetingSchema/resources",
]

_existing = _os.environ.get("PXR_PLUGINPATH_NAME", "")
_os.environ["PXR_PLUGINPATH_NAME"] = ":".join(
    [p for p in _SCHEMA_PLUGIN_PATHS + [_existing] if p]
)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.anim.navigation.schema-106.4.0")
enable_extension("omni.anim.navigation.core-106.4.0")
enable_extension("omni.anim.navigation.recast-106.4.0")
enable_extension("omni.anim.people")           # 0.6.7
enable_extension("omni.anim.graph.bundle")     # 106.5.0 
enable_extension("omni.anim.retarget.bundle")  # 106.5.0
simulation_app.update()


from pxr import Plug as _Plug
for _p in _Plug.Registry().GetAllPlugins():
    if _p.name.endswith("Schema") and not _p.isLoaded:
        try:
            _p.Load()
        except Exception:
            pass


from pxr import Usd as _Usd
_reg = _Usd.SchemaRegistry()
for _alias in ("AnimationGraphAPI", "NavMeshVolume", "NavMeshAreaAPI"):
    _reg.IsAppliedAPISchema(_alias)
    _reg.IsConcrete(_alias)




# Late imports
import time

import carb
import omni.kit.commands
import omni.timeline
import omni.usd
import omni.anim.navigation.core as nav
import omni.anim.graph.core as ag

from pxr import Sdf, Usd, UsdGeom, UsdLux, Gf


def _warmup_schemas():
    import AnimGraphSchema as _AGS
    import NavSchema as _NS
    _ctx = omni.usd.get_context()
    _ctx.new_stage()
    _stage = _ctx.get_stage()
    _p = _stage.DefinePrim("/SchemaWarmup", "Xform")
    _p.HasAPI(_AGS.AnimationGraphAPI)
    _NS.NavMeshVolume.Define(_stage, Sdf.Path("/SchemaWarmupNav"))
_warmup_schemas()

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# Same NavigationManager + helpers that the working goto_goal.py uses.
from omni.anim.people.scripts.navigation_manager import NavigationManager
from omni.anim.people.scripts.utils import Utils
from isaacsim.replicator.metropolis.utils.carb_util import CarbUtil








# Configuration
GOAL_POSITION = (6.29, 0.0, -1.6)  
BIPED_SETUP_PRIM_PATH = "/World/Characters/Biped_Setup"
CHARACTER_PRIM_PATH = "/World/Characters/F_Business_02"
ANIM_GRAPH_PRIM_PATH = "/World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph"





# function to wait 
def pump_app(n=20):
    """Run n app update ticks so async USD/Hydra work can drain."""
    for _ in range(n):
        simulation_app.update()


# finding SkelRoot of object
def find_skel_root(stage, root_prim_path):
    
    root = stage.GetPrimAtPath(root_prim_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "SkelRoot":
            return prim.GetPath()
    return None


# function to define volume of NavMesh
def define_navmesh_volume(stage, prim_path, center, half_size):

    import NavSchema  # registered by omni.anim.navigation.schema

    volume = NavSchema.NavMeshVolume.Define(stage, Sdf.Path(prim_path))
    prim = volume.GetPrim()

    boundable = UsdGeom.Boundable(prim)
    boundable.GetExtentAttr().Set(
        [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]
    )

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(*center))
    xformable.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    xformable.AddScaleOp().Set(Gf.Vec3f(2 * half_size[0], 2 * half_size[1], 2 * half_size[2]))

    # Apply NavMeshAreaAPI so the bake includes this volume as walkable.
    omni.kit.commands.execute(
        "ApplyNavMeshAPICommand",
        prim_path=Sdf.Path(prim_path),
        api=NavSchema.NavMeshAreaAPI,
    )
    return prim

# BAKING
def bake_navmesh_blocking():

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
        carb.log_error("[anim_people_test] navmesh bake did not complete in time")
    sub = None  


def main():
    # World + minimal scene
    world = World(stage_units_in_meters=1.0)
    stage = world.stage

    world.scene.add_default_ground_plane()

    dome_light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome_light.CreateIntensityAttr(500)

    set_camera_view(
        eye=[10.0, 10.0, 6.0],
        target=[0.0, 0.0, 1.0],
        camera_prim_path="/OmniverseKit_Persp",
    )

    
    if not stage.GetPrimAtPath("/World/Characters").IsValid():
        UsdGeom.Xform.Define(stage, "/World/Characters")

    # Reference Biped_Setup (provides skeleton + AnimationGraph) and the character mesh.
    asset_root = get_assets_root_path()
    if asset_root is None:
        carb.log_error("[anim_people_test] could not find Isaac Sim assets root")
        simulation_app.close()
        return

    biped_usd = f"{asset_root}/Isaac/People/Characters/Biped_Setup.usd"
    character_usd = f"{asset_root}/Isaac/People/Characters/F_Business_02/F_Business_02.usd"

    add_reference_to_stage(biped_usd, BIPED_SETUP_PRIM_PATH)
    add_reference_to_stage(character_usd, CHARACTER_PRIM_PATH)


    #wait 30 frames
    pump_app(30)

    # find bipe and make it invisible
    biped_prim = stage.GetPrimAtPath(BIPED_SETUP_PRIM_PATH)
    if biped_prim.IsValid():
        UsdGeom.Imageable(biped_prim).MakeInvisible()

    # Find the SkelRoot inside the referenced character.
    skel_root_path = find_skel_root(stage, CHARACTER_PRIM_PATH)
    if skel_root_path is None:
        carb.log_error(
            f"[anim_people_test] no SkelRoot under {CHARACTER_PRIM_PATH} — "
            "check that the character USD reference resolved correctly."
        )
        simulation_app.close()
        return

    # Verify the AnimationGraph prim from Biped_Setup actually exists before
    if not stage.GetPrimAtPath(ANIM_GRAPH_PRIM_PATH).IsValid():
        carb.log_error(
            f"[anim_people_test] AnimationGraph prim not found at "
            f"{ANIM_GRAPH_PRIM_PATH} — Biped_Setup reference probably did not "
            "load yet. Try increasing pump_app() count."
        )
        simulation_app.close()
        return

    # Apply the AnimationGraph to the character's SkelRoot.
    omni.kit.commands.execute(
        "RemoveAnimationGraphAPICommand",
        paths=[Sdf.Path(skel_root_path)],
    )
    omni.kit.commands.execute(
        "ApplyAnimationGraphAPICommand",
        paths=[Sdf.Path(skel_root_path)],
        animation_graph_path=Sdf.Path(ANIM_GRAPH_PRIM_PATH),
    )



    # nav mesh size
    define_navmesh_volume(
        stage,
        prim_path="/World/NavMeshVolume",
        center=(0.0, 0.0, 1.0),
        half_size=(25.0, 25.0, 5.0),
    )
    pump_app(5)

    
    bake_navmesh_blocking()

    # Reset world to bind physics + stage, then start the timeline.
    world.reset()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    
    
    character = None
    for _ in range(60):
        simulation_app.update()
        character = ag.get_character(str(skel_root_path))
        if character is not None:
            break
    if character is None:
        carb.log_error(
            "[anim_people_test] ag.get_character returned None — "
            "AnimationGraph not bound, character won't animate."
        )
        simulation_app.close()
        return

    nav_mgr = NavigationManager(
        str(skel_root_path),
        navmesh_enabled=True,
        dynamic_avoidance_enabled=False,
    )

    gx, gy, gz = GOAL_POSITION
    nav_mgr.generate_goto_path([str(gx), str(gy), str(gz), "_"])
    character.set_variable("Action", "Walk")

    
    desired_speed = 0.0
    actual_speed = 0.0
    done = False
    last = time.time()

    while simulation_app.is_running() and not done:
        now = time.time()
        dt = now - last
        last = now

        if nav_mgr.destination_reached():
            desired_speed = 0.0
            if actual_speed < 0.001:
                character.set_variable("Action", "None")
                nav_mgr.set_path_points(None)
                nav_mgr.clean_path_targets()
                done = True
        else:
            desired_speed = 2.0

        if not done:
            character.set_variable("Action", "Walk")
            nav_mgr.update_path()
            character.set_variable("PathPoints", nav_mgr.get_path_points())

            max_change = dt / Utils.CONFIG["WalkBlendTime"]
            delta = CarbUtil.clamp(desired_speed - actual_speed, -max_change, max_change)
            actual_speed = CarbUtil.clamp(actual_speed + delta, 0.0, 1.0)
            character.set_variable("Walk", actual_speed)

        world.step(render=True)

    # Keep the viewport open after arrival so the user can inspect the result.
    while simulation_app.is_running():
        world.step(render=True)

    nav_mgr.destroy()
    simulation_app.close()


if __name__ == "__main__":
    main()
