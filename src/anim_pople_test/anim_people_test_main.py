"""
Standalone Python translation of the working GUI recipe for spawning
F_Business_02 with omni.anim.people on Isaac Sim 4.5.

Run from the workspace root:
    ~/isaacsim/python.sh src/anim_pople_test/anim_people_test_main.py

What it does (mirrors the GUI clicks 1:1):
  1. SimulationApp + enable the same extension stack the GUI recipe uses.
  2. Create a World with a default ground plane and a dome light.
  3. Reference Biped_Setup.usd (skeleton + AnimationGraph) and F_Business_02.usd.
  4. Apply AnimationGraphAPI to the F_Business_02 SkelRoot via the same
     omni.kit.commands the replicator agent core uses.
  5. Create a NavMesh include volume that auto-encloses the stage and bake.
  6. Press Play, then drive NavigationManager directly each frame to walk
     the character to GOAL.
"""

# 1. SimulationApp must be created BEFORE any omni.* import.
from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

# 2. Enable the animation/navigation extension stack with EXPLICIT versions.
#
#    Why explicit "name-version" pinning is mandatory in standalone:
#
#    `omni.anim.people-0.6.7` hard-pins `omni.anim.navigation.core == 106.4.0`
#    and `omni.anim.navigation.recast == 106.4.0`. But navigation.core 106.4.0
#    has only an UNVERSIONED dependency on `omni.anim.navigation.schema`, so
#    the resolver picks schema-106.5.0 (the latest available). The C++ ABI
#    between core-106.4.0 and schema-106.5.0 does not match, so NavSchema
#    types fail to register in USD's TfType system. Symptoms when this is
#    wrong: a flood of `HasAPI: Invalid unknown schema type (TfType::_Unknown)`
#    warnings during navmesh bake, the bake never completing, and
#    `ApplyAnimationGraphAPICommand` silently failing to bind the API.
#
#    Forcing schema-106.4.0 (and the rest of the stack at 106.4.0) first
#    locks the resolver onto a fully consistent set of navigation extensions.
#
#    The GUI's "selected version" dropdowns in Window -> Extensions are
#    persistent state that the GUI honours, but `enable_extension` from a
#    standalone script does NOT read them — it picks the highest available
#    version that satisfies the current dep graph.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.anim.navigation.schema-106.4.0")
enable_extension("omni.anim.navigation.core-106.4.0")
enable_extension("omni.anim.navigation.recast-106.4.0")
enable_extension("omni.anim.people")           # 0.6.7
enable_extension("omni.anim.graph.bundle")     # 106.5.0 — registers AnimGraphSchema
enable_extension("omni.anim.retarget.bundle")  # 106.5.0
simulation_app.update()

# CRITICAL: force-load every USD schema plugin so UsdSchemaRegistry knows
# their concrete prim / applied API kinds.
#
# Why: Kit's `enable_extension` registers the C++ schema libs with
# `Plug.Registry()` but does NOT call `Plug.Plugin.Load()` on them. Their
# .so files are not dlopen'd, so although the TfTypes exist (names are
# known), USD's `UsdSchemaRegistry::IsConcrete()` /
# `IsAppliedAPISchema()` return False because the registry's
# `_concreteTypeToPrimDefinition` /
# `_singleApplyAPISchemaTypeToPrimDefinition` maps are only populated
# from a LOADED plugin's `generatedSchema.usda`.
#
# Symptoms when this is missing:
#   - "AnimGraphSchemaAnimationGraphAPI is not an applied API schema type"
#     from inside `ApplyAnimationGraphAPICommand` (it calls
#     `prim.HasAPI(AnimGraphSchema.AnimationGraphAPI)`).
#   - "Empty typeName for </World/NavMeshVolume.extent>" when defining a
#     NavMeshVolume — the prim type isn't known so its attribute spec
#     gets created with no typeName.
#   - A flood of "HasAPI: Invalid unknown schema type (TfType::_Unknown)"
#     when omni.anim.skelJoint walks the loaded biped looking for its
#     own schemas (omniSkelSchema, behaviorSchema, animationSchema,
#     retargetingSchema).
#
# In the GUI flow this isn't an issue because the schemas are auto-loaded
# during kit boot via the persistent extension list, BEFORE
# UsdSchemaRegistry is first touched. In standalone, we enable them
# AFTER SimulationApp has already constructed the schema registry, and
# UsdSchemaRegistry only learns about new plugin schemas when the plugin
# is actually `Load`ed (it listens for `Plug.Notice.DidLoadPlugins`).
#
# We force-load every plugin whose name ends in "Schema" — same hammer
# the omni.anim.people-related extensions would have applied themselves
# if Kit's lazy loader had triggered them.
from pxr import Plug as _Plug
_LOADED_SCHEMAS = []
for _p in _Plug.Registry().GetAllPlugins():
    if _p.name.endswith("Schema") and not _p.isLoaded:
        try:
            _p.Load()
            _LOADED_SCHEMAS.append(_p.name)
        except Exception as _e:
            print(f"[anim_people_test] failed to load schema plugin {_p.name}: {_e}")
print(f"[anim_people_test] force-loaded schema plugins: {_LOADED_SCHEMAS}")

# 3. Late imports — only legal after SimulationApp + the extensions above.
import time

import carb
import omni.kit.commands
import omni.timeline
import omni.usd
import omni.anim.navigation.core as nav
import omni.anim.graph.core as ag

from pxr import Sdf, Usd, UsdGeom, UsdLux, Gf

from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.storage.native import get_assets_root_path

# Same NavigationManager + helpers that the working goto_goal.py uses.
from omni.anim.people.scripts.navigation_manager import NavigationManager
from omni.anim.people.scripts.utils import Utils
from isaacsim.replicator.metropolis.utils.carb_util import CarbUtil

# ── Schema sanity check ──────────────────────────────────────────────────────
# Verify that after Plug.Load + late imports, USD's schema registry
# correctly classifies the types we depend on. Write the results to a
# log file so we can inspect them even if Kit swallows stdout.
def _schema_sanity_probe():
    import AnimGraphSchema as _AGS
    import NavSchema as _NS
    from pxr import Tf as _Tf, Usd as _Usd
    _reg = _Usd.SchemaRegistry()
    with open("/tmp/anim_people_schema_state.txt", "w") as _f:
        def _w(msg):
            _f.write(str(msg) + "\n")
        _w("=== schema state right before main() ===")
        for _tname in (
            "AnimGraphSchemaAnimationGraphAPI",
            "NavSchemaNavMeshVolume",
            "NavSchemaNavMeshAreaAPI",
        ):
            _t = _Tf.Type.FindByName(_tname)
            _w(f"  {_tname}: Tf.Type={_t}  "
               f"IsAppliedAPISchema={_reg.IsAppliedAPISchema(_t)}  "
               f"IsConcrete={_reg.IsConcrete(_t)}")
        # Direct HasAPI test on a throwaway prim
        _ctx = omni.usd.get_context()
        _ctx.new_stage()
        _stage = _ctx.get_stage()
        _p = _stage.DefinePrim("/SanityProbe", "Xform")
        try:
            _has = _p.HasAPI(_AGS.AnimationGraphAPI)
            _w(f"  prim.HasAPI(AnimGraphSchema.AnimationGraphAPI) -> {_has}")
        except Exception as _e:
            _w(f"  prim.HasAPI raised: {type(_e).__name__}: {_e}")
        try:
            _vol = _NS.NavMeshVolume.Define(_stage, Sdf.Path("/SanityProbeNav"))
            _ext_tn = UsdGeom.Boundable(_vol.GetPrim()).GetExtentAttr().GetTypeName()
            _w(f"  NavMeshVolume.extent typeName: {_ext_tn}")
        except Exception as _e:
            _w(f"  NavMeshVolume.Define raised: {type(_e).__name__}: {_e}")
_schema_sanity_probe()


# ── Configuration ──────────────────────────────────────────────────────────
GOAL_POSITION = (6.29, 0.0, -1.6)  # Same point used in the GUI test
BIPED_SETUP_PRIM_PATH = "/World/Characters/Biped_Setup"
CHARACTER_PRIM_PATH = "/World/Characters/F_Business_02"
ANIM_GRAPH_PRIM_PATH = "/World/Characters/Biped_Setup/CharacterAnimation/AnimationGraph"
# ───────────────────────────────────────────────────────────────────────────


def pump_app(n=20):
    """Run n app update ticks so async USD/Hydra work can drain."""
    for _ in range(n):
        simulation_app.update()


def find_skel_root(stage, root_prim_path):
    """Walk a referenced character's prim tree and return the first SkelRoot path."""
    root = stage.GetPrimAtPath(root_prim_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() == "SkelRoot":
            return prim.GetPath()
    return None


def define_navmesh_volume(stage, prim_path, center, half_size):
    """Define a NavMeshVolume with explicit bounds — skips the
    auto-bounding-box logic in `CreateNavMeshVolumeCommand`, which depends on
    Hydra having already computed world bounds (unreliable in standalone).

    Replicates the same prim shape the test stage uses:
        def NavMeshVolume "NavMeshVolume" {
            float3[] extent = [(-0.5,-0.5,-0.5),(0.5,0.5,0.5)]
            xformOp:translate = center
            xformOp:scale     = (2*hx, 2*hy, 2*hz)
            xformOpOrder = ["xformOp:translate","xformOp:rotateXYZ","xformOp:scale"]
        }
    """
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


def bake_navmesh_blocking():
    """Trigger a navmesh bake and pump the app until it finishes.

    The navmesh subsystem fires both EVENT_TYPE_NAVMESH_UPDATING (with a
    `progress` payload) and EVENT_TYPE_NAVMESH_UPDATED (final). We treat
    either as a completion signal: UPDATED on its own, or UPDATING with
    progress >= 1.0. The official test (`base_unit_test.bake_navmesh_and_wait`)
    waits for the SAME pair of events.
    """
    inav = nav.acquire_interface()
    bake_done = {"v": False}

    def _on_evt(evt):
        if evt.type == int(nav.EVENT_TYPE_NAVMESH_UPDATING):
            progress = evt.payload.get("progress", 0.0)
            carb.log_warn(f"[anim_people_test] navmesh bake progress: {progress:.2f}")
            if progress >= 1.0:
                bake_done["v"] = True
        elif evt.type == int(nav.EVENT_TYPE_NAVMESH_UPDATED):
            carb.log_warn("[anim_people_test] navmesh bake event: UPDATED")
            bake_done["v"] = True

    # NOTE: keep `sub` alive for the entire wait — the subscription is
    # cancelled when its handle is garbage-collected.
    sub = inav.get_navmesh_event_stream().create_subscription_to_pop(_on_evt)
    carb.log_warn("[anim_people_test] starting navmesh bake")
    inav.start_navmesh_baking()

    for _ in range(600):
        if bake_done["v"]:
            break
        simulation_app.update()

    if not bake_done["v"]:
        carb.log_error("[anim_people_test] navmesh bake did not complete in time")
    sub = None  # release subscription explicitly


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

    # Make sure the parent Xform exists so character paths are clean.
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

    # Pump updates so USD payloads finish loading and Hydra processes the
    # references. Without this the SkelRoot prim may not exist yet, and the
    # navmesh bounding-box compute returns an empty range.
    pump_app(30)

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
    carb.log_warn(f"[anim_people_test] character SkelRoot at {skel_root_path}")

    # Verify the AnimationGraph prim from Biped_Setup actually exists before
    # we try to bind it.
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

    # Create the NavMesh include volume MANUALLY — bypass
    # `CreateNavMeshVolumeCommand`'s auto-bounding-box compute, which depends
    # on Hydra-cached world bounds and is unreliable in standalone.
    define_navmesh_volume(
        stage,
        prim_path="/World/NavMeshVolume",
        center=(0.0, 0.0, 1.0),
        half_size=(25.0, 25.0, 5.0),
    )
    pump_app(5)

    # Bake the navmesh BEFORE world.reset() / Play, same order as the GUI recipe.
    bake_navmesh_blocking()

    # Reset world to bind physics + stage, then start the timeline.
    world.reset()

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    # ag.get_character() only returns a non-None handle once the runtime has
    # picked the character up. Pump frames until that happens.
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
    carb.log_warn("[anim_people_test] character runtime ready, starting GoTo")

    nav_mgr = NavigationManager(
        str(skel_root_path),
        navmesh_enabled=True,
        dynamic_avoidance_enabled=False,
    )

    gx, gy, gz = GOAL_POSITION
    nav_mgr.generate_goto_path([str(gx), str(gy), str(gz), "_"])
    character.set_variable("Action", "Walk")

    # Walk loop ported from omni/anim/people/scripts/goto_goal.py:_walk
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
                carb.log_warn(f"[anim_people_test] reached goal {GOAL_POSITION}")
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
