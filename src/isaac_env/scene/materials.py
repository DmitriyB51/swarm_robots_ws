import os
from pxr import UsdShade, UsdGeom, Sdf, Gf
import carb


def _create_omnipbr(stage, mat_path, albedo=None, color=None,
                    roughness=0.5, metallic=0.0, tiling=1.0):
    material = UsdShade.Material.Define(stage, mat_path)
    shader = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shader.SetSourceAsset("OmniPBR.mdl", "mdl")
    shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")

    if albedo:
        shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset).Set(albedo)
        shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2).Set(
            Gf.Vec2f(tiling, tiling)
        )
    if color:
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(*color)
        )

    shader.CreateInput("reflection_roughness_constant",
                       Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("metallic_constant",
                       Sdf.ValueTypeNames.Float).Set(metallic)
    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def _bind_to_subtree(stage, root_path, material):
    count = 0
    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(Sdf.Path(root_path)):
            if prim.IsA(UsdGeom.Mesh):
                UsdShade.MaterialBindingAPI(prim).Bind(material)
                count += 1
    return count


def apply_all_materials(stage):
    base = os.path.expanduser(
        "~/swarm_robots_ws/src/isaac_env/scene/textures"
    )

    ground_tex = os.path.join(base, "ground_albedo.jpg")
    wall_tex   = os.path.join(base, "wall_albedo.jpg")
    drone_tex  = None
    ugv_tex    = None

    ground_tex = ground_tex if os.path.exists(ground_tex) else None
    wall_tex   = wall_tex   if os.path.exists(wall_tex)   else None

    if ground_tex:
        carb.log_info(f"[materials] Ground texture found: {ground_tex}")
    else:
        carb.log_warn("[materials] Ground texture NOT found, using color fallback")

    if wall_tex:
        carb.log_info(f"[materials] Wall texture found: {wall_tex}")
    else:
        carb.log_warn("[materials] Wall texture NOT found, using color fallback")

    ground_mat = _create_omnipbr(
        stage, "/World/Looks/GroundMat",
        albedo=ground_tex,
        color=(0.4, 0.4, 0.4),
        roughness=0.9, tiling=30.0
    )

    wall_mat = _create_omnipbr(
        stage, "/World/Looks/WallMat",
        albedo=wall_tex,
        color=(0.55, 0.52, 0.48),
        roughness=0.75, tiling=2.0
    )

    # Ground
    ground_prim = stage.GetPrimAtPath(
        "/World/Environment/groundPlane/CollisionMesh"
    )
    if ground_prim:
        UsdShade.MaterialBindingAPI(ground_prim).Bind(ground_mat)
        carb.log_info("[materials] Ground mesh textured.")
    else:
        carb.log_warn("[materials] Ground mesh not found!")

    # The maze uses its own built-in OmniPBR shader directly
    existing_shader = stage.GetPrimAtPath(
        "/World/Environment/Looks/OmniPBR/Shader"
    )
    if existing_shader:
        sh = UsdShade.Shader(existing_shader)
        if wall_tex:
            sh.CreateInput(
                "diffuse_texture", Sdf.ValueTypeNames.Asset
            ).Set(wall_tex)
            sh.CreateInput(
                "texture_scale", Sdf.ValueTypeNames.Float2
            ).Set(Gf.Vec2f(2.0, 2.0))
        sh.CreateInput(
            "diffuse_color_constant", Sdf.ValueTypeNames.Color3f
        ).Set(Gf.Vec3f(0.25, 0.24, 0.22))
        sh.CreateInput(
            "reflection_roughness_constant", Sdf.ValueTypeNames.Float
        ).Set(0.75)
        carb.log_info("[materials] Built-in maze shader updated with texture.")

    carb.log_info("[materials] Environment done.")

    drone_mat = _create_omnipbr(
        stage, "/World/Looks/DroneMat",
        albedo=drone_tex if drone_tex and os.path.exists(drone_tex) else None,
        color=(0.15, 0.15, 0.15),
        roughness=0.3, metallic=0.7
    )
    for name in ("vtol_1", "vtol_2", "vtol_3"):
        count = _bind_to_subtree(stage, f"/World/{name}", drone_mat)
        carb.log_info(f"[materials] {name}: {count} meshes.")

    ugv_mat = _create_omnipbr(
        stage, "/World/Looks/UGVMat",
        albedo=ugv_tex if ugv_tex and os.path.exists(ugv_tex) else None,
        color=(0.2, 0.45, 0.2),
        roughness=0.6, metallic=0.3
    )
    for name in ("ugv_1", "ugv_2", "ugv_3"):
        count = _bind_to_subtree(stage, f"/World/{name}", ugv_mat)
        carb.log_info(f"[materials] {name}: {count} meshes.")

    carb.log_info("[materials] All done.")