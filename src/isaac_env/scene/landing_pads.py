from pxr import UsdGeom, UsdShade, UsdPhysics, Sdf, Gf
import carb

# Landing pad positions
PAD_POSITIONS = [
    (-31.0, 30.0, 0.0),  # pad for vtol_1
    (-34.0, 35.0, 0.0),  # pad for vtol_2
    (-34.0, 25.0, 0.0),  # pad for vtol_3
]

PAD_SIZE   = 3.0   # meters, slightly larger than drone footprint
PAD_HEIGHT = 0.05  # thin slab


def _create_pad_material(stage, path):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset("OmniPBR.mdl", "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    # Blue pad
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(0.1, 0.5, 0.9)
    )
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(0.8)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _create_landing_pad(stage, position, name):
    root = f"/World/LandingPads/{name}"

    # Root xform
    xform = UsdGeom.Xform.Define(stage, root)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    # Pad geometry — flat cube
    pad_path = f"{root}/Pad"
    cube = UsdGeom.Cube.Define(stage, pad_path)
    cube.GetSizeAttr().Set(1.0)

    # Scale to pad dimensions
    UsdGeom.XformCommonAPI(cube).SetScale(
        Gf.Vec3f(PAD_SIZE, PAD_SIZE, PAD_HEIGHT)
    )
    UsdGeom.XformCommonAPI(cube).SetTranslate(
        Gf.Vec3d(0.0, 0.0, PAD_HEIGHT / 2)
    )

    # Static collision so drones can land on it
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
    rigid = UsdPhysics.RigidBodyAPI(xform.GetPrim())
    rigid.CreateKinematicEnabledAttr(True)  # static, doesn't move

    # Material
    mat = _create_pad_material(stage, f"{root}/PadMat")
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(mat)

    # H marker — three white bars forming letter H
    h_color = Gf.Vec3f(1.0, 1.0, 1.0)
    h_z = PAD_HEIGHT + 0.02
    h_thick = 0.15
    h_height = 0.02
    h_len = 1.2
    h_cross = 0.7

    for bar_name, sx, sy, tx, ty in [
        ("H_left",  h_len,   h_thick,  0.0, -0.4),
        ("H_right", h_len,   h_thick,  0.0,  0.4),
        ("H_cross", h_thick, h_cross,  0.0,  0.0),
    ]:
        bar = UsdGeom.Cube.Define(stage, f"{root}/{bar_name}")
        bar.GetSizeAttr().Set(1.0)
        UsdGeom.XformCommonAPI(bar).SetScale(Gf.Vec3f(sx, sy, h_height))
        UsdGeom.XformCommonAPI(bar).SetTranslate(Gf.Vec3d(tx, ty, h_z))

        bar_mat = UsdShade.Material.Define(stage, f"{root}/{bar_name}Mat")
        bar_sh  = UsdShade.Shader.Define(stage, f"{root}/{bar_name}Mat/Shader")
        bar_sh.SetSourceAsset("OmniPBR.mdl", "mdl")
        bar_sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        bar_sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
        bar_sh.CreateInput(
            "diffuse_color_constant", Sdf.ValueTypeNames.Color3f
        ).Set(h_color)
        bar_sh.CreateInput(
            "emissive_color", Sdf.ValueTypeNames.Color3f
        ).Set(Gf.Vec3f(0.3, 0.3, 0.3))
        bar_mat.CreateSurfaceOutput().ConnectToSource(
            bar_sh.ConnectableAPI(), "surface"
        )
        UsdShade.MaterialBindingAPI(bar.GetPrim()).Bind(bar_mat)

    carb.log_info(f"[landing_pads] Created: {root} at {position}")
    return root


def create_landing_pads(stage):
    """Creates 3 landing pads in the bottom-left corner."""
    UsdGeom.Xform.Define(stage, "/World/LandingPads")

    for i, pos in enumerate(PAD_POSITIONS):
        _create_landing_pad(stage, pos, f"Pad_{i+1}")

    carb.log_info(f"[landing_pads] Total pads: {len(PAD_POSITIONS)}")