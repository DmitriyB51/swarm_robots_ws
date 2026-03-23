import os
import glob
import random
from pxr import UsdGeom, UsdShade, Sdf, Gf
import carb


# Maze boundaries
MAZE_MIN_X = -20.0
MAZE_MAX_X =  20.0
MAZE_MIN_Y = -21.0
MAZE_MAX_Y =  21.0

# Full map boundaries
MAP_MIN_X = -40.0
MAP_MAX_X =  40.0
MAP_MIN_Y = -40.0
MAP_MAX_Y =  40.0

# Number of trees to spawn
NUM_TREES = 3

# Minimum distance between trees
MIN_DIST  = 25.0

ENV_ROOT = "/World/SceneObjects"


def _is_in_maze(x, y):
    """Returns True if point is inside maze bounds."""
    margin = 3.0
    return (MAZE_MIN_X - margin < x < MAZE_MAX_X + margin and
            MAZE_MIN_Y - margin < y < MAZE_MAX_Y + margin)


def _too_close(x, y, existing):
    """Returns True if point is too close to existing trees."""
    for ex, ey in existing:
        if ((x - ex) ** 2 + (y - ey) ** 2) < MIN_DIST ** 2:
            return True
    return False


def _generate_tree_positions(seed=42):
    """Generates random tree positions outside the maze."""
    random.seed(seed)
    positions = []
    attempts = 0
    max_attempts = 1000

    while len(positions) < NUM_TREES and attempts < max_attempts:
        x = random.uniform(MAP_MIN_X, MAP_MAX_X)
        y = random.uniform(MAP_MIN_Y, MAP_MAX_Y)
        attempts += 1

        if _is_in_maze(x, y):
            continue
        if _too_close(x, y, positions):
            continue

        positions.append((x, y))

    if len(positions) < NUM_TREES:
        carb.log_warn(
            f"[environment] Only placed "
            f"{len(positions)}/{NUM_TREES} trees"
        )

    return [(x, y, 0.0) for x, y in positions]


def _find_tree_file():
    base = os.path.expanduser(
        "~/swarm_robots_ws/src/isaac_env/scene/assets"
    )
    for ext in ("*.usdc", "*.usd", "*.usdz"):
        matches = glob.glob(os.path.join(base, ext))
        if matches:
            return matches[0]
    return None


def _make_color_mat(stage, path, color, roughness=0.8):
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset("OmniPBR.mdl", "mdl")
    sh.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
    sh.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    sh.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(*color)
    )
    sh.CreateInput("reflection_roughness_constant",
                   Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

def _add_usd_tree(stage, position, name, usd_path):
    prim_path = f"{ENV_ROOT}/{name}"
    xform = UsdGeom.Xform.Define(stage, prim_path)

    xform_api = UsdGeom.XformCommonAPI(xform)
    xform_api.SetTranslate(Gf.Vec3d(position[0], position[1], 0.0))
    xform_api.SetRotate(Gf.Vec3f(90.0, 0.0, 0.0))
    xform_api.SetScale(Gf.Vec3f(0.1, 0.1, 0.1))

    stage.GetPrimAtPath(prim_path).GetReferences().AddReference(usd_path)
    return prim_path

def _add_procedural_tree(stage, position, name):
    root = f"{ENV_ROOT}/{name}"
    xform = UsdGeom.Xform.Define(stage, root)
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))

    # Realistic tree ~5m tall with variation
    trunk_height = random.uniform(2.5, 3.5)
    crown_radius = random.uniform(1.8, 2.5)
    total_height = trunk_height + crown_radius * 1.4  # ~4.5-6m total

    trunk = UsdGeom.Cylinder.Define(stage, f"{root}/Trunk")
    trunk.GetHeightAttr().Set(trunk_height)
    trunk.GetRadiusAttr().Set(0.12)
    UsdGeom.XformCommonAPI(trunk).SetTranslate((0, 0, trunk_height / 2))

    crown = UsdGeom.Sphere.Define(stage, f"{root}/Crown")
    crown.GetRadiusAttr().Set(crown_radius)
    UsdGeom.XformCommonAPI(crown).SetTranslate((0, 0, trunk_height + crown_radius * 0.7))

    trunk_mat = _make_color_mat(
        stage, f"{root}/TrunkMat", (0.35, 0.20, 0.05), 0.9
    )
    crown_mat = _make_color_mat(
        stage, f"{root}/CrownMat", (0.12, 0.45, 0.08), 0.85
    )
    UsdShade.MaterialBindingAPI(trunk.GetPrim()).Bind(trunk_mat)
    UsdShade.MaterialBindingAPI(crown.GetPrim()).Bind(crown_mat)
    return root


def populate_environment(stage):
    """Spawns 4 trees at fixed positions near maze corners."""
    UsdGeom.Xform.Define(stage, ENV_ROOT)

    tree_usd = _find_tree_file()

    if tree_usd:
        carb.log_info(f"[environment] USD tree found: {tree_usd}")
    else:
        carb.log_warn("[environment] No USD tree found, using procedural trees.")

    # 2 trees near maze entrances (beside, not blocking), 2 along walls
    positions = [
        (-21.5,  -5.0, 0.0),   # near left entrance, offset to the side
        ( 5.0,  -22.0, 0.0),   # near bottom entrance, offset to the side
        ( 22.0,  14.0, 0.0),   # along right wall
        (-14.0,  23.0, 0.0),   # along top wall
    ]

    for i, pos in enumerate(positions):
        name = f"Tree_{i:02d}"
        if tree_usd:
            _add_usd_tree(stage, pos, name, tree_usd)
        else:
            _add_procedural_tree(stage, pos, name)

    carb.log_info(f"[environment] Trees placed: {len(positions)} (fixed corners)")