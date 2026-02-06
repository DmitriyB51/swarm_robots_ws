from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import math
import numpy as np

from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.asset.importer.urdf import _urdf
import omni.kit.commands

# ---------- WORLD ----------
my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()
set_camera_view(
    eye=[3.0, 0.0, 2.0],
    target=[0.0, 0.0, 0.5],
    camera_prim_path="/OmniverseKit_Persp",
)

stage_units = get_stage_units()

# ---------- IMPORT URDF ----------
urdf_path = r"C:\Users\user\Desktop\akiya_description\urdf\akiya.urdf"

urdf_interface = _urdf.acquire_urdf_interface()
import_config = _urdf.ImportConfig()
import_config.fix_base = True   # двигаем дрон сами, не физикой

result, robot_model = omni.kit.commands.execute(
    "URDFParseFile",
    urdf_path=urdf_path,
    import_config=import_config,
)
result, prim_path = omni.kit.commands.execute(
    "URDFImportRobot",
    urdf_robot=robot_model,
    import_config=import_config,
)

print("URDF prim path:", prim_path)

drone = Articulation(prim_paths_expr=prim_path, name="drone")

my_world.reset()

# ---------- INITIAL POSE ----------
start_height = 0.5
cruise_height = 1.5
base_pos = np.array([0.0, 0.0, start_height], dtype=float)

# твой базовый yaw (-270°), который даёт “ровный” дрон
base_yaw = math.radians(-270.0)

def euler_to_quat(rx: float, ry: float, rz: float):
    cx, sx = math.cos(rx/2), math.sin(rx/2)
    cy, sy = math.cos(ry/2), math.sin(ry/2)
    cz, sz = math.cos(rz/2), math.sin(rz/2)
    qw = cx*cy*cz + sx*sy*sz
    qx = sx*cy*cz - cx*sy*sz
    qy = cx*sy*cz + sx*cy*sz
    qz = cx*cy*sz - sx*sy*cz
    return qx, qy, qz, qw

qx, qy, qz, qw = euler_to_quat(0.0, 0.0, base_yaw)
orientations = np.array([[qx, qy, qz, qw]], dtype=np.float32)

drone.set_world_poses(
    positions=(base_pos / stage_units)[None, :],
    orientations=orientations,
)

print("num_dof:", drone.num_dof)
dofs = drone.num_dof

# ---------- PROPELLERS ----------
base_spin_vel = np.array([80.0, -80.0, 80.0, -80.0], dtype=np.float32)  # rad/s
num_rotors = min(4, dofs)
rotor_angles = np.zeros(num_rotors, dtype=np.float32)

# ---------- ALTITUDE PHYSICS + PID ----------
dt = 1.0 / 60.0
step = 0

mass = 1.0
g = 9.81
k_thrust = 25.0

z = start_height
vz = 0.0

Kp = 2.0
Ki = 0.8
Kd = 0.7

integral_err = 0.0
prev_err = 0.0

# фазы:
# 0–t_takeoff   : взлёт (target вверх)
# t_takeoff–t_hover : висим на высоте
# t_hover–t_land_end: плавная посадка
t_takeoff = 3.0
t_hover = t_takeoff + 5.0
t_land_end = t_hover + 3.0

hover_throttle = (mass * g) / k_thrust  # ~0.39

print("🚁 TAKEOFF → HOVER → LAND (props spin, корпус не переворачивается)")

while simulation_app.is_running():
    t = step * dt

    # ---------- ALTITUDE TARGET BY PHASE ----------
    if t < t_takeoff:
        # взлёт: плавно растим z_target
        s = t / t_takeoff
        s_eased = 0.5 - 0.5 * math.cos(math.pi * s)
        z_target = start_height + (cruise_height - start_height) * s_eased

    elif t_takeoff <= t < t_hover:
        # висим
        z_target = cruise_height

    elif t_hover <= t < t_land_end:
        # плавная посадка
        s = (t - t_hover) / (t_land_end - t_hover)
        s_eased = 0.5 - 0.5 * math.cos(math.pi * s)
        z_target = cruise_height - (cruise_height - start_height) * s_eased

    else:
        # после посадки стоим на земле и выключаем тягу
        z_target = start_height

    # ---------- ALTITUDE PID (только пока летим) ----------
    if t < t_land_end:
        error = z_target - z

        integral_err += error * dt
        integral_err = max(-2.0, min(2.0, integral_err))

        derivative = (error - prev_err) / dt if step > 0 else 0.0
        prev_err = error

        u = Kp * error + Ki * integral_err + Kd * derivative
        throttle = hover_throttle + u
        throttle = max(0.0, min(1.0, throttle))
    else:
        # после посадки — двигатели глушим
        throttle = 0.0
        z = start_height
        vz = 0.0
        integral_err = 0.0
        prev_err = 0.0

    # ---------- ROTOR SPEEDS ----------
    rotor_omega = base_spin_vel[:num_rotors] * throttle

    # обновляем углы пропов
    rotor_angles += rotor_omega * dt
    rotor_angles = (rotor_angles + math.pi) % (2 * math.pi) - math.pi

    joint_pos = np.zeros((1, dofs), dtype=np.float32)
    joint_pos[0, :num_rotors] = rotor_angles
    drone.set_joint_positions(joint_pos)

    joint_vel = np.zeros((1, dofs), dtype=np.float32)
    joint_vel[0, :num_rotors] = rotor_omega
    drone.set_joint_velocities(joint_vel)

    # ---------- VERTICAL DYNAMICS ----------
    if t < t_land_end:
        T = k_thrust * throttle
        damping = 1.5
        az = (T / mass) - g - damping * vz

        vz += az * dt
        vz = max(-3.0, min(3.0, vz))
        z += vz * dt

        if z < start_height:
            z = start_height
            vz = 0.0
    else:
        z = start_height
        vz = 0.0

    # ---------- POSITION (стоим по x,y) ----------
    x = 0.0
    y = 0.0

    # ориентация — ВСЕГДА базовая, без yaw-кручения корпуса
    qx, qy, qz, qw = euler_to_quat(0.0, 0.0, base_yaw)
    orientations = np.array([[qx, qy, qz, qw]], dtype=np.float32)

    # ---------- APPLY POSE ----------
    base_pos[:] = [x, y, z]
    positions = (base_pos / stage_units)[None, :]

    drone.set_world_poses(
        positions=positions,
        orientations=orientations,
    )

    my_world.step(render=True)
    step += 1
