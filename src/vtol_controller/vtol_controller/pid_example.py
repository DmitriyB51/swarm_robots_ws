# Copyright (c) 2022-2025, The Isaac Lab Project Developers[](https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""
This script demonstrates how to simulate a quadcopter with PID control for stable hovering.
.. code-block:: bash
    # Usage
    ./isaaclab.sh -p scripts/demos/quadcopter.py
"""
"""Launch Isaac Sim Simulator first."""
import argparse
from isaaclab.app import AppLauncher
import math
# add argparse arguments
parser = argparse.ArgumentParser(description="This script demonstrates how to simulate a quadcopter.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
"""Rest everything follows."""
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from isaaclab.utils import configclass
##
# Pre-defined configs
##
# from isaaclab_assets import CRAZYFLIE_CFG # isort:skip
# from vtol_drone.robots.vtol_drone import VTOL_DRONE_CFG # isort:skip
from vtol_quadcopter_ws.robots.vtol_quadcopter import VTOL_DRONE_CFG # isort:skip

class PID:
    def __init__(self, kp: float, ki: float, kd: float, dt: float, device: str):
        self.kp = torch.tensor(kp, device=device)
        self.ki = torch.tensor(ki, device=device)
        self.kd = torch.tensor(kd, device=device)
        self.dt = torch.tensor(dt, device=device)
        self.integral = torch.tensor(0.0, device=device)
        self.prev_error = torch.tensor(0.0, device=device)

    def update(self, error: torch.Tensor) -> torch.Tensor:
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

def quat_to_euler(quat: torch.Tensor) -> torch.Tensor:
    """Convert quaternion (w, x, y, z) to Euler angles (roll, pitch, yaw)."""
    w, x, y, z = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = torch.atan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = torch.asin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = torch.atan2(siny_cosp, cosy_cosp)
    return torch.tensor([roll, pitch, yaw], device=quat.device)

def get_rotation_matrix(quat: torch.Tensor) -> torch.Tensor:
    """Get rotation matrix from quaternion (w, x, y, z)."""
    w, x, y, z = quat
    return torch.tensor([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x**2 - 2*y**2]
    ], device=quat.device)

def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view(eye=[0.5, 0.5, 1.0], target=[0.0, 0.0, 0.5])
    # Spawn things into stage
    # Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # Lights
    cfg = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)
    # Robots
    robot_cfg = VTOL_DRONE_CFG.replace(prim_path="/World/VTOL_Drone")
    robot_cfg.spawn.func("/World/VTOL_Drone", robot_cfg.spawn, translation=robot_cfg.init_state.pos)
    # create handles for the robots
    robot = Articulation(robot_cfg)
    # Play the simulator
    sim.reset()
    # Fetch relevant parameters to make the quadcopter hover in place
    prop_body_ids = robot.find_bodies(".*propeller.*|.*right__.*")[0]
    robot_mass = robot.root_physx_view.get_masses().sum()
    gravity = torch.tensor(sim.cfg.gravity, device=sim.device).norm()
    # Define target position (adjust as needed)
    target_pos = torch.tensor([5.0, 0.0, 5.0], device=sim.device)
    target_yaw = 0.0
    # Base forces for balanced hover (from user's manual adjustment)
    base_force_ratios = torch.tensor([0.2915, 0.2915, 0.21, 0.21], device=sim.device)
    nominal_thrust = robot_mass * gravity
    base_f = nominal_thrust * base_force_ratios
    # PID controllers (tune gains as needed)
    dt = sim.get_physics_dt()
    # Position PIDs
    pid_pos_x = PID(kp=0.3, ki=0.001, kd=0.1, dt=dt, device=sim.device)
    pid_pos_y = PID(kp=0.3, ki=0.001, kd=0.1, dt=dt, device=sim.device)
    pid_pos_z = PID(kp=0.4, ki=0.002, kd=0.15, dt=dt, device=sim.device)

    # Velocity PIDs
    pid_vel_x = PID(kp=0.15, ki=0.025, kd=0.05, dt=dt, device=sim.device)
    pid_vel_y = PID(kp=0.15, ki=0.025, kd=0.05, dt=dt, device=sim.device)
    pid_vel_z = PID(kp=0.25, ki=0.055, kd=0.1, dt=dt, device=sim.device)

    # Attitude PIDs
    pid_roll = PID(kp=16.0, ki=0.0, kd=4.0, dt=dt, device=sim.device)
    pid_pitch = PID(kp=16.0, ki=0.0, kd=4.0, dt=dt, device=sim.device)
    pid_yaw = PID(kp=4.0, ki=0.0, kd=1.5, dt=dt, device=sim.device)
    # Compute allocation matrix A (assuming root is approximate CoM)
    robot.update(dt)  # Update to get initial body positions
    rel_pos = robot.data.body_pos_w[0, prop_body_ids] - robot.data.root_pos_w[0]
    r_x = rel_pos[:, 0]
    r_y = rel_pos[:, 1]
    A = torch.zeros(4, 4, device=sim.device)
    A[0, :] = 1.0
    A[1, :] = r_y  # tau_x contributions
    A[2, :] = -r_x  # tau_y contributions
    s = torch.tensor([-1.0, 1.0, 1.0, -1.0], device=sim.device)  # Prop directions (adjust signs if needed)
    k = 0.01  # Torque-to-thrust ratio (tune as needed)
    A[3, :] = s * k  # tau_z contributions
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    # Simulate physics
    while simulation_app.is_running():
        if count % 20 == 0:  # print every 200 steps
            robot.update(sim_dt)
            pos = robot.data.root_pos_w[0]
            quat = robot.data.root_quat_w[0]  # quaternion
            print(f"[Pose] Position: {pos.cpu().numpy()}, Orientation: {quat.cpu().numpy()}")
            print(f"roll pitch yaw: {quat_to_euler(quat).cpu().numpy()}")
            print(f"velocity: {robot.data.root_lin_vel_w[0].cpu().numpy()}")
        # reset
        if count % 7000 == 0:
            # reset counters
            sim_time = 0.0
            count = 0
            # reset dof state
            joint_pos, joint_vel = robot.data.default_joint_pos, robot.data.default_joint_vel
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.write_root_pose_to_sim(robot.data.default_root_state[:, :7])
            robot.write_root_velocity_to_sim(robot.data.default_root_state[:, 7:])
            robot.reset()
            # Reset PIDs
            for pid in [pid_pos_x, pid_pos_y, pid_pos_z, pid_vel_x, pid_vel_y, pid_vel_z, pid_roll, pid_pitch, pid_yaw]:
                pid.integral = torch.tensor(0.0, device=sim.device)
                pid.prev_error = torch.tensor(0.0, device=sim.device)
                        # reset command
            print(">>>>>>>> Reset!")
        # Get current state
        pos = robot.data.root_pos_w[0]
        vel = robot.data.root_lin_vel_w[0]
        quat = robot.data.root_quat_w[0]
        euler = quat_to_euler(quat)
        roll, pitch, yaw = euler
        # Position control (x, y, z)
        error_x = target_pos[0] - pos[0]
        desired_vx = pid_pos_x.update(error_x)
        error_vx = desired_vx - vel[0]
        desired_ax = pid_vel_x.update(error_vx)
        error_y = target_pos[1] - pos[1]
        desired_vy = pid_pos_y.update(error_y)
        error_vy = desired_vy - vel[1]
        desired_ay = pid_vel_y.update(error_vy)
        error_z = target_pos[2] - pos[2]
        desired_vz = pid_pos_z.update(error_z)
        error_vz = desired_vz - vel[2]
        desired_az = pid_vel_z.update(error_vz)
        # Compute desired thrust



        
        # Compute desired attitudes (small angle approximation)
        desired_pitch = desired_ax / gravity
        desired_roll = -desired_ay / gravity
        desired_yaw = target_yaw

        desired_pitch = torch.clamp(desired_pitch, -torch.pi/4, torch.pi/4)  # Limit to +/-45° (0.785 rad)
        desired_roll = torch.clamp(desired_roll, -torch.pi/4, torch.pi/4)

        # Attitude control
        error_roll = desired_roll - roll
        desired_tau_x = pid_roll.update(error_roll)
        error_pitch = desired_pitch - pitch
        desired_tau_y = pid_pitch.update(error_pitch)
        error_yaw = desired_yaw - yaw
        desired_tau_z = pid_yaw.update(error_yaw)

        # thrust_scaling = 1.0 / (math.cos(desired_roll) * math.cos(desired_pitch)) if abs(desired_roll) < math.pi/2 and abs(desired_pitch) < math.pi/2 else 1.0
        # desired_thrust = robot_mass * (desired_az + gravity)
        # desired_thrust *= thrust_scaling

        # Thrust compensation for tilt (feedforward)
        thrust_scaling = torch.tensor(1.0, device=sim.device) / (torch.cos(desired_pitch) * torch.cos(desired_roll))
        thrust_scaling = torch.clamp(thrust_scaling, max=torch.tensor(5.0, device=sim.device))

        desired_thrust = robot_mass * (desired_az + gravity)
        desired_thrust *= thrust_scaling



        # Compute forces
        forces = torch.zeros(robot.num_instances, 4, 3, device=sim.device)
        torques = torch.zeros_like(forces)
        # Scale base for thrust
        scaling = desired_thrust / nominal_thrust
        f_thrust = base_f * scaling
        # Deltas for torques
        u_tau = torch.tensor([0.0, desired_tau_x, desired_tau_y, desired_tau_z], device=sim.device)
        delta_f = torch.linalg.solve(A, u_tau)
        # Total propeller forces (magnitudes)
        f = f_thrust + delta_f
        # Get thrust direction
        unit_z = get_rotation_matrix(quat) @ torch.tensor([0.0, 0.0, 1.0], device=sim.device)
        if unit_z[2] > 0.1:
            desired_thrust /= unit_z[2]
        # Apply to each propeller
        for i in range(4):
            forces[0, i, :] = f[i] * unit_z
            torques[0, i, :] = s[i] * k * f[i] * unit_z  # Adjust sign of s if yaw direction is wrong
        # Apply actions
        robot.set_external_force_and_torque(forces, torques, body_ids=prop_body_ids)
        robot.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        robot.update(sim_dt)

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()