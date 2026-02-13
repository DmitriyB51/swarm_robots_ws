import argparse
import threading
import sys
import time
import math
from isaaclab.app import AppLauncher
# CLI args
parser = argparse.ArgumentParser(description="Quadcopter position control with dynamic PID and keyboard input.")
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
# Launch app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation
from isaaclab.sim import SimulationContext
from pynput import keyboard
from vtol_quadcopter_ws.robots.vtol_quadcopter import VTOL_DRONE_CFG as CRAZYFLIE_CFG

# ---------------- PID Controller ----------------
class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(None, None), angular=False):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0
        self.min_output, self.max_output = output_limits
        self.angular = angular
        
    def update(self, target, current):
        error = target - current
        if self.angular:
            error = math.atan2(math.sin(error), math.cos(error))
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        if self.max_output is not None:
            output = min(self.max_output, output)
        if self.min_output is not None:
            output = max(self.min_output, output)
        return output

# ---------------- Quadcopter Controller ----------------
class QuadcopterController:
    def __init__(self, kp=8.0, ki=3.0, kd=5.0, kp_angular=5.0, ki_angular=1.0, kd_angular=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kp_angular = kp_angular
        self.ki_angular = ki_angular
        self.kd_angular = kd_angular
        
    def create_pid_controllers(self, dt, gravity, angular_limits=(-5.0, 5.0)):
        """Create PID controllers for all degrees of freedom"""
        output_limits = (-gravity.item(), gravity.item())
        pid_x = PIDController(self.kp, self.ki, self.kd, dt, output_limits)
        pid_y = PIDController(self.kp, self.ki, self.kd, dt, output_limits)
        pid_z = PIDController(self.kp, self.ki, self.kd, dt, output_limits)
        pid_yaw = PIDController(self.kp_angular, self.ki_angular, self.kd_angular, dt, angular_limits, angular=True)
        pid_pitch = PIDController(self.kp_angular, self.ki_angular, self.kd_angular, dt, angular_limits, angular=True)
        pid_roll = PIDController(self.kp_angular, self.ki_angular, self.kd_angular, dt, angular_limits, angular=True)
        
        return {
            'x': pid_x, 'y': pid_y, 'z': pid_z,
            'yaw': pid_yaw, 'pitch': pid_pitch, 'roll': pid_roll
        }

# ---------------- Quadcopter Agent ----------------
class QuadcopterAgent:
    def __init__(self, name, articulation, is_master=False, offset=None, goal_pose=None):
        self.name = name
        self.articulation = articulation
        self.is_master = is_master
        self.offset = offset if offset else {"x": 0.0, "y": 0.0, "z": 0.0}
        self.goal_pose = goal_pose if goal_pose else {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}
        self.current_pose = {"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}
        self.pids = None
        self.prop_body_ids = None
        self.base_body_ids = None
        
    def initialize_physics(self, sim):
        """Initialize physics-related properties"""
        self.prop_body_ids = self.articulation.find_bodies(".*propeller.*|.*right__.*")[0]
        self.base_body_ids = self.articulation.find_bodies("base_link")[0]
        
    def update_state(self):
        """Update current pose from simulation data"""
        pos = self.articulation.data.root_pos_w[0]
        quat = self.articulation.data.root_quat_w[0].tolist()
        
        self.current_pose["x"] = pos[0].item()
        self.current_pose["y"] = pos[1].item()
        self.current_pose["z"] = pos[2].item()
        self.current_pose["yaw"] = self.get_yaw(quat)
        
        return self.get_roll(quat), self.get_pitch(quat)
    
    def update_goal_from_master(self, master_pose):
        """Update goal pose based on master's current pose and offset (for slaves)"""
        if not self.is_master:
            yaw_m = master_pose["yaw"]
            dx_world = self.offset["x"] * math.cos(yaw_m) - self.offset["y"] * math.sin(yaw_m)
            dy_world = self.offset["x"] * math.sin(yaw_m) + self.offset["y"] * math.cos(yaw_m)
            
            self.goal_pose["x"] = master_pose["x"] + dx_world
            self.goal_pose["y"] = master_pose["y"] + dy_world
            self.goal_pose["z"] = master_pose["z"] + self.offset["z"]
            self.goal_pose["yaw"] = master_pose["yaw"]
    
    @staticmethod
    def get_roll(quat):
        """Extract roll from quaternion (w, x, y, z)."""
        sinr_cosp = 2 * (quat[0] * quat[1] + quat[2] * quat[3])
        cosr_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2])
        return math.atan2(sinr_cosp, cosr_cosp)

    @staticmethod
    def get_pitch(quat):
        """Extract pitch from quaternion (w, x, y, z)."""
        sinp = 2 * (quat[0] * quat[2] - quat[3] * quat[1])
        if abs(sinp) >= 1:
            return math.copysign(math.pi / 2, sinp)
        else:
            return math.asin(sinp)

    @staticmethod
    def get_yaw(quat):
        """Extract yaw from quaternion (w, x, y, z)."""
        siny_cosp = 2 * (quat[0] * quat[3] + quat[1] * quat[2])
        cosy_cosp = 1 - 2 * (quat[2] * quat[2] + quat[3] * quat[3])
        return math.atan2(siny_cosp, cosy_cosp)

# ---------------- Quadcopter Fleet Manager ----------------
class QuadcopterFleet:
    def __init__(self, sim, controller_config=None):
        self.sim = sim
        self.agents = []
        self.controller_config = controller_config if controller_config else {}
        self.gravity = torch.tensor(sim.cfg.gravity, device=sim.device).norm()
        self.robot_mass = None
        self.tilt_factor = 0.01
        
    def add_agent(self, name, articulation, is_master=False, offset=None, initial_pose=None):
        """Add a quadcopter agent to the fleet"""
        agent = QuadcopterAgent(name, articulation, is_master, offset, initial_pose)
        self.agents.append(agent)
        return agent
    
    def initialize_fleet(self):
        """Initialize all agents in the fleet"""
        if not self.agents:
            raise ValueError("No agents in the fleet")
            
        # Get robot mass from first agent (assuming all are same)
        self.robot_mass = self.agents[0].articulation.root_physx_view.get_masses().sum()
        print(f"[INFO]: robot_mass={self.robot_mass:.4f} gravity={self.gravity:.4f}")
        
        # Create controller for all agents
        controller = QuadcopterController(**self.controller_config)
        dt = self.sim.get_physics_dt()
        
        for agent in self.agents:
            agent.initialize_physics(self.sim)
            agent.pids = controller.create_pid_controllers(dt, self.gravity)
    
    def update_goals(self):
        """Update goal positions for all slave agents based on master"""
        master_agent = None
        for agent in self.agents:
            if agent.is_master:
                master_agent = agent
                break
        
        if master_agent:
            for agent in self.agents:
                if not agent.is_master:
                    agent.update_goal_from_master(master_agent.current_pose)
    
    def apply_control(self, agent):
        """Apply PID control to a single agent"""
        current_roll, current_pitch = agent.update_state()
        
        # Position control
        a_x = agent.pids['x'].update(agent.goal_pose["x"], agent.current_pose["x"])
        a_y = agent.pids['y'].update(agent.goal_pose["y"], agent.current_pose["y"])
        a_z = agent.pids['z'].update(agent.goal_pose["z"], agent.current_pose["z"])
        
        # Attitude control
        front = a_x * math.cos(agent.goal_pose["yaw"]) + a_y * math.sin(agent.goal_pose["yaw"])
        side = -a_x * math.sin(agent.goal_pose["yaw"]) + a_y * math.cos(agent.goal_pose["yaw"])
        
        desired_pitch = -self.tilt_factor * front
        desired_roll = self.tilt_factor * side
        
        tau_x = agent.pids['roll'].update(desired_roll, current_roll)
        tau_y = agent.pids['pitch'].update(desired_pitch, current_pitch)
        tau_z = agent.pids['yaw'].update(agent.goal_pose["yaw"], agent.current_pose["yaw"])
        
        # Calculate forces
        u = torch.tensor([a_x, a_y, a_z + self.gravity.item()], device=self.sim.device)
        u_norm = u.norm()
        direction = u / u_norm if u_norm > 0 else torch.tensor([0.0, 0.0, 1.0], device=self.sim.device)
        
        total_thrust_mag = self.robot_mass * u_norm
        per_prop_force = (total_thrust_mag / 4.0) * direction
        
        # Apply propeller forces
        forces = torch.zeros(agent.articulation.num_instances, 4, 3, device=self.sim.device)
        forces[..., 0] = per_prop_force[0]
        forces[..., 1] = per_prop_force[1]
        forces[..., 2] = per_prop_force[2]
        torques = torch.zeros_like(forces)
        
        agent.articulation.set_external_force_and_torque(forces, torques, body_ids=agent.prop_body_ids)
        
        # Apply base torques
        base_forces = torch.zeros(agent.articulation.num_instances, 1, 3, device=self.sim.device)
        base_torques = torch.zeros_like(base_forces)
        base_torques[..., 0] = tau_x
        base_torques[..., 1] = tau_y
        base_torques[..., 2] = tau_z
        
        agent.articulation.set_external_force_and_torque(base_forces, base_torques, body_ids=agent.base_body_ids)
        
        agent.articulation.write_data_to_sim()
    
    def control_all_agents(self):
        """Apply control to all agents in the fleet"""
        for agent in self.agents:
            self.apply_control(agent)

# ---------------- Input Handlers ----------------
def keyboard_control_thread(goal_ref, current_state):
    """Listens for keyboard input and updates goal position accordingly."""
    step = 0.1  # meters per key press
    step_yaw = 0.035  # radians per key press (~5.7 degrees)
    
    def on_press(key):
        try:
            dx_local = 0.0
            dy_local = 0.0
            dz = 0.0
            dyaw = 0.0
            
            if key.char == "w":
                dx_local = step
                label = "Forward"
            elif key.char == "s":
                dx_local = -step
                label = "Backward"
            elif key.char == "a":
                dy_local = step 
                label = "Right"
            elif key.char == "d":
                dy_local = -step 
                label = "Left"
            elif key.char == "i":
                dz = step
                label = "Up"
            elif key.char == "k":
                dz = -step
                label = "Down"
            elif key.char == "q":
                dyaw = step_yaw
                label = "Rotate left"
            elif key.char == "e":
                dyaw = -step_yaw
                label = "Rotate right"
            else:
                return
            
            if dx_local != 0 or dy_local != 0:
                yaw = current_state["yaw"]
                dx_world = dx_local * math.cos(yaw) - dy_local * math.sin(yaw)
                dy_world = dx_local * math.sin(yaw) + dy_local * math.cos(yaw)
                goal_ref["x"] += dx_world
                goal_ref["y"] += dy_world
                print(f"[KEYBOARD] {label} -> goal x = {goal_ref['x']:.2f}, y = {goal_ref['y']:.2f}")
            elif dz != 0:
                goal_ref["z"] += dz
                print(f"[KEYBOARD] {label} -> z = {goal_ref['z']:.2f}")
            elif dyaw != 0:
                goal_ref["yaw"] += dyaw
                print(f"[KEYBOARD] {label} -> yaw = {goal_ref['yaw']:.2f}")
        except AttributeError:
            pass  # Ignore special keys like Shift or Ctrl
            
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

def goal_input_thread(goal_ref):
    """Thread for manual goal input"""
    while True:
        try:
            new_val = input("\nEnter new target x y z yaw (meters, radians): ")
            if new_val.strip() == "":
                continue
            vals = list(map(float, new_val.split()))
            if len(vals) != 4:
                raise ValueError
            goal_ref["x"], goal_ref["y"], goal_ref["z"], goal_ref["yaw"] = vals
            print(f"[USER] Updated goal position -> ({vals[0]:.2f}, {vals[1]:.2f}, {vals[2]:.2f}, {vals[3]:.2f}) m/rad")
        except ValueError:
            print("[WARN] Invalid input, please enter four numeric values separated by spaces.")
        except Exception:
            break

# ---------------- Main Simulation ----------------
def main():
    # Get number of slave drones from user
    try:
        num_slaves = int(input("Enter number of slave drones: "))
    except ValueError:
        print("Invalid input. Using default of 1 slave drone.")
        num_slaves = 1
    
    # Simulation setup
    sim_cfg = sim_utils.SimulationCfg(dt=0.005, device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[1.5, 1.5, 1.0], target=[0.5, 0.5, 0.5])
    
    # Environment setup
    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/Ground", ground_cfg)
    light_cfg = sim_utils.DistantLightCfg(intensity=3000.0)
    light_cfg.func("/World/Light", light_cfg)
    
    # Create fleet manager
    fleet = QuadcopterFleet(sim, controller_config={
        'kp': 8.0, 'ki': 3.0, 'kd': 5.0,
        'kp_angular': 5.0, 'ki_angular': 1.0, 'kd_angular': 2.0
    })
    
    # Spawn master quadcopter
    master_cfg = CRAZYFLIE_CFG.replace(prim_path="/World/master")
    master_cfg.spawn.func("/World/master", master_cfg.spawn, translation=master_cfg.init_state.pos)
    master_articulation = Articulation(master_cfg)
    
    master_agent = fleet.add_agent(
        "master", 
        master_articulation, 
        is_master=True,
        initial_pose={"x": 0.0, "y": 0.0, "z": 1.0, "yaw": 0.0}
    )
    
    # Spawn slave quadcopters with offsets
    for i in range(num_slaves):
        slave_name = f"slave_{i+1}"
        slave_cfg = CRAZYFLIE_CFG.replace(prim_path=f"/World/{slave_name}")
        
        # Calculate offset relative to previous drone
        offset_x = 3.0 * (i + 1)
        offset_y = 3.0 * (i + 1)
        offset_z = -0.5 * (i + 1)
        
        slave_cfg.init_state.pos = [offset_x, offset_y, offset_z]
        slave_cfg.spawn.func(f"/World/{slave_name}", slave_cfg.spawn, translation=slave_cfg.init_state.pos)
        slave_articulation = Articulation(slave_cfg)
        
        slave_agent = fleet.add_agent(
            slave_name,
            slave_articulation,
            is_master=False,
            offset={"x": offset_x, "y": offset_y, "z": offset_z},
            initial_pose={"x": offset_x, "y": offset_y, "z": offset_z, "yaw": 0.0}
        )
    
    sim.reset()
    fleet.initialize_fleet()
    
    # Start input threads for master
    threading.Thread(target=goal_input_thread, args=(master_agent.goal_pose,), daemon=True).start()
    threading.Thread(target=keyboard_control_thread, args=(master_agent.goal_pose, master_agent.current_pose,), daemon=True).start()
    
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0
    
    print(f"[INFO] Simulation running with 1 master and {num_slaves} slave(s)...")
    print("[INFO] Use W/A/S/D/Q/E keys to move or type new x y z yaw values anytime!")
    
    while simulation_app.is_running():
        # Update all agents
        for agent in fleet.agents:
            agent.articulation.update(sim_dt)
        
        # Update slave goals based on master position
        fleet.update_goals()
        
        # Apply control to all agents
        fleet.control_all_agents()
        
        sim.step()
        sim_time += sim_dt
        count += 1
        
        # Print status periodically
        if count % 20 == 0:
            for agent in fleet.agents:
                print(f"[{agent.name.capitalize()} Pose t={sim_time:.2f}s] "
                      f"Pos=({agent.current_pose['x']:.2f},{agent.current_pose['y']:.2f},{agent.current_pose['z']:.2f}) "
                      f"Yaw={agent.current_pose['yaw']:.2f} | "
                      f"Goal=({agent.goal_pose['x']:.2f},{agent.goal_pose['y']:.2f},{agent.goal_pose['z']:.2f},{agent.goal_pose['yaw']:.2f})")
            print("-" * 80)
    
    simulation_app.close()

if __name__ == "__main__":
    main()