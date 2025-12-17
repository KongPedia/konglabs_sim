import os
import hydra
import rclpy
import torch
import time
import math
import argparse
from isaaclab.app import AppLauncher
# add argparse arguments
parser = argparse.ArgumentParser(description="Unitree go2 ros2 setup")

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
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext

from go2.go2_env import Myscene


FILE_PATH = os.path.join(os.path.dirname(__file__), "config")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    sim_cfg = sim_utils.SimulationCfg(device=args_cli.device)
    sim = SimulationContext(sim_cfg)
    sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])
    scene_cfg = Myscene(num_envs=cfg.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # play the simulator
    sim.reset()
    print("[INFO]: simulation started")
    sim_dt = sim.get_physics_dt()
    count = 0
    robot = scene["go2"]
    while simulation_app.is_running():
        if count % 300 == 0:
            count = 0
            root_state = robot.data.default_root_state.clone()
            root_state[:,:3] += scene.env_origins
            robot.write_root_pose_to_sim(root_state[:,:7])
            robot.write_root_velocity_to_sim(root_state[:,7:])
            joint_pos, joint_vel = robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone()
            joint_pos += torch.rand_like(joint_pos) * 0.1
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            scene.reset()
            print("[INFO]: Resetting robot scene")

        joint_pos_target = torch.randn_like(robot.data.joint_pos) * 0.1
        # apply action to the robot
        robot.set_joint_position_target(joint_pos_target)
        # -- write data to sim
        scene.write_data_to_sim()
        # Perform step
        sim.step()
        # Increment counter
        count += 1
        # Update buffers
        scene.update(sim_dt)        
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    