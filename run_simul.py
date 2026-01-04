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
import time
import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sim import SimulationContext


import omni
import carb
import go2.go2_ctrl as go2_ctrl
from go2.go2_env import go2_rl_env, Go2RLEnvCfg
from envs.usdz_import import GS_import
from go2.go2_sensors import sensor_manager
import ros2.go2_ros2_bridge as go2_ros2_bridge


FILE_PATH = os.path.join(os.path.dirname(__file__), "config")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    go2_env_cfg = Go2RLEnvCfg()
    go2_env_cfg.decimation = math.ceil(1./go2_env_cfg.sim.dt/cfg.freq)
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)

    env, policy = go2_rl_env(go2_env_cfg, cfg)
    time.sleep(10)

    # Sensor setup
    sm = sensor_manager(cfg.num_envs)
    # cameras = sm.create_camera()
    lidar_annotators = sm.create_lidar()

    # Keyboard control
    system_input = carb.input.acquire_input_interface()
    system_input.subscribe_to_keyboard_events(omni.appwindow.get_default_app_window().get_keyboard(), go2_ctrl.sub_keyboard_event)
    
    # ROS2 Bridge
    rclpy.init()
    #run simulation
    dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    obs, _ = env.reset()
    cameras = env.unwrapped.scene["front_cam"]
    dm = go2_ros2_bridge.RobotDataManager(env, lidar_annotators, cameras, cfg)

    print("[INFO]: simulation started")


    while simulation_app.is_running():
        start_time = time.time()
        with torch.inference_mode():
                
                actions = policy(obs)

                obs, _, _, _ = env.step(actions)

                # ROS2 data
                dm.pub_ros2_data()
                rclpy.spin_once(dm)

        elapsed_time = time.time() - start_time

        sleep_time = dt - (elapsed_time)
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, dt/elapsed_time)
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end='', flush=True)
    dm.destroy_node()
    rclpy.shutdown()
    simulation_app.close()

if __name__ == "__main__":
    GS_import()
    run_simulator()
    