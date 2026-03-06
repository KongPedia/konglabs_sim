import os
import hydra
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

args_cli.headless = False 
args_cli.enable_cameras = True
args_cli.kit_args = "--/renderer/multiGpu/enabled=true --/renderer/multiGpu/maxGpuCount=2 --enable isaacsim.asset.gen.omap" 

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app



import omni.kit.app
ext_manager = omni.kit.app.get_app().get_extension_manager()

# 확장 프로그램 활성화 (최신 ID 사용 권장)
extension_id = "isaacsim.ros2.bridge" 
ext_manager.set_extension_enabled_immediate(extension_id, True)
ext_manager.set_extension_enabled_immediate("omni.anim.graph.core", True)
ext_manager.set_extension_enabled_immediate("omni.anim.graph.bundle", True)
ext_manager.set_extension_enabled_immediate("omni.anim.graph.ui", True)


"""Rest everything follows."""

import torch
import time
import isaaclab.sim as sim_utils
from isaacsim.core.utils.prims import create_prim
import omni.usd
import omni.anim.graph.core as ag
import omni
import carb
import go2.go2_ctrl as go2_ctrl
from go2.go2_env import go2_rl_env, Go2RLEnvCfg
from go2_utils.omap_gen import generate_nav2_map
from go2.go2_sensors import sensor_manager
import ros2.go2_ros2_bridge as go2_ros2_bridge
import envs.sim_env as sim_env
from isaaclab.assets import Articulation, ArticulationCfg
from isaaclab.actuators import ImplicitActuatorCfg

FILE_PATH = os.path.join(os.path.dirname(__file__), "config")
@hydra.main(config_path=FILE_PATH, config_name="sim", version_base=None)
def run_simulator(cfg):
    go2_env_cfg = Go2RLEnvCfg()
    go2_env_cfg.decimation = 5
    go2_env_cfg.sim.render_interval = go2_env_cfg.decimation 
    go2_env_cfg.scene.num_envs = cfg.num_envs
    go2_ctrl.init_base_vel_cmd(cfg.num_envs)
    
    # Apply sensor transforms from config to environment
    go2_env_cfg.scene.front_cam.offset.pos = tuple(cfg.sensor.camera.pos)
    go2_env_cfg.scene.front_cam.offset.rot = tuple(cfg.sensor.camera.rot)
    go2_env_cfg.sim.use_fabric = True  
    go2_env_cfg.sim.device = "cpu"
    env, policy = go2_rl_env(go2_env_cfg, cfg)
    
    env_cfg = cfg.envs.get(cfg.env_name, None) if hasattr(cfg, "envs") else None
    turret_cfg = env_cfg.get("turret", None) if env_cfg is not None else None
    turret_enabled = bool(turret_cfg is not None and turret_cfg.get("enabled", False))

    global_turrets = None
    if turret_enabled:
        # 시뮬레이션 환경 생성(go2_rl_env) 직후 호출
        global_turret_cfg = ArticulationCfg(
            prim_path="/World/Turret_.*", # setup_global_turrets로 만든 모든 터렛 매칭
            spawn=None,
            actuators={
                "pan_tilt": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=1000.0, damping=100.0)
            }
        )
        global_turrets = Articulation(cfg=global_turret_cfg)
        global_turrets._device = env.unwrapped.device  # 1. RL 환경과 동일한 디바이스 강제 할당
        global_turrets._initialize_impl()              # 2. 물리 핸들(PhysX) 및 텐서 버퍼 초기화
        global_turrets._is_initialized = True          # 3. 초기화 완료 플래그 강제 설정 [1]
        global_turrets.reset()
    else:
        print(f"[INFO] Turret control is disabled for env '{cfg.env_name}'.")

    # Simulation environment
    if (cfg.env_name == "warehouse"):
        sim_env.create_warehouse_env() # warehouse
    elif (cfg.env_name == "lit_tower_4f"):
        sim_env.create_warehouse_lit_tower_4f_env() # warehouse forklifts
    elif (cfg.env_name == "full-warehouse"):
        sim_env.create_full_warehouse_env() # full warehouse
    elif (cfg.env_name == "stage"):
        sim_env.create_stage_env() # stage1
    elif (cfg.env_name == "warehouse_custom"):
        sim_env.create_warehouse_custom_env() # warehouse_custom
    elif (cfg.env_name == "konglabs"):
        sim_env.create_konglabs_custom_env() # warehouse_custom
    for _ in range(1):
        simulation_app.update()

    # Sensor setup
    sm = sensor_manager(cfg)
    # cameras, lidars
    lidars_3d = sm.create_lidar_3d()
    lidars_2d = sm.create_lidar_2d()
    cameras = env.unwrapped.scene["front_cam"]
    # ROS2 Bridge
    dm = go2_ros2_bridge.RobotDataManager(
        env,
        lidar_sensors_3d=lidars_3d,
        lidar_sensors_2d=lidars_2d,
        cameras=cameras,
        cfg=cfg,
        global_turrets=global_turrets,
    )

    # generate occupancy grid map
    if cfg.generate_map:
        generate_nav2_map(cfg)


    person_usd_path = "models/USD/simple_person.usd"
    character_root_path = "/World/Character"

    create_prim(
        prim_path=character_root_path,
        prim_type="Xform", 
        position=[3.0, 0.0, 0.0],
        orientation=[1.0, 0.0, 0.0, 0.0], 
        usd_path=person_usd_path 
    )

    print(f"[INFO]: Loaded character USD from {person_usd_path}")
    # 환경 오브젝트 생성 후 물리 엔진에 등록될 시간을 줍니다.
    for _ in range(1):
        simulation_app.update()
    # 문서에 명시된 파라미터대로 호출
    CHAR_PATH = "/World/Character/biped_demo_meters"
    character = ag.get_character(CHAR_PATH)

    print("[INFO]: simulation started")

    obs, _ = env.reset()

    #run simulation
    dt = float(go2_env_cfg.sim.dt * go2_env_cfg.decimation)
    
    while simulation_app.is_running():
        start_time = time.time()

        if global_turrets is not None and dm.turret_targets is not None:
            global_turrets.set_joint_position_target(dm.turret_targets)
            global_turrets.write_data_to_sim()

        with torch.inference_mode():
            actions = policy(obs)
            obs, _, _, _ = env.step(actions)

        if global_turrets is not None and dm.turret_targets is not None:
            global_turrets.update(dt=env.unwrapped.physics_dt)

        dm.update(character)

        elapsed_time = time.time() - start_time

        sleep_time = dt - (elapsed_time)
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        actual_loop_time = time.time() - start_time
        rtf = min(1.0, dt/elapsed_time)
        print(f"\rStep time: {actual_loop_time*1000:.2f}ms, Real Time Factor: {rtf:.2f}", end='', flush=True)
    dm.destroy_node()
    simulation_app.close()

if __name__ == "__main__":
    run_simulator()
    
