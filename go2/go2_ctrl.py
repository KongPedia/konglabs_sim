import os
import torch
import carb
from isaaclab.envs import ManagerBasedEnv
from rsl_rl.runners import OnPolicyRunner
from isaaclab.devices import Se2Keyboard, Se2KeyboardCfg


base_vel_cmd_input = None
keyboardCfg = Se2KeyboardCfg()
keyboardCfg.v_x_sensitivity = 1.2      # X축 선속도 민감도 변경
keyboardCfg.v_y_sensitivity = 1.2      # Y축 선속도 민감도 변경
keyboardCfg.omega_z_sensitivity = 1.5  # Z축 각속도(회전) 민감도 변경
keyboard = Se2Keyboard(keyboardCfg)


# Initialize base_vel_cmd_input as a tensor when created
def init_base_vel_cmd(num_envs):
    global base_vel_cmd_input
    base_vel_cmd_input = torch.zeros((num_envs, 3), dtype=torch.float32)


# Modify base_vel_cmd to use the tensor directly
def base_vel_cmd(env: ManagerBasedEnv) -> torch.Tensor:
    global base_vel_cmd_input

        
    return base_vel_cmd_input.clone().to(env.device)

def get_keyboard_cmd():
    global keyboard
    if keyboard is not None:
        return keyboard.advance()
    return [0.0, 0.0, 0.0]
