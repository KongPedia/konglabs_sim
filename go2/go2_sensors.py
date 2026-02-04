from isaaclab.sensors import Camera, CameraCfg
import omni
import numpy as np
import isaaclab.sim as sim_utils
from pxr import Gf
import omni.replicator.core as rep


class sensor_manager:

    def __init__(self, cfg):
        self.cfg = cfg
        self.num_envs = cfg.num_envs



    def create_lidar_3d(self):
        lidar_sensors_3d = []

        sensor_attributes = {
            'omni:sensor:Core:scanRateBaseHz': 10,
            'omni:sensor:Core:farRangeM': 10.0, 
            'omni:sensor:Core:patternFiringRateHz': 360,
            'omni:sensor:Core:validStartAzimuthDeg': 270.0,
            'omni:sensor:Core:validEndAzimuthDeg': 90.0,
            'omni:sensor:Core:tiledSubSampling': 8,
        
        }

        for env_idx in range(self.num_envs):
            parent_path = f"/World/envs/env_{env_idx}/Go2/base"

            # Create the RTX Lidar with the specified attributes.
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                translation=Gf.Vec3d(*self.cfg.sensor.lidar.pos),
                orientation=Gf.Quatd(*self.cfg.sensor.lidar.rot),
                path="lidar_3d",
                parent=parent_path,
                config="HESAI_XT32_SD10",
                **sensor_attributes,
            )
        

            lidar_sensors_3d.append(sensor)

        return lidar_sensors_3d
    
    def create_lidar_2d(self):
        lidar_sensors_2d = []

        sensor_attributes = {'omni:sensor:Core:scanRateBaseHz': 10}

        for env_idx in range(self.num_envs):
            parent_path = f"/World/envs/env_{env_idx}/Go2/base"

            # Create the RTX Lidar with the specified attributes.
            _, sensor = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar",
                translation=Gf.Vec3d(*self.cfg.sensor.lidar.pos),
                orientation=Gf.Quatd(*self.cfg.sensor.lidar.rot),
                path="lidar_2d",
                parent=parent_path,
                config="Example_Rotary_2D",
                **sensor_attributes,
            )
        

            lidar_sensors_2d.append(sensor)

        return lidar_sensors_2d