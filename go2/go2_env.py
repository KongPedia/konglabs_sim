from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.sensors.ray_caster import RayCasterCfg
from isaaclab.sensors.ray_caster.patterns import GridPatternCfg
from isaaclab.sensors.ray_caster import patterns
from isaaclab.utils import configclass
from isaaclab_assets.robots.unitree import UNITREE_GO2_CFG  # isort:skip
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
import isaaclab.sim as sim_utils

@configclass
class Myscene(InteractiveSceneCfg):
    # 지형 정의
    terrain = TerrainImporterCfg(
        prim_path = "/World/ground",
        terrain_type = "plane",
    )

    # 로봇 정의
    go2: ArticulationCfg = UNITREE_GO2_CFG.replace(prim_path="{ENV_REGEX_NS}/Go2")

    # 센서 정의
    height_scanner = RayCasterCfg(
        prim_path = "{ENV_REGEX_NS}/Go2/base",
        update_period = 0.02,
        offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
        ray_alignment="yaw",
        pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]), # pattern_cfg
        debug_vis=True,
        mesh_prim_paths=["/World/ground"],
    )

    # 조명 정의
    light = AssetBaseCfg(
        prim_path = "/World/light",
        spawn = sim_utils.DistantLightCfg(intensity=1000.0),
    )
