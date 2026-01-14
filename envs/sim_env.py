from isaacsim.core.utils.prims import define_prim, get_prim_at_path
try:
    import isaacsim.storage.native as nucleus_utils
except ModuleNotFoundError:
    import isaacsim.core.utils.nucleus as nucleus_utils

import omni.replicator.core as rep
from isaaclab.sim.schemas import CollisionPropertiesCfg, define_collision_properties
from isaaclab.sim.utils import apply_nested
import isaaclab.sim as sim_utils
import omni
from pxr import UsdGeom, Usd





def create_warehouse_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_forklifts_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_with_forklifts.usd"
    prim.GetReferences().AddReference(asset_path)

def create_warehouse_shelves_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/warehouse_multiple_shelves.usd"
    prim.GetReferences().AddReference(asset_path)

def create_full_warehouse_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Warehouse")
    prim = define_prim("/World/Warehouse", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Simple_Warehouse/full_warehouse.usd"
    prim.GetReferences().AddReference(asset_path)

def create_hospital_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Hospital")
    prim = define_prim("/World/Hospital", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Hospital/hospital.usd"
    prim.GetReferences().AddReference(asset_path)

def create_office_env():
    assets_root_path = nucleus_utils.get_assets_root_path()
    prim = get_prim_at_path("/World/Office")
    prim = define_prim("/World/Office", "Xform")
    asset_path = assets_root_path+"/Isaac/Environments/Office/office.usd"
    prim.GetReferences().AddReference(asset_path)


    