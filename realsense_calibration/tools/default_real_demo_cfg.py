from yacs.config import CfgNode as CN

_C = CN()
_C.TABLE_SIZE = [1.212, 1.212, 0.01]
_C.TABLE_POSE = [0.2, 0, 0, 0, 0, 0, 1]
_C.RACK_POSE = [0.45, 0.5, 0, 0, 0, 0.707107, 0.707107] 
_C.HOME_JOINTS = [-0.7058, -0.1157, 0.8146, -2.446, 0.1245, 2.357, -0.725]
_C.OUT_OF_FRAME_JOINTS = [-0.7058, -0.1157, 0.8146, -1.558, 0.1245, 2.357, -0.725]
_C.DEFAULT_EE_FRAME = 'panda_link8'

_C.GRASP_CLOSE_WIDTH = 0.001
_C.GRASP_CLOSE_FORCE = 45

_C.DEFAULT_RACK_PLACE_OFFSET = [0, -0.042, 0.016]  # for wooden amazon rack 
#_C.DEFAULT_RACK_PLACE_OFFSET = [0, -0.042, 0.05]  # for hacky 8020 rack
_C.DEFAULT_SHELF_PLACE_OFFSET = [0, 0, 0.075]
_C.DEFAULT_PLACE_OFFSET = [0, 0, 0.075]

_C.ARM_EE_LINK_TO_GRASP_TARGET = [0, 0, 0.105, 0.000, 0.000, -0.383, 0.924]
_C.GRASP_TARGET_TO_ARM_EE_LINK = [-0.0, 0.0, -0.105, 0.0, -0.0, 0.3829, 0.9237]

_C.WAYPOINT_POSE = [
 0.3022092329626218,
 0.06527618932815576,
 0.5503250413547468,
 -0.9226211386017904,
 -0.3852537274796118,
 -0.008367826726283768,
 -0.01672661185296748
]
_C.DEFAULT_PLAYBACK_LOOP_TIME = 0.01

_C.WRIST2TIP = CN()
_C.TIP2WRIST = CN()
_C.WRIST2TIP.PANDA_HAND = [0.0, 0.0, 0.1034, 0.0, 0.0, -0.3826834323650898, 0.9238795325112867] 
_C.WRIST2TIP.ROBOTIQ_2F140 = [0.0, 0.0, 0.244, 0.0, 0.0, 0.0, 1.0]
_C.TIP2WRIST.PANDA_HAND = [0.0, 0.0, -0.1034, 0.0, 0.0, 0.3826834323650898, 0.9238795325112867] 
_C.TIP2WRIST.ROBOTIQ_2F140 = [0.0, 0.0, -0.244, 0.0, 0.0, 0.0, 1.0]


_C.CAM = CN()
_C.CAM.WORKSPACE_LIMITS_12_X = [425, 575]
_C.CAM.WORKSPACE_LIMITS_12_Y = [-50, 50]
_C.CAM.WORKSPACE_LIMITS_12_Z = [300, 375]

_C.CAM.WORKSPACE_LIMITS_34_X = [425, 575]
_C.CAM.WORKSPACE_LIMITS_34_Y = [-50, 50]
_C.CAM.WORKSPACE_LIMITS_34_Z = [300, 375]

# dummy placement objects from simulation (shelf and rack)
_C.PLACEMENT_OBJECTS = CN()
_C.PLACEMENT_OBJECTS.RACK_POSE_TABLE_FRAME = [0.3, -0.1, 1.0, 0.0, 0.0, 1.0, 0.0]
_C.PLACEMENT_OBJECTS.RACK_MESH_FILE = 'hanging/table/simple_rack.obj'
_C.PLACEMENT_OBJECTS.RACK_SCALE = [1.0, 1.0, 1.0]
_C.PLACEMENT_OBJECTS.SHELF_POSE_TABLE_FRAME = [-0.45, 0.1125, 1.0, 0.0, 0.0, -0.7071, 0.7071]
_C.PLACEMENT_OBJECTS.SHELF_MESH_FILE = 'hanging/table/shelf_back.obj'
_C.PLACEMENT_OBJECTS.SHELF_SCALE = [0.75, 0.75, 0.5]


def get_real_demo_cfg_defaults():
    return _C.clone()