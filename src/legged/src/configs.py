import numpy as np
from enum import Enum
from raisimpy import ControlMode, World


class ACTION_TYPE(Enum):
    PDCONTROL = 1
    FORCE = 2
    VELOCITY = 3
    DISCRETEVEL = 4


class EnvConfig:
    def __init__(self, enable_render=True, world_file="", robot_file="", name="r1"):
        self.module_name = name
        self.world_file = world_file
        self.robot_file = robot_file
        self.joint_order = []

        self.enable_render = enable_render
        self.record_video: bool = False

        self.control_time_step = 0.1
        self.simulation_time_step = 0.02

        self.control_mode = ControlMode.PD_PLUS_FEEDFORWARD_TORQUE  # FORCE_AND_TORQUE, or PD_PLUS_FEEDFORWARD_TORQUE

        self.action_type = None
        self.p_gain = None
        self.d_gain = None
        self.init_pose = None
        self.init_vel = None
        self.action_space = None
        self.observation_space = None
        self.damping = None


class WheeledConfig(EnvConfig):
    def __init__(self, enable_render=True, world_file="", robot_file="", name="r1"):
        super(WheeledConfig, self).__init__(enable_render=enable_render, world_file=world_file,
                                            robot_file=robot_file, name=name)
        self.action_type: ACTION_TYPE = ACTION_TYPE.VELOCITY  # 0: D control; 1: PD control; 2: force control
        self.init_pose = np.array([0., 0., 0.71, 1., 0., 0., 0., 0., 0., 0., 0.], dtype=float)
        self.init_vel = np.zeros(10, dtype=float)
        # self.p_gain = np.concatenate((np.zeros(6),np.ones(4)*1.))
        self.p_gain = np.zeros(10)
        self.d_gain = np.concatenate((np.zeros(6), np.ones(4) * 100.))

        self.damping = np.concatenate((np.zeros(6),np.ones(4)))

        # husky: wheel distance: w:0.5708  l:0.512 r:0.17775
        # wheeled spot: w:0.11 r:0.17775
        self.wheel_dis_w = 0.11  # husky: 1.01
        self.wheel_radius = 0.1775



class LeggedConfig(EnvConfig):
    def __init__(self, enable_render=True, world_file="", robot_file="", name="r1"):
        super(LeggedConfig, self).__init__(enable_render=enable_render, world_file=world_file,
                                           robot_file=robot_file, name=name)
        self.action_type: ACTION_TYPE = ACTION_TYPE.PDCONTROL  # 0: D control; 1: PD control; 2: force control
        self.init_pose = np.array([0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.], dtype=float)
        self.init_vel = np.zeros(12, dtype=float)
        self.p_gain = np.ones(10) * 1.
        self.d_gain = np.ones(10) * 0.2

        self.damping = np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1])


class LidarConfig:
    def __init__(self, hfv=360., vfv=30, channels=16, distance=100):
        self.hfv = hfv
        self.vfv = vfv
        self.channels = channels
        self.distance = distance

    def get_ray(self, position, world: World):
        pass