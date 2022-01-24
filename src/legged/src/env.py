from abc import ABC
import os
import numpy as np
from copy import deepcopy as dcp
from scipy.spatial.transform.rotation import Rotation
import gym
from gym.vector.vector_env import VectorEnv
from gym.spaces import Space, Box, Discrete, MultiDiscrete, MultiBinary, Tuple, Dict

import raisimpy as raisim
from raisimpy import ControlMode, ArticulatedSystem

from configs import EnvConfig, WheeledConfig, LeggedConfig, ACTION_TYPE


class Environment(gym.Env):
    def __init__(self, config: EnvConfig):
        self.config: EnvConfig = config
        self.action_space = None
        self.observation_space = None
        self.enable_render = self.config.enable_render

        if self.config.world_file:
            self.world = raisim.World(self.config.world_file)
        else:
            self.world = raisim.World()
            self.world.addGround()
        # self.world.setLicenseFile(licenseFile="/home/jing/.raisim/activation.raisim")
        self.world.setTimeStep(self.config.simulation_time_step)

        self.robot: ArticulatedSystem = self.world.addArticulatedSystem(urdf_path=self.config.robot_file,
                                                                        joint_order=self.config.joint_order)
        self.robot.setControlMode(self.config.control_mode)
        self.robot.setJointDamping(self.config.damping)

        self.vel_dim = self.robot.getDOF()
        self.pos_dim = self.robot.getGeneralizedCoordinateDim()
        self.joint_dim = self.vel_dim - 6

        """
        A feedforward force term can be added by setGeneralizedForce() if desired. This term is set to zero by default. 
        Note that this value is stored in the class instance and does not change unless the user specifies it so. 
        If this feedforward force should be applied for a single time step, 
        it should be set to zero in the subsequent control loop (after integrate() call of the world).
        """
        self.robot.setGeneralizedForce(np.zeros(self.vel_dim))

        self.robot.setPdGains(p_gains=self.config.p_gain, d_gains=self.config.d_gain)

        if self.enable_render:
            self.server = raisim.RaisimServer(self.world)
            self.server.launchServer(8080)  # port
            self.server.focusOn(self.robot)

        self.robot.printOutBodyNamesInOrder()

    def step(self, action):
        if self.config.action_type == ACTION_TYPE.PDCONTROL:
            assert len(action[1]) == self.joint_dim and len(action[0]) == self.joint_dim, \
                "action[0] or action[1] is different from the DOF {}".format(self.joint_dim)
            pose_target = np.concatenate((np.zeros(7), np.array(action[0])))
            velocity_target = np.concatenate((np.zeros(6), np.array(action[1])))
            self.robot.setPdTarget(pos_targets=pose_target, vel_targets=velocity_target)
        elif self.config.action_type == ACTION_TYPE.FORCE:
            assert len(action) == self.joint_dim, "action is different from the DOF {}".format(self.joint_dim)
            self.robot.setGeneralizedForce(np.concatenate((np.zeros(6), np.array(action))))
        elif self.config.action_type == ACTION_TYPE.VELOCITY:
            assert len(action) == 2, "the actions should only has linear and angular velocities"
            vl = (2 * action[0] - self.config.wheel_dis_w * action[1]) / (2 * self.config.wheel_radius)
            vr = (2 * action[0] + self.config.wheel_dis_w * action[1]) / (2 * self.config.wheel_radius)
            pose_target = np.zeros(self.pos_dim)
            velocity_target = np.concatenate((np.zeros(6), np.array([vl, vr, vl, vr])))
            self.robot.setPdTarget(pos_targets=pose_target, vel_targets=velocity_target)
        else:
            assert False, "the action type is undefined!"

    def get_observation(self):
        self._get_status_observation()

        poses, vels = self.robot.getState()
        return poses, vels

    def _get_status_observation(self):
        # calculate collision of objects and bodies in each simulation step
        for i in range(int(self.config.control_time_step / self.config.simulation_time_step + 1e-10)):
            if self.enable_render:
                self.server.integrateWorldThreadSafe()
            else:
                self.world.integrate()

    def compute_reward(self):
        pass

    def reset(self):
        # self.robot.setGeneralizedCoordinate(self.config.init_pose)
        # self.robot.setGeneralizedVelocity(self.config.init_vel)
        self.robot.setState(self.config.init_pose, self.config.init_vel)

    def render(self, mode=None):
        pass

    def seed(self, seed=None):
        pass

    def close(self):
        if self.enable_render:
            try:
                self.server.killServer()
            except:
                print("server is already deconstructed")


class WheeledEnv(Environment):
    def __init__(self, config: WheeledConfig):
        super(WheeledEnv, self).__init__(config)

    def sim(self):
        last_pose = np.zeros(11)
        for i in range(10000000000):
            if self.config.action_type == ACTION_TYPE.PDCONTROL:
                Tvelocity = np.array([0.3, 0.3, 0.3, 0.3])
                Tposition = np.zeros(4)
                # Tposition = last_pose[7:] + Tvelocity*self.config.simulation_time_step*2
                total = np.array([Tposition, Tvelocity])
                self.step(total)
                gc, vc = self.get_observation()
                diff_p = (gc[0:3] - last_pose[0:3]) / (self.config.simulation_time_step * int(
                    self.config.control_time_step / self.config.simulation_time_step + 1e-10))
                last_pose = dcp(gc)
                print("d:{}; v:{}, w:{}/{}".format(diff_p[0], vc[0], vc[6:] * 0.17775, vc[6:]))
            elif self.config.action_type == ACTION_TYPE.VELOCITY:
                self.step([0.5, 0.9])
                gc, vc = self.get_observation()
                diff_p = (gc[0:3] - last_pose[0:3]) / (self.config.simulation_time_step * int(self.config.control_time_step / self.config.simulation_time_step + 1e-10))
                last_pose = dcp(gc)
                d = (vc[7] + vc[9] - vc[8] - vc[6]) * self.config.wheel_radius / (2 * vc[5])
                print("p:{}; v:{}; a:{}; d:{}; w:{}".format(np.linalg.norm(diff_p[0:3]), np.linalg.norm(vc[0:3]), vc[5],
                                                            d, vc[6:] * self.config.wheel_radius))
            if abs(gc[0]) > 35. or abs(gc[1]) > 1000.:
                self.robot.setGeneralizedCoordinate(np.array([0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0]))


class LeggedEnv(Environment):
    def __init__(self, config: LeggedConfig):
        super(LeggedEnv, self).__init__(config)
        self.robot.setJointDamping(np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1]))

    def step_sim(self):
        while True:
            pose_target = np.array([0., 0., 0., 0., 0., 0., 0., 10., 10., 10., 10.])
            velocity_target = np.array([0., 0., 0., 0., 0., 0., 10., 10., 10., 10.])
            self.robot.setPdTarget(pose_target, velocity_target)
            for i in range(int(self.config.control_time_step / self.config.simulation_time_step + 1e-10)):
                self.world.integrate()


# class VecEnv(VectorEnv):
#     def __init__(self, num_envs, observation_space, action_space):
#         super(VecEnv, self).__init__(num_envs, observation_space, action_space)

"""
class InfoEnv(Environment):
    def __init__(self, config: EnvConfig):
        super(InfoEnv, self).__init__(config)

    def get_frame_body_names(self):
        print("--------------- body names: ----------------------")
        # joints also has the same order
        # root corresponding to the base of the robot: getBaseOrientation...
        self.robot.printOutBodyNamesInOrder()  #
        print("--------------- frame names: ----------------------")
        self.robot.printOutFrameNamesInOrder()

        # get total states:
        poses, vels = self.robot.getState()
        vel = self.robot.getGeneralizedVelocity()
        dof = self.robot.getDOF()
        Coordinate = self.robot.getGeneralizedCoordinate()
        cord = self.robot.getGeneralizedCoordinateDim()

        # get base information: can also get position information from pybind11
        orib = self.robot.getBaseOrientation()

        # there could also be orientation in cpp files with pybind11
        joint_pose = self.robot.getJointPos_P

        # get frames info:
        for i in self.robot.getBodyNames():
            id = self.robot.getBodyIdx(i)
            # relative
            pos = self.robot.getBodyPosition(id).flatten()
            ori = self.robot.getBodyOrientation(id).flatten()
            # absolute
            pos2 = self.robot.getPosition(id).flatten()
            ori2 = self.robot.getOrientation(id).flatten()
            avel = self.robot.getAngularVelocity(id).flatten()
            vel = self.robot.getVelocity(id).flatten()
            # print("body: {}/{}:  {}/{} || {}/{} || {}/{}".format(id,i, pos, pos2, ori, ori2, avel, vel))

        for frame in self.robot.getFrames():
            id = self.robot.getFrameIdxByName(frame.name)
            pos = self.robot.getFramePosition(id).flatten()
            ori = self.robot.getFrameOrientation(id).flatten()
            avel = self.robot.getFrameAngularVelocity(id).flatten()
            vel = self.robot.getFrameVelocity(id).flatten()
            ## frame.position.flatten(), frame.orientation.flatten() will never change

        mass_b = self.robot.getBodyCOM_B()  # w.r.t. each base body coordinate
        mass = self.robot.getMass()
        mass_com = self.robot.getCompositeMass()  # the first one is the full body, and the others are branches links
        maxx = self.robot.getCompositeCOM()  # the first one is the full body, and the others are branches links
        # print("{}/{}/{}/{}".format(maxx, mass_b,mass,mass_com))

        potential_energy = self.robot.getPotentialEnergy(gravity=[0, 0, -1])
        kinetic_energy = self.robot.get_kinetic_energy()
        energy = self.robot.getEnergy(gravity=[0, 0, -9.8])
        # print("{} = {} + {}".format(energy, kinetic_energy, potential_energy))

        gf = self.robot.getGeneralizedForce()
        ff_gf = self.robot.getFeedForwardGeneralizedForce()
        # print("force: {}/{}".format(gf, ff_gf))

        # up = []
        # low = []
        # self.robot.setActuationLimits(upper=up, lower=low)

        return self.robot.getBodyNames(), self.robot.getFrames()

    def sim(self):
        for i in range(10000000000):
            self.server.integrateWorldThreadSafe()
            self.robot.setGeneralizedForce(np.array([0, 0, 0, 0, 0, 0, 50, 50, 50, 50]))
            gc = self.robot.getGeneralizedCoordinate()
            if abs(gc[0]) > 35. or abs(gc[1]) > 35.:
                self.robot.setGeneralizedCoordinate(np.array([0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0]))
            body_names, frame_names = self.get_frame_body_names()

    def step_sim(self):
        while True:
            pose_target = np.array([0., 0., 0., 0., 0., 0., 0., 10., 10., 10., 10.])
            velocity_target = np.array([0., 0., 0., 0., 0., 0., 10., 10., 10., 10.])
            self.robot.setPdTarget(pose_target, velocity_target)
            for i in range(int(self.config.control_time_step / self.config.simulation_time_step + 1e-10)):
                self.world.integrate()
"""

if __name__ == "__main__":
    # wheeled = WheeledEnv(WheeledConfig(world_file="/home/jing/Documents/spot/src/test/pyt/worlds/heightMapUsingPng.xml",
    #                                    robot_file="/home/jing/Documents/spot/src/test/pyt/models/husky/husky.urdf"))
    wheeled = WheeledEnv(WheeledConfig(robot_file="/home/jing/Documents/spot/src/legged/models/wheeledspot/spot_raisim_wheel.urdf"))
    wheeled.reset()
    wheeled.sim()
    print("test")
