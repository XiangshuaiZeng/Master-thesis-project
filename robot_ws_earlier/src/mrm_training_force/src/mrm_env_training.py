#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import rospy
import numpy as np
import time, random
from geometry_msgs.msg import Point
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection   # seems usable directly without modification
from joint_publisher import JointPub             # seems usable directly without modification
from mrm_state import MrmState           # needs to be re-written
from std_msgs.msg import Float64
from mrm_training_force.msg import Floatarray
from load_model import model_control
from math import *
#register the training environment in the gym as an available one
reg = register(
    id='Mrm-v0',
    entry_point='mrm_env_training:MrmEnv',
    timestep_limit=100,       # each time step corresponds to 0.01 s in simulation, 500 means if the arm cannot reach the target within 5 seconds, the task is
                             # thought as a failire.
    )

class MrmEnv(gym.Env):

    def __init__(self):
        # specify the dimension of observation space and action space
        self.observation_space = 22   # joint states (8) + end_pos (2) + target_pos(2) + laer_data(10)
        self.action_space = 4         # joint efforts
        # We assume that a ROS node has already been created before initialising the environment

        # gets training parameters from param server
        self.desired_target = Point()
        self.desired_target.x = rospy.get_param("/desired_target_point/x")
        self.desired_target.y = rospy.get_param("/desired_target_point/y")
        self.desired_target.z = rospy.get_param("/desired_target_point/z")
        self.running_step = rospy.get_param("/running_step")


        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.mrm_state_object = MrmState( weight_r1=self.weight_r1,
                                          weight_r2=self.weight_r2,
                                          weight_r3=self.weight_r3)

        self.mrm_joint_pubisher_object = JointPub()

        self._seed()

        # list for recording the minimal value from the laser
        self.mini_laser = []

        self.moco = model_control()

        self.pipe_angle = 0.0

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def _reset(self):

        # It also UNPAUSES the simulation
        rospy.logdebug("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_condition...")
        self.mrm_joint_pubisher_object.set_init_condition()

        # Get the state of the Robot defined by the XY distances from the
        # desired point, jointState of the four joints and
        rospy.logdebug("check_all_systems_ready...")
        self.mrm_state_object.check_all_systems_ready()   # this step is very important!!!! the system will be easily failed if it is removed

        # 6th: We restore the gravity to original and unpause the sim
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # delete the old target
        self.moco.delete_model('target')
        # spawn the new target
        target_pos = self.change_target_pos(self.moco, self.pipe_angle)
        # set the target point for the gym env
        self.set_target_position([target_pos[0], target_pos[1], 0.0])

        # We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        observation = self.mrm_state_object.get_observations()

        avg_laser = 0.0
        # average the recorded minimal distance from the end-effector to the pipe
        if len(self.mini_laser) > 0:
            avg_laser = np.mean(self.mini_laser)
            # outF = open("/home/joshua/robot_ws_earlier/src/mrm_training_force/training_results/laser_compare/avg_laser.txt", "a")
            # outF.write('%s' % avg_laser)
            # outF.write("\n")
            # outF.close()
            self.mini_laser = []
        return observation, avg_laser

    def _step(self, action):
        # We apply torques to each joint for a duration of time; in order to apply torques to each joint at the same time, we call the service while
        # gazebo is paused
        self.mrm_joint_pubisher_object.apply_joints_effort(action, self.running_step)
        # unpasue the simulation and start applying the effort
        self.gazebo.unpauseSim()

        start = time.time()
        rospy.sleep(self.running_step)
        end = time.time()
        # pause the simelation again to process data
        self.gazebo.pauseSim()

        # Generate State based on observations
        observation = self.mrm_state_object.get_observations()
        # finally we get an evaluation based on what happened in the sim
        reward, reach_target, done = self.mrm_state_object.process_data()

        if end - start > 1.0:
            stuck = True
        else:
            stuck = False

        # record the minimal value of laser data every time step
        laser_data = observation[-1:-11:-1]
        self.mini_laser.append(min(laser_data))
        return observation, reward, done, reach_target, stuck, {}

    def set_target_position(self, target_pos):
        self.mrm_state_object.set_desired_target_point(target_pos[0], target_pos[1], target_pos[2])

    def change_target_pos(self, moco, angle):
        # how far will be the target away from the turning center approximately
        dis_target_left = [0.23, 0.24]
        dis_target = random.uniform(dis_target_left[0], dis_target_left[1])
        target_pos = [0.08 + dis_target*cos((angle-90.0)/180*pi), - 0.35 - dis_target*sin((angle-90.0)/180*pi)]
        # add some noise to vertical position
        target_pos[1] -= random.uniform(0.04, 0.07)

        target_name = 'target'    # the model name and namespace
        targetdir = '/home/joshua/robot_ws_earlier/src/mrm_training_force/urdf/target.xacro'
        # visulize the target in Gazebo
        moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], 0.0], [0, 0, 0])
        return target_pos
