#!/usr/bin/env python
'''
    By Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Visit our website at www.theconstructsim.com
'''
import gym
import rospy
import numpy as np
import time
from gym import utils, spaces
from geometry_msgs.msg import Pose, Point
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection   # seems usable directly without modification
from joint_publisher import JointPub             # seems usable directly without modification
from mrm_state import MrmState           # needs to be re-written
from std_msgs.msg import Float64
from mrm_training_force.msg import Floatarray
#register the training environment in the gym as an available one
reg = register(
    id='Mrm-v0',
    entry_point='mrm_env_predicting:MrmEnv',
    timestep_limit=100,       # each time step corresponds to 0.01 s in simulation, 500 means if the arm cannot reach the target within 5 seconds, the task is
                             # thought as a failire.
    )

class MrmEnv(gym.Env):

    def __init__(self):
        # specify the dimension of observation space and action space
        self.observation_space = 44
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
        self.inr_r3 = rospy.get_param("/inr_r3")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.mrm_state_object = MrmState( weight_r1=self.weight_r1,
                                          weight_r2=self.weight_r2,
                                          weight_r3=self.weight_r3)

        self.mrm_state_object.set_desired_target_point(self.desired_target.x,
                                                          self.desired_target.y,
                                                          self.desired_target.z)

        self.mrm_joint_pubisher_object = JointPub()

        self._seed()

        self.passed_episode = 0
        self.total_episode = rospy.get_param("/total_episode")

        # list for recording the minimal value from the laser
        self.mini_laser = []

        self.obstacle_penalty = 0.0
    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def _reset(self):

        self.passed_episode += 1

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.logdebug("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_condition...")
        self.mrm_joint_pubisher_object.set_init_condition()

        # change the weights of rewards to enable curriculum learning
        self.weight_r3 **= self.inr_r3
        #self.weight_r3 = 1 - np.exp(-90 / (self.total_episode - self.passed_episode))

        self.mrm_state_object.change_weights(weight_r1=self.weight_r1,
                                             weight_r2=self.weight_r2,
                                             weight_r3=self.weight_r3)

        # Get the state of the Robot defined by the XY distances from the
        # desired point, jointState of the four joints and
        rospy.logdebug("check_all_systems_ready...")
        self.mrm_state_object.check_all_systems_ready()   # this step is very important!!!! the system will be easily failed if it is removed

        # 6th: We restore the gravity to original and unpause the sim
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        self.mrm_state_object.history = []   # clear the history
        observation = self.mrm_state_object.get_observations()
        observation.extend([0.0,0.0,0.0,0.0])  # assume the previous action is 0 for the first step

        return observation

    def _step(self, action):
        # We apply torques to each joint for a duration of time; in order to apply torques to each joint at the same time, we call the service while
        # gazebo is paused
        self.mrm_joint_pubisher_object.apply_joints_effort(action, self.running_step)
        # unpasue the simulation and start applying the effort
        self.gazebo.unpauseSim()
        rospy.sleep(self.running_step)
        # pause the simelation again to process data
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward, reach_target, done = self.mrm_state_object.process_data()

        # Generate State based on observations
        observation = self.mrm_state_object.get_observations()
        observation.extend(action)   # add previous action to the state

        # record the minimal value of laser data every time step
        laser_data = observation[8:18]
        self.mini_laser.append(min(laser_data))
        return observation, reward, done, reach_target, {}

    def set_target_position(self, target_pos):
        self.mrm_state_object.set_desired_target_point(target_pos[0], target_pos[1], target_pos[2])
