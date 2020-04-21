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
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection   # seems usable directly without modification
from joint_publisher import JointPub             # seems usable directly without modification
from mrm_state import MrmState           # needs to be re-written
from controllers_connection import ControllersConnection   # seems usable directly without modification
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
        self.observation_space = 22
        self.action_space = 4
        # We assume that a ROS node has already been created before initialising the environment

        self.running_step = rospy.get_param("/running_step")

        self.done_reward = rospy.get_param("/collision_reward")

        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.controllers_object = ControllersConnection(namespace="")

        self.mrm_state_object = MrmState( weight_r1=self.weight_r1,
                                          weight_r2=self.weight_r2,
                                          weight_r3=self.weight_r3)

        self.mrm_joint_pubisher_object = JointPub()

        self._seed()
        self.reset_controller_pub = rospy.Publisher('/reset_controller', Bool, queue_size=1)
        self.pause_sim_pub = rospy.Publisher('/pause_sim', Bool, queue_size=1)

    # A function to initialize the random generator
    def _seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    # Resets the state of the environment and returns an initial observation.
    def _reset(self):
        # 0st: We pause the controller
        self.pause_sim_pub.publish(Bool(True))

        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        # 1st: resets the simulation to initial values
        rospy.logdebug("Reset SIM...")
        self.gazebo.resetWorld()

        # 2nd: We Set the gravity to 0.0 so that we dont fall when reseting joints
        # It also UNPAUSES the simulation
        rospy.logdebug("Remove Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, 0.0)

        # close the controller
        # self.controllers_object.shut_down_controller()

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_condition...")
        self.mrm_joint_pubisher_object.set_init_condition()
        # 4th: probably add the function to change the target position is the target is reached

        # reset the controllers
        self.reset_controller_pub.publish(Bool(True))
        # self.controllers_object.reset_mrm_joint_controllers()

        # 6th: We restore the gravity to original
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        rospy.logdebug("check_all_systems_ready...")
        self.mrm_state_object.check_all_systems_ready()

        # 7th: pauses simulation
        rospy.logdebug("Pause SIM...")
        self.gazebo.pauseSim()

        rospy.logdebug("get_observations...")
        observation = self.mrm_state_object.get_observations()

        self.reset_controller_pub.publish(Bool(False))

        return observation

    def _step(self, joint_targets):
        # We move it to that pos
        self.gazebo.unpauseSim()
        self.pause_sim_pub.publish(Bool(False))

        self.mrm_joint_pubisher_object.move_joints(joint_targets)

        # Then we send the command to the robot and let it go for running_step seconds
        start = time.time()
        rospy.sleep(self.running_step)
        end = time.time()

        self.pause_sim_pub.publish(Bool(True))
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward, reach_target, done = self.mrm_state_object.process_data()

        # Generate State based on observations
        observation = self.mrm_state_object.get_observations()

        return observation, reward, done, reach_target, {}

    def set_target_position(self, target_pos):
        self.mrm_state_object.set_desired_target_point(target_pos[0], target_pos[1], target_pos[2])

    def change_joints_init(self, diameter):
        joints_inits = {80:[0.28, -0.78, -0.7, 0], 90:[0.3,-0.8, -0.75, 0], 100:[0.4, -1.05, -0.9, 0], 110:[0.51, -1.27,-0.95, 0], 120:[0.53, -1.29,-0.98, 0]}
        self.mrm_joint_pubisher_object.init_pos = joints_inits[diameter]

    def get_joints_init(self):
        return self.mrm_joint_pubisher_object.init_pos
