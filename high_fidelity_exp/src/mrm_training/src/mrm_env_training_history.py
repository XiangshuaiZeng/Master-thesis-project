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
from std_msgs.msg import Bool
from gym.utils import seeding
from gym.envs.registration import register
from gazebo_connection import GazeboConnection   # seems usable directly without modification
from joint_publisher import JointPub             # seems usable directly without modification
from mrm_state_history import MrmState           # needs to be re-written
from std_msgs.msg import Float64
from mrm_training_force.msg import Floatarray
from load_model import model_control
from math import *
#register the training environment in the gym as an available one
reg = register(
    id='Mrm-v0',
    entry_point='mrm_env_training_history:MrmEnv',
    timestep_limit=100,       # each time step corresponds to 0.01 s in simulation, 500 means if the arm cannot reach the target within 5 seconds, the task is
                             # thought as a failire.
    )

class MrmEnv(gym.Env):

    def __init__(self):
        # specify the dimension of observation space and action space
        self.observation_space = 44
        self.action_space = 4         # joint efforts
        # We assume that a ROS node has already been created before initialising the environment

        self.running_step = rospy.get_param("/running_step")


        self.weight_r1 = rospy.get_param("/weight_r1")
        self.weight_r2 = rospy.get_param("/weight_r2")
        self.weight_r3 = rospy.get_param("/weight_r3")
        # self.inr_r3 = rospy.get_param("/inr_r3")

        # stablishes connection with simulator
        self.gazebo = GazeboConnection()

        self.mrm_state_object = MrmState( weight_r1=self.weight_r1,
                                          weight_r2=self.weight_r2,
                                          weight_r3=self.weight_r3)

        self.mrm_joint_pubisher_object = JointPub()

        self._seed()

        self.reset_controller_pub = rospy.Publisher('/reset_controller', Bool, queue_size=1)
        self.pause_sim_pub = rospy.Publisher('/pause_sim', Bool, queue_size=1)

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

        # 3rd: resets the robot to initial conditions
        rospy.logdebug("set_init_condition...")
        self.mrm_joint_pubisher_object.set_init_condition()

         # reset the controllers
        self.reset_controller_pub.publish(Bool(True))

        # 6th: We restore the gravity to original and unpause the sim
        rospy.logdebug("Restore Gravity...")
        self.gazebo.change_gravity(0.0, 0.0, -9.81)

        # Get the state of the Robot defined by the XY distances from the
        # desired point, jointState of the four joints and
        rospy.logdebug("check_all_systems_ready...")
        self.mrm_state_object.check_all_systems_ready()   # this step is very important!!!! the system will be easily failed if it is removed

        # delete the old target
        self.moco.delete_model('target')
        # spawn the new target
        target_pos = self.change_target_pos(self.moco, self.pipe_angle)
        # set the target point for the gym env
        self.set_target_position([target_pos[0], target_pos[1], 0.0])

        # We pause the Simulator
        rospy.logdebug("Pausing SIM...")
        self.gazebo.pauseSim()

        self.mrm_state_object.history = []   # clear the history
        observation = self.mrm_state_object.get_observations()
        observation.extend([0.0,0.0,0.0,0.0])  # assume the previous action is 0 for the first step

        self.reset_controller_pub.publish(Bool(False))

        return observation

    def _step(self, action):
        # We move it to that pos
        self.gazebo.unpauseSim()
        self.pause_sim_pub.publish(Bool(False))

        self.mrm_joint_pubisher_object.move_joints(action)

        start = time.time()
        rospy.sleep(self.running_step)
        end = time.time()

        self.pause_sim_pub.publish(Bool(True))
        self.gazebo.pauseSim()

        # finally we get an evaluation based on what happened in the sim
        reward, reach_target, done = self.mrm_state_object.process_data()

        # Generate State based on observations
        observation = self.mrm_state_object.get_observations()
        observation.extend(action)   # add previous action to the state

        # record the minimal value of laser data every time step
        # laser_data = observation[8:18]
        # self.mini_laser.append(min(laser_data))

        if end - start > 1.6:
            stuck = True
        else:
            stuck = False

        return observation, reward, done, reach_target, stuck, {}

    def set_target_position(self, target_pos):
        self.mrm_state_object.set_desired_target_point(target_pos[0], target_pos[1], target_pos[2])

    def change_target_pos(self, moco, angle):
        # how far will be the target away from the turning center approximately
        dis_target_left = [0.22, 0.23]

        b = 0.07 + random.uniform(0.0, 0.018)

        dis_target = np.asarray(random.uniform(dis_target_left[0], dis_target_left[1]))

        dis_target += (-0.018 + (angle - 60) / 10 * 0.006)

        target_pos = [0.07 + dis_target*cos((angle-90.0)/180*pi)-b*sin((angle-90.0)/180*pi), - 0.31 - dis_target*sin((angle-90.0)/180*pi)-b*cos((angle-90.0)/180*pi)]
        # add some noise to vertical position
        # target_pos[1] -= random.uniform(0.04, 0.07)

        target_name = 'target'    # the model name and namespace
        targetdir = '/home/joshua/robot_arm_ws/src/mrm_training_force/urdf/target.xacro'
        # visulize the target in Gazebo
        moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], 0.0], [0, 0, 0])
        return target_pos
