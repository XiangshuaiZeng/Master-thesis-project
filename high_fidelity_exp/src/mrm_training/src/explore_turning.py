#!/usr/bin/env python

from joint_publisher import JointPub
import rospy
from math import *
import numpy as np
from std_msgs.msg import Float64, Float64MultiArray
from load_model import model_control
from gazebo_connection import GazeboConnection
import random
import gym
import mrm_env
from geometry_msgs.msg import Transform

def change_pipe(moco, diameter, angle):
    # info for spawning the pipe
    pipedir = '/home/joshua/high_fidelity_exp/src/mrm_training/urdf/pipe_d%d_%d.xacro' % (diameter, angle)
    pipe_name = 'pipe_d%d_%d' % (diameter, angle)

    pipe_pos_left = [0.065, -0.31, 0]
    pipe_orient_left = [0, 0, 3.1416]
    # visulize the pipe in Gazebo
    moco.spawn_model(pipedir, pipe_name, pipe_name, pipe_pos_left, pipe_orient_left)
    return pipe_name

def quaternion_mul_vector(q, v):
    # Extract the vector part of the quaternion
    u = np.asarray([q.x, q.y, q.z])
    v = np.asarray(v)
    # Extract the scalar part of the quaternion
    s = q.w
    # Do the math
    vprime = 2.0 * np.dot(u, v) * u + (s*s - np.dot(u, u)) * v + 2.0 * s * np.cross(u, v)

    return vprime

rospy.init_node('explore_turning')

moco = model_control()
# Create the Gym environment
env = gym.make('Mrm-v0')
env = env.unwrapped
rospy.loginfo ("Gym environment done")

mrm_jointpub = JointPub()

# start training from turning left
rospy.loginfo ("Start Validation")
success_time = 0.0
angles = range(55, 126, 5)

rate = rospy.Rate(10) # rate at 100 hz

global end_pos
def end_effector_pos_callback(msg):
    global end_pos
    end_pos = msg

# Get the position of end-effector
rospy.Subscriber("/end_effector_pos", Transform, end_effector_pos_callback)

while not rospy.is_shutdown():
    # choose a pipe randomly
    angle = random.choice(angles)
    d = 110
    ob = env.reset()        # reset env and get the initial state
    # spawn the pipe
    pipe_name = change_pipe(moco, d, angle)

    delta_third = 0.15
    delta_trans = 0.02

    while True:
        joint_targets = ob[0:4] + np.asarray([0.0, 0.0, delta_third, delta_trans])
        res = env.step(joint_targets, joint_targets)      # take the action
        ob = res[0]

        if min(ob[-1:-11:-1]) < 0.06:
            delta_trans /= 8.0

        if max(ob[-1:-11:-1]) == 0.5:

            target_dis = 0.2 + random.uniform(-0.02, 0.05)
            target_pos = [target_dis* sin(45.0/180*pi), -target_dis * cos(45.0/180*pi), 0]
            target_pos = quaternion_mul_vector(end_pos.rotation, target_pos)
            target_pos = target_pos + np.asarray([end_pos.translation.x, end_pos.translation.y, end_pos.translation.z])

            # spawn the target
            target_name = 'target'    # the model name and namespace
            targetdir = '/home/joshua/high_fidelity_exp/src/mrm_training/urdf/target.xacro'

            # visulize the target in Gazebo
            moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], target_pos[2]], [0, 0, 0])
            break

        rate.sleep()

    moco.delete_model(pipe_name)
    moco.delete_model('target')
