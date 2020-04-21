#!/usr/bin/env python

import gym
import time
import numpy as np
import random
from math import *
# from My_TRPO.model import TRPO
# from My_TRPO.utils import rollout
from My_PPO.model import PPO
from My_PPO.utils import rollout
from gym import wrappers
from std_msgs.msg import Float64
# ROS packages required
import rospy
import rospkg
from load_model import model_control
# import our predicting environment
import mrm_env_predicting

def change_pipe(moco, diameter, angle):
    # info for spawning the pipe
    pipedir = '/home/joshua/robot_ws_earlier/src/mrm_training_force/urdf/pipe_template.xacro'

    # change the xacro file
    with open(pipedir, 'r') as file:
        filedata = file.read()
    # Replace the target string
    pipe_dim = 'd%d_%d' % (diameter, angle)
    filedata = filedata.replace('di_pipe', pipe_dim)
    # Write the file out again
    with open(pipedir, 'w') as file:
        file.write(filedata)

    pipe_name = 'pipe_d%d_%d' % (diameter, angle)

    offset = (120 - diameter) / 10.0 * 0.001
    pipe_pos_left = [0.077 - offset, -0.31, 0]
    pipe_orient_left = [0, 0, 3.1416]
    # visulize the pipe in Gazebo
    moco.spawn_model(pipedir, pipe_name, pipe_name, pipe_pos_left, pipe_orient_left)

     # change the xacro file back to template
    with open(pipedir, 'r') as file:
        filedata = file.read()
    # Replace the target string
    filedata = filedata.replace(pipe_dim, 'di_pipe')
    # Write the file out again
    with open(pipedir, 'w') as file:
        file.write(filedata)

    return pipe_name

def change_target_pos(moco, diameter, angle):
    # how far will be the target away from the turning center approximately
    dis_target_left = [0.23, 0.24]
    dis_target = random.uniform(dis_target_left[0], dis_target_left[1])

    offset = (120 - diameter) / 10.0 * 0.001
    target_pos = [0.08 + dis_target*cos((angle-90.0)/180*pi) - offset, - 0.35 - dis_target*sin((angle-90.0)/180*pi)]
    # add some noise to vertical position
    target_pos[1] -= random.uniform(0.04, 0.07)

    target_name = 'target'    # the model name and namespace
    targetdir = '/home/joshua/robot_ws_earlier/src/mrm_training_force/urdf/target.xacro'
    # visulize the target in Gazebo
    moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], 0.0], [0, 0, 0])
    return target_pos

if __name__ == '__main__':

    rospy.init_node('mrm_gym_predict', anonymous=True, log_level=rospy.INFO, disable_signals=True)

    args = {
    # these parameters should stay the same
    "task" : "Mrm-v0",
    "n_steps" : 2400,
    "gamma" : 0.98,
    "lamda" : 0.975,   # Best lambda value is lower than gamma, empirically lambda introduces far less bias than gamma for a reasonably accruate value function'
    "vf_constraint" : 0.01,
    "max_kl" : .01,
    "coef_en": 0.01,  # coefficient for entropy bonus, only used in PPO
    "lr_ph": 0.0006, # learning rate for adam, only used in PPO
    "cg_damping" : 0.001,
    "max_pathlength" : 80,
    "n_validation": 120
    }

    moco = model_control()
    # Create the Gym environment
    env = gym.make('Mrm-v0')
    env = env.unwrapped
    rospy.loginfo ("Gym environment done")

    # Initialises the algorithm that we are going to use for predicting
    agent = PPO(args=args, observation_size=env.observation_space,
                    action_size=env.action_space)

    # load the trained model
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mrm_training_force')
    outdir = pkg_path + '/training_results'
    string_coffs = ['00']

    # pipe coefficient
    angles = range(55, 61, 5)
    D = [100]

    rospy.loginfo ("Start Validation")
    for string_coff in string_coffs:
        modeldir = outdir + '/PPO_model_new1/torque_laser_%s.ckpt' % string_coff
        agent.load_model(modeldir)

        # start training from turning left
        for d in D:
            # for recording the average distance between the end-effector and the target every episode
            success_times = [[] for _ in range(len(angles))]
            for episode in range(args['n_validation']):
                # choose a pipe randomly
                angle = angles[episode % len(angles)]
                ob = env.reset()        # reset env and get the initial state
                # spawn the pipe
                pipe_name = change_pipe(moco, d, angle)
                # spawn the target
                target_pos = change_target_pos(moco, d, angle)
                # set the target point for the gym env
                env.set_target_position([target_pos[0], target_pos[1], 0.0])
                # for checking the joint positions
                stuck = False
                joint_pos = []
                joint_delta_min = 0.2
                N = 3

                # start the validation
                rewards = []
                for time_step in xrange(args['max_pathlength']):
                    #check whether the robotic arm stops moving
                    if time_step % N == 0:
                        joint_pos.append(np.asarray(ob[0:3]))
                        if len(joint_pos) > 1:
                            joint_delta = np.linalg.norm(joint_pos[-1] - joint_pos[-2])
                            # print joint_delta
                            stuck = joint_delta < joint_delta_min

                    if stuck or not stuck:
                        # get action from the policy network
                        action, action_dist_mu, _ = agent.act(ob)
                        # action = action_dist_mu[0]
                    else:
                        # if the robot stops moving, give it some random action
                        print 'Give the robot some random action'
                        action = np.random.uniform(-2.0, 2.0, 4)
                        stuck = False

                    res = env.step(action)      # take the action
                    ob = res[0]
                    rewards.append(res[1])

                    if res[2] or res[4]:                               # record a path when an episode is terminated
                        rospy.loginfo("Episode is over.")
                        break

                reach_goal = res[3]

                # calculate the average dis
                j = (angle - angles[0]) / 5
                if reach_goal:
                    success_times[j].append(1.0)
                else:
                    success_times[j].append(0.0)

                moco.delete_model(pipe_name)
                moco.delete_model('target')

                timesteps_used = len(rewards)
                rospy.loginfo("The time steps used: " + str(timesteps_used))

            # calculate mean for each angle
            data_dir = '/home/joshua/robot_ws_earlier/src/mrm_training_force/training_results/compare_results1/Validation/' + \
            'success_rate_%s_%s.txt' % (string_coff, d)
            success_rate = [np.mean(success_time) for success_time in success_times]
            np.savetxt(data_dir, success_rate, fmt='%f')

    rospy.signal_shutdown('Shut down the node')
