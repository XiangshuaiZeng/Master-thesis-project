#!/usr/bin/env python

'''
    Original Training code made by Ricardo Tellez <rtellez@theconstructsim.com>
    Moded by Miguel Angel Rodriguez <duckfrost@theconstructsim.com>
    Adapted by Xiangshuai Zeng at the University of Twente for master thesis
'''
import gym
import numpy as np
from math import *
import random
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
import mrm_env_training_history

def change_pipe(moco, diameter, angle):
    # info for spawning the pipe
    pipedir = '/home/joshua/robot_arm_ws/src/mrm_training_force/urdf/pipe_template.xacro'

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

    pipe_pos_left = [0.077, -0.31, 0]
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

if __name__ == '__main__':
    rospy.init_node('mrm_gym_training', anonymous=True, log_level=rospy.INFO, disable_signals=True)

    args = {
    # these parameters should stay the same
    "timesteps_per_minibatch" : 400,
    "task" : "Mrm-v0",
    "timesteps_per_batch" : 800,
    "gamma" : 0.98,
    "lamda" : 0.975,   # Best lambda value is lower than gamma, empirically lambda introduces far less bias than gamma for a reasonably accruate value function'
    "vf_constraint" : 0.01,
    "max_kl" : .01,
    "coef_en": 0.001,  # coefficient for entropy bonus, only used in PPO
    "lr_ph": 0.001, # learning rate for adam, only used in PPO
    "cg_damping" : 0.001,
    "timesteps_per_episode" : 100,
    "max_iteration" : 2
    }

    moco = model_control()
    # Create the Gym environment
    env = gym.make('Mrm-v0')
    env = env.unwrapped
    rospy.logdebug ("Gym environment done")

    # Set the logging system
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mrm_training')
    outdir = pkg_path + '/training_results'


    #Initialises the algorithm that we are going to use for learning
    agent = PPO(args=args, observation_size=env.observation_space,
                    action_size=env.action_space)

    string_coff = ['15']

    i = 0
    for obs_coff in [1.5]:
        modeldir = outdir + '/PPO_model_new/high_fidelity_%s_1.ckpt' % string_coff[i]
        # save the episode reward and the trained RL model
        data_dir = outdir + '/iteration_reward_high_fidelity_%s_1.txt' % string_coff[i]
        data_dir1 = outdir + '/success_times_high_fidelity_%s_1.txt' % string_coff[i]
        env.mrm_state_object._weight_r2 = obs_coff

        #re-initilize the weights
        agent.initilize_weights()

        # load the previous trained model
        agent.load_model(modeldir)

        # start training from turning left
        angles = [60, 70, 80, 90, 100, 110, 120]

        iterations_rewards = []
        if_reach_goal = []
        lasers_dis = []

        for iteration in range(args['max_iteration']):
            # choose a pipe randomly
            angle = random.choice([60, 70])
            d = 120
            # spawn the pipe
            pipe_name = change_pipe(moco, d, angle)
            # start the simulation and training
            totalsteps = 0
            while True:
                 # set the pipe angle for generating target pos
                env.pipe_angle = angle
                rospy.loginfo ("START..............")
                # collect a batch of data
                rollouts, reach_targets = rollout(env, agent, args["timesteps_per_episode"], args["timesteps_per_batch"])   # collect data
                rospy.loginfo ("Data collection is done, start training the model........")
                # train the agent
                episodereward = agent.learn(rollouts)
                iteration_reward = np.mean(episodereward)
                iterations_rewards.append(iteration_reward)
                rospy.loginfo ("Training is done...............")

                # record success times
                if_reach_goal.append(np.mean(reach_targets))

                totalsteps += args["timesteps_per_batch"]

                # save the episode rewards and success rate
                np.savetxt(data_dir, iterations_rewards, fmt='%f')
                np.savetxt(data_dir1, if_reach_goal, fmt='%f')

                print "%d total steps have happened" % totalsteps
                if totalsteps >= args["timesteps_per_batch"]:
                    print "%d iterations have passed" % iteration
                    break
            rospy.loginfo ("Training is over, saving the model")
            agent.save_model(modeldir)   # save the model
            # delete the pipe
            moco.delete_model(pipe_name)

        i += 1

    rospy.signal_shutdown('Training is completed')
