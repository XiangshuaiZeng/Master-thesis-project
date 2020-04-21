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
from std_msgs.msg import Float64, Bool
# ROS packages required
import rospy
import rospkg
from load_model import model_control
from geometry_msgs.msg import Pose, Quaternion, Point, Transform
from joint_publisher import JointPub
# import our predicting environment
import mrm_env_predicting_history
from spawn_model import SpawnModel

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

    offset = (120 - diameter) / 10.0 * 0.003
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
    dis_target_left = [0.22, 0.225]

    b = 0.073 + random.uniform(0.0, 0.018)

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

def quaternion_mul_vector(q, v):
    # Extract the vector part of the quaternion
    u = np.asarray([q.x, q.y, q.z])
    v = np.asarray(v)
    # Extract the scalar part of the quaternion
    s = q.w
    # Do the math
    vprime = 2.0 * np.dot(u, v) * u + (s*s - np.dot(u, u)) * v + 2.0 * s * np.cross(u, v)

    return vprime

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
    "max_pathlength" : 90,
    "n_validation": 120
    }

    moco = model_control()
    # Create the Gym environment
    env = gym.make('Mrm-v0')
    env = env.unwrapped
    rospy.loginfo ("Gym environment done")

    # position and orientation of the end-effector
    global end_pos
    def end_effector_pos_callback(msg):
        global end_pos
        end_pos = msg
    # Get the position of end-effector
    rospy.Subscriber("/end_effector_pos", Transform, end_effector_pos_callback)

    # Initialises the algorithm that we are going to use for predicting
    agent = PPO(args=args, observation_size=env.observation_space,
                    action_size=env.action_space)

    # load the trained model
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mrm_training')
    outdir = pkg_path + '/training_results'
    string_coffs = ['15']

    # pipe coefficient
    angles = range(55, 126, 5)
    D = [120, 110, 100, 90]

    # joint publisher
    mrm_jointpub = JointPub()
    rospy.loginfo ("Start Validation")

    sm = SpawnModel()
    sm.urdf_format = True
    sm.initial_xyz = [0, 0, 0]
    sm.initial_rpy = [0, 0, 0]
    sm.model_name = 'mrm'


    for string_coff in string_coffs:
        modeldir = outdir + '/PPO_model_new/high_fidelity_%s_1.ckpt' % string_coff
        # modeldir = outdir + '/PPO_model/pirate_pos_control3.ckpt'
        agent.load_model(modeldir)

        # start training from turning left
        for d in D:
            # for recording the average distance between the end-effector and the target every episode
            success_times = [[] for _ in range(len(angles))]
            success_times_second = [[] for _ in range(len(angles))]
            # change the init joint position to adapt with pipe diameter
            d = 100
            env.change_joints_init(d)
            # get the init position of the upper joints
            init_joints = env.get_joints_init()
            init_joints_2 = init_joints[0:2] + [-init_joints[2]]

            for episode in range(args['n_validation']):
                targets_path = []
                targets_path.append(env.mrm_joint_pubisher_object.init_pos)
                # choose a pipe randomly
                angle = 85 #angles[episode % len(angles)]
                ob = env.reset()        # reset env and get the initial state

                # spawn the pipe
                pipe_name = change_pipe(moco, d, angle)
                # env.gazebo.pauseSim()
                # x = input()

                # ################ start locating a target #########################
                delta_third = 0.15
                delta_trans = 0.02
                while True:
                    joint_targets = ob[0:4] + np.asarray([0.0, 0.0, delta_third, delta_trans])
                    targets_path.append(joint_targets)  # record the path
                    res = env.step(joint_targets)      # take the action

                    ob = res[0]

                    if min(ob[12:15]) < 0.04:
                        delta_trans /= 8.0

                    if max(ob[8:18]) == 0.5:
                        target_dis = 0.2 #+ random.uniform(-0.02, 0.02)
                        target_pos = [target_dis* sin(45.0/180*pi), -target_dis * cos(45.0/180*pi), 0]
                        target_pos = quaternion_mul_vector(end_pos.rotation, target_pos)
                        target_pos = target_pos + np.asarray([end_pos.translation.x, end_pos.translation.y, end_pos.translation.z])
                        # spawn the target
                        target_name = 'target'    # the model name and namespace
                        targetdir = '/home/joshua/high_fidelity_exp/src/mrm_training/urdf/target.xacro'
                        # visulize the target in Gazebo
                        moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], 0.0], [0, 0, 0])
                        break

                # ############ target found ad spawned ###########################
                # spawn the target
                # target_pos = change_target_pos(moco, d, angle)
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

                    if not stuck:
                        # get action from the policy network
                        action, action_dist_mu, _ = agent.act(ob)
                        # action = action_dist_mu[0]
                    else:
                        # if the robot stops moving, give it some random action
                        print 'Give the robot some random action'
                        action = np.random.uniform(-2.0, 2.0, 3)
                        prism = random.uniform(-0.5, 0.5)
                        action = np.concatenate((action, [prism]))
                        stuck = False

                     # smooth the trajectory
                    alpha = 0.2
                    joint_targets = ob[0:4] + alpha * action
                    res = env.step(joint_targets)      # take the action
                    ob = res[0]
                    rewards.append(res[1])

                    # record the joint trajectory
                    targets_path.append(joint_targets)

                    if res[2] or res[4]:                               # record a path when an episode is terminated
                        rospy.loginfo("Episode is over.")
                        break

                reach_goal = res[3]

                #calculate the average dis
                j = (angle - angles[0]) / 5
                if reach_goal:
                    success_times[j].append(1.0)
                else:
                    success_times[j].append(0.0)

                moco.delete_model('target')
                # moco.delete_model(pipe_name)
                timesteps_used = len(rewards)
                rospy.loginfo("The time steps used: " + str(timesteps_used))

                if not reach_goal:
                    moco.delete_model(pipe_name)
                else:
                    # ####################### Let the arm go back #############################################
                    # # record the final configuration of the joints
                    last_ob = ob
                    last_ob[3] -= 0.01

                    # delete and respawn the robotic arm
                    moco.delete_model('mrm')
                    sm.param_name = 'robot_description%d' % angle
                    sm.joint_names = ["joint_01", "joint_02", "joint_03", "prismatic"]
                    sm.joint_positions = last_ob[0:4]
                    print 'SpawnModel'
                    sm.callSpawnService()
                    print 'Succeed!!'
                    time.sleep(2.0)

                    # now start moving backwards
                    path_len = len(targets_path)
                    translation_noise = random.uniform(-0.05,0.05)

                    for step in range(path_len):
                        set_point = targets_path[-1 - step]
                        # set_point[3] += translation_noise
                        res = env.step(set_point)
                        g = raw_input("Enter any key for next step")

                    # targets_path[0][3] -= translation_noise
                    for _ in range(14):
                        res = env.step(targets_path[0])

                    ob = res[0]
                    j = (angle - angles[0]) / 5

                    if abs(ob[3]) < 0.02:
                        rospy.loginfo('Second phase succeed!' + '\n')
                        success_times_second[j].append(1.0)
                    else:
                        success_times_second[j].append(0.0)

                    env.gazebo.unpauseSim()
                    env.pause_sim_pub.publish(Bool(False))

                    time.sleep(5.0)
                    env.pause_sim_pub.publish(Bool(True))
                    #delete the arm and pipe in gazebo
                    moco.delete_model('mrm')
                    moco.delete_model(pipe_name)

                    # spawn the original robotic arm
                    sm.param_name = 'robot_description'
                    sm.joint_names = []
                    sm.joint_positions = []
                    sm.callSpawnService()

            # calculate mean for each angle
            data_dir1 = '/home/joshua/high_fidelity_exp/src/mrm_training/training_results/high_fidelity_simulation/Validation/No_approach_ii/' + \
            'success_rate_high_fidelity_%s_%s.txt' % (string_coff, d)
            success_rate = [np.mean(success_time) for success_time in success_times]
            np.savetxt(data_dir1, success_rate, fmt='%f')

            data_dir2 = '/home/joshua/high_fidelity_exp/src/mrm_training/training_results/high_fidelity_simulation/Validation/No_approach_ii/' + \
            'success_rate_second_%s_%s.txt' % (string_coff, d)
            success_rate = [np.mean(success_time) for success_time in success_times_second]
            np.savetxt(data_dir2, success_rate, fmt='%f')

    rospy.signal_shutdown('Shut down the node')
