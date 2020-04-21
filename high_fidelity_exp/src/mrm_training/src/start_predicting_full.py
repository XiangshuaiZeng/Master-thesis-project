#!/usr/bin/env python
import gym
import time
import numpy as np
import random
from math import *
from My_PPO.model import PPO
from My_PPO.utils import rollout
from gym import wrappers
from std_msgs.msg import Float64, Bool
# ROS packages required
import rospy
import rospkg
from load_model import model_control
# import our training environment
import mrm_env_predicting
from joint_publisher import JointPub
# for respawn the robot
from geometry_msgs.msg import Pose, Quaternion, Point, Transform
from spawn_model import SpawnModel

def change_pipe(moco, diameter, angle):
    # info for spawning the pipe
    pipedir = '/home/joshua/high_fidelity_exp/src/mrm_training/urdf/pipe_template.xacro'

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

    pipe_pos_left = [0.065, -0.31, 0]
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


def change_target_pos(moco, angle):
    # how far will be the target away from the turning center approximately
    dis_target_left = [0.22, 0.25]
    dis_target = random.uniform(dis_target_left[0], dis_target_left[1])
    target_pos = [dis_target*cos((angle-90.0)/180*pi), -0.4 - dis_target*sin((angle-90.0)/180*pi)]
    # add some noise
    target_pos[1] += random.uniform(-0.03, 0.03)

    target_name = 'target'    # the model name and namespace
    targetdir = '/home/joshua/high_fidelity_exp/src/mrm_training/urdf/target.xacro'

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
    "timesteps_per_episode" : 70,
    "n_validation" : 120
    }

    # position and orientation of the end-effector
    global end_pos
    def end_effector_pos_callback(msg):
        global end_pos
        end_pos = msg
    # Get the position of end-effector
    rospy.Subscriber("/end_effector_pos", Transform, end_effector_pos_callback)

    # for spawn models
    moco = model_control()
    # Create the Gym environment
    env = gym.make('Mrm-v0')
    env = env.unwrapped
    rospy.loginfo ("Gym environment done")

    mrm_jointpub = JointPub()

    # Initialises the algorithm that we are going to use for predicting
    agent = PPO(args=args, observation_size=env.observation_space,
                    action_size=env.action_space)

    # load the trained model
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('mrm_training')
    outdir = pkg_path + '/training_results'
    modeldir = outdir + '/PPO_model/pirate_pos_control3.ckpt'
    agent.load_model(modeldir)

    # start training from turning left
    rospy.loginfo ("Start Validation")
    success_time = 0.0
    angles = range(60, 126, 5)
    D = [80,90,100,110,120]
    episodes_rewards = np.array([])

    sm = SpawnModel()
    sm.urdf_format = True
    sm.initial_xyz = [0, 0, 0]
    sm.initial_rpy = [0, 0, 0]
    sm.model_name = 'mrm'

    for episode in range(args['n_validation']):
        # choose a pipe
        angle = angles[episode % len(angles)]
        d = 110
        # change the init joint position to adapt with pipe diameter
        env.change_joints_init(d)
        # get the init position of the upper joints
        init_joints = env.get_joints_init()
        init_joints_2 = init_joints[0:2] + [-init_joints[2]]
        ob = env.reset()        # reset env and get the initial state
        # spawn the pipe
        pipe_name = change_pipe(moco, d, angle)
        ################ start locating a target #########################
        targets_path = []
        delta_third = 0.15
        delta_trans = 0.02
        while True:
            joint_targets = ob[0:4] + np.asarray([0.0, 0.0, delta_third, delta_trans])
            targets_path.append(joint_targets)  # record the path
            joint_targets = np.concatenate((joint_targets, init_joints_2))
            res = env.step(joint_targets)      # take the action

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
                moco.spawn_model(targetdir, target_name, target_name, [target_pos[0], target_pos[1], 0.0], [0, 0, 0])
                break
        ############ target found ad spawned ###########################

        # set the target point for the gym env
        env.set_target_position([target_pos[0], target_pos[1], 0.0])
        # for checking the joint positions
        stuck = False
        joint_pos = []
        joint_delta_min = 0.2
        N = 3
        # start the validation
        rewards = []
        for time_step in xrange(args['timesteps_per_episode']):
            #check whether the robotic arm stops moving
            if time_step % N == 0:
                joint_pos.append(np.asarray(ob[0:3]))
                if len(joint_pos) > 1:
                    joint_delta = np.linalg.norm(joint_pos[-1] - joint_pos[-2])
                    stuck = joint_delta < joint_delta_min

            # get action from the policy network
            action, action_dist_mu, _ = agent.act(ob)
            action = action_dist_mu[0]
            if stuck and not stuck:
                # if the robot stops moving, give it some random action
                print 'Give the robot some random action'
                action[0:3] = np.random.uniform(-1.0, 1.0, 3)
                action[3] = np.random.uniform(-0.06, 0.03, 1)
                stuck = False

            alpha = 0.2  # for smoothing the trajectory
            joint_targets = ob[0:4] + alpha * action
            targets_path.append(joint_targets)    # store the target path
            # add the target pos for lower part joints
            joint_targets = np.concatenate((joint_targets, init_joints_2))
            res = env.step(joint_targets)      # take the action

            ob = res[0]
            rewards.append(res[1])
            if res[2]:                               # record a path when an episode is terminated
                rospy.loginfo("Episode is over.")
                if time_step < args['timesteps_per_episode']:
                    success_time += 1.0
                break

        timesteps_used = len(rewards)
        rospy.loginfo("The time steps used: " + str(timesteps_used))
        moco.delete_model('target')

        ####################### Let the arm go back #############################################
        # At this moment, the tail of the robot will start moving, so all the target positions should become negative
        # record the final configuration of the joints
        # last_ob = ob
        # # delete and respawn the robotic arm
        # moco.delete_model('mrm')
        #
        # sm.param_name = 'robot_description%d' % angle
        # sm.joint_names = ["joint_01", "joint_02", "joint_03", "prismatic", "joint_01_2", "joint_02_2", "joint_03_2"]
        #
        # # init_pos_second = init_joints[0:4] + init_joints_2
        # init_pos_second = init_joints[0:3] + [-last_ob[3], last_ob[0], last_ob[1], -last_ob[2]]
        # sm.joint_positions = init_pos_second
        # sm.callSpawnService()
        # time.sleep(2.0)
        #
        # # # Give some random purtubation to the joints
        # # random_action = list(np.random.uniform(-0.02, 0.02, 3))
        # # random_action.append(0.0)  # the prismatic joint remain still
        # # joint_targets = last_ob[0:4] + action
        # # env.gazebo.unpauseSim()
        # # env.pause_sim_pub.publish(Bool(False))
        # # mrm_jointpub.move_joints(joint_targets)
        # # time.sleep(2.0)
        # # env.pause_sim_pub.publish(Bool(True))
        #
        # # now start move backwards
        # path_len = len(targets_path)
        # for step in range(path_len):
        #     joint_pas = targets_path[-1 - step]
        #     joint_targets = init_joints[0:3] +  [-joint_pas[0], joint_pas[1], joint_pas[2] ,-joint_pas[3]]
        #     env.step(joint_targets)
        #
        # env.gazebo.unpauseSim()
        # env.pause_sim_pub.publish(Bool(False))
        # mrm_jointpub.move_joints(init_joints[0:4] + init_joints_2)
        # time.sleep(5.0)
        # env.pause_sim_pub.publish(Bool(True))
        # #delete the arm and pipe in gazebo
        # moco.delete_model('mrm')
        # moco.delete_model(pipe_name)
        #
        # # spawn the original robotic arm
        # sm.param_name = 'robot_description'
        # sm.joint_names = []
        # sm.joint_positions = []
        # sm.callSpawnService()

    rospy.signal_shutdown('Shut down the node')
