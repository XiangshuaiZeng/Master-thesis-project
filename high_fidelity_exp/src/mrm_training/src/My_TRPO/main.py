import numpy as np
import tensorflow as tf
import gym
from utils import *
from model import *
import argparse
from rollouts import *
import json

parser = argparse.ArgumentParser(description='TRPO.')
# these parameters should stay the same
parser.add_argument("--timesteps_per_batch", type=int, default=10000)  # the time steps from the env for each batch
parser.add_argument("--n_steps", type=int, default=600000)   # the total training time steps
parser.add_argument("--gamma", type=float, default=.99)
parser.add_argument("--max_kl", type=float, default=.001)
parser.add_argument("--cg_damping", type=float, default=1e-3)

# change these parameters for testing
parser.add_argument("--decay_method", type=str, default="adaptive") # adaptive, none at this moment
parser.add_argument("--timestep_adapt", type=int, default=0)
parser.add_argument("--kl_adapt", type=float, default=0)

args = parser.parse_args()
args.max_pathlength = gym.spec(args.task).timestep_limit   # which is 500 at this moment

# build the env and the agent
env = gym.make(args.task)
agent = TRPO(args, learner_env.observation_space, learner_env.action_space)

start_time = time.time()
history = {}
history["rollout_time"] = []
history["learn_time"] = []
history["mean_reward"] = []
history["timesteps"] = []
history["maxkl"] = []

# start it off with a big negative number
last_reward = -1000000
recent_total_reward = 0

totalsteps = 0;

starting_timesteps = args.timesteps_per_batch
starting_kl = args.max_kl

iteration = 0
while True:
    iteration += 1;

    rollouts = rollout(env, agent, args.max_pathlength, args.timesteps_per_batch)   # collect data
    # train the agent
    agent.learn(rollouts)

    recent_total_reward += mean_reward

    if args.decay_method == "adaptive":
        if iteration % 10 == 0:
            if recent_total_reward < last_reward:
                print "Policy is not improving. Decrease KL and increase steps."
                if args.timesteps_per_batch < 20000:
                    args.timesteps_per_batch += args.timestep_adapt
                if args.max_kl > 0.001:
                    args.max_kl -= args.kl_adapt
            else:
                print "Policy is improving. Increase KL and decrease steps."
                if args.timesteps_per_batch > 1200:
                    args.timesteps_per_batch -= args.timestep_adapt
                if args.max_kl < 0.01:
                    args.max_kl += args.kl_adapt
            last_reward = recent_total_reward
            recent_total_reward = 0


    if args.decay_method == "adaptive-margin":
        if iteration % 10 == 0:
            scaled_last = last_reward + abs(last_reward * 0.05)
            print "Last reward: %f Scaled: %f Recent: %f" % (last_reward, scaled_last, recent_total_reward)
            if recent_total_reward < scaled_last:
                print "Policy is not improving. Decrease KL and increase steps."
                if args.timesteps_per_batch < 10000:
                    args.timesteps_per_batch += args.timestep_adapt
                if args.max_kl > 0.001:
                    args.max_kl -= args.kl_adapt
            else:
                print "Policy is improving. Increase KL and decrease steps."
                if args.timesteps_per_batch > 1200:
                    args.timesteps_per_batch -= args.timestep_adapt
                if args.max_kl < 0.01:
                    args.max_kl += args.kl_adapt
            last_reward = recent_total_reward
            recent_total_reward = 0

    # print "Current steps is " + str(args.timesteps_per_batch) + " and KL is " + str(args.max_kl)

    if iteration % 100 == 0:
        with open("%s-%s-%f-%f-%f-%f" % (args.task, args.decay_method, starting_timesteps, starting_kl, args.timestep_adapt, args.kl_adapt), "w") as outfile:
            json.dump(history,outfile)

    totalsteps += args.timesteps_per_batch
    print "%d total steps have happened" % totalsteps
    if totalsteps > args.n_steps:
        break
