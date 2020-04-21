import numpy as np
import tensorflow as tf
from utils import *
import random
from GAE.gae import GAE

# the policy net is set to be with two hidden layers

class PPO():
    def __init__(self, args, observation_size, action_size, hidden_size1 = 128, hidden_size2 = 64, hidden_size3 = 32):   # __init__ will build the computation graph for TRPO
        self.observation_size = observation_size
        self.action_size = action_size
        self.hidden_size1 = hidden_size1
        self.hidden_size2 = hidden_size2
        self.hidden_size3 = hidden_size3
        self.args = args
        self.clip_range = 0.2

        # initilizer for weights and biases in NN
        weight_init = tf.random_uniform_initializer(-0.1, 0.1)
        bias_init = tf.constant_initializer(0)

        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        self.session = tf.Session(config=config)

        self.obs = tf.placeholder(tf.float32, [None, self.observation_size]) # feed with a rollout of states
        self.action = tf.placeholder(tf.float32, [None, self.action_size])   # feed with a rollout of actions
        self.advantage = tf.placeholder(tf.float32, [None])
        self.oldaction_dist_mu = tf.placeholder(tf.float32, [None, self.action_size])
        self.oldaction_dist_logstd = tf.placeholder(tf.float32, [None, self.action_size])

        # construct the policy net with three hidden layers
        with tf.variable_scope("policy"):
            h1 = fully_connected(self.obs, self.observation_size, self.hidden_size1, weight_init, bias_init, "policy_h1")
            h1 = tf.nn.tanh(h1)
            h2 = fully_connected(h1, self.hidden_size1, self.hidden_size2, weight_init, bias_init, "policy_h2")
            h2 = tf.nn.tanh(h2)
            h3 = fully_connected(h2, self.hidden_size2, self.hidden_size3, weight_init, bias_init, "policy_h3")
            h3 = tf.nn.tanh(h3)
            h4 = fully_connected(h3, self.hidden_size3, self.action_size, weight_init, bias_init, "policy_h4")
            action_dist_logstd_param = tf.Variable((.01*np.random.randn(1, self.action_size)).astype(np.float32) - 0.3, name="policy_logstd")
        # means for each action
        self.action_dist_mu = h4
        # log standard deviations for each actions
        self.action_dist_logstd = tf.tile(action_dist_logstd_param, tf.stack((tf.shape(self.action_dist_mu)[0], 1)))

        ################ until now the policy net is built ##############################################

        batch_size = tf.shape(self.obs)[0]   # the total timesteps of one rollout

        # what are the probabilities of taking self.action, given new and old distributions
        log_p_n = gauss_log_prob(self.action_dist_mu, self.action_dist_logstd, self.action)    # self.action contains actions for different states
        log_oldp_n = gauss_log_prob(self.oldaction_dist_mu, self.oldaction_dist_logstd, self.action)

        # tf.exp(log_p_n) / tf.exp(log_oldp_n)
        ratio = tf.exp(log_p_n - log_oldp_n)

        # Original importance sampling of surrogate loss (L in paper)
        surr_orig = -tf.reduce_mean(ratio * self.advantage)
        # clipped surrogate loss
        surr_clipped = -tf.reduce_mean(tf.clip_by_value(ratio, 1.0 - self.clip_range, 1.0 + self.clip_range) * self.advantage)

        surr = tf.maximum(surr_orig, surr_clipped)
        var_list = tf.trainable_variables()

        # the total amount of time steps
        batch_size_float = tf.cast(batch_size, tf.float32)

        # kl divergence and shannon entropy
        kl = gauss_KL(self.oldaction_dist_mu, self.oldaction_dist_logstd, self.action_dist_mu, self.action_dist_logstd) / batch_size_float
        ent = gauss_ent(self.action_dist_mu, self.action_dist_logstd) / batch_size_float

        self.losses = [surr, kl, ent]

        # add a entropy bonus to ensure sufficient exploration
        self.policy_loss = surr + args["coef_en"] * ent

        # adam optimizer for updating parameters
        self.learning_rate = tf.placeholder(tf.float32)
        optimizer = tf.train.AdamOptimizer(self.learning_rate)
        self.train_op = optimizer.minimize(self.policy_loss)

        self.session.run(tf.global_variables_initializer())   # initilize the weights and biaes in the policy net

        # General advantage estimation
        self.gae = GAE(self.session, self.observation_size, self.args["gamma"], self.args["lamda"], self.args["vf_constraint"])

        ############################### object for saving the model ########################################
        self.saver = tf.train.Saver()

    def learn(self, paths):

        # puts all the experiences in a matrix: total_timesteps x options
        action_dist_mus = np.concatenate([path["action_dists_mu"] for path in paths])
        action_dist_logstds = np.concatenate([path["action_dists_logstd"] for path in paths])
        obs_ns = np.concatenate([path["obs"] for path in paths])
        action_ns = np.concatenate([path["actions"] for path in paths])

        # Get advantage from gae
        advant_ns = self.gae.get_advantage(paths)

        n_batch = self.args["timesteps_per_batch"] / self.args["timesteps_per_minibatch"]
        size = 0
        for e in range(n_batch):
            action_dist_mu = action_dist_mus[size:(size+self.args["timesteps_per_minibatch"])]
            action_dist_logstd = action_dist_logstds[size:(size+self.args["timesteps_per_minibatch"])]
            obs_n = obs_ns[size:(size+self.args["timesteps_per_minibatch"])]
            action_n = action_ns[size:(size+self.args["timesteps_per_minibatch"])]
            advant_n = advant_ns[size:(size+self.args["timesteps_per_minibatch"])]
            feed_dict = {self.obs: obs_n, self.action: action_n, self.advantage: advant_n, self.oldaction_dist_mu: action_dist_mu, \
            self.oldaction_dist_logstd: action_dist_logstd, self.learning_rate: self.args["lr_ph"]}
            ################This is where the policy net is updated########################################
            self.session.run(self.train_op, feed_dict)
            size += self.args["timesteps_per_minibatch"]

        # Update value function parameter
        # Policy update is perfomed using the old value function parameter
        self.gae.train()

        surrogate_after, kl_after, entropy_after = self.session.run(self.losses,feed_dict)

        # adapt the learning rate based on kl_divergence
        if kl_after > 2.0 * self.args["max_kl"]:
            self.args["lr_ph"] /= 2.0
        elif kl_after < 0.5 * self.args["max_kl"]:
            self.args["lr_ph"] *= 2.0

        episoderewards = np.array(
            [path["rewards"].sum()/len(path["rewards"]) for path in paths])
        stats = {}
        stats["Average sum of rewards per episode"] = episoderewards.mean()
        stats["Entropy"] = entropy_after
        stats["Timesteps"] = sum([len(path["rewards"]) for path in paths])
        stats["KL between old and new distribution"] = kl_after
        stats["Surrogate loss"] = surrogate_after
        # print ("\n********** Iteration {} ************".format(i))
        for k, v in stats.iteritems():
            print(k + ": " + " " * (40 - len(k)) + str(v))

        return episoderewards

    # function for getting the action of a given state
    def act(self, obs):
        obs = np.expand_dims(obs, 0)
        action_dist_mu, action_dist_logstd = self.session.run([self.action_dist_mu, self.action_dist_logstd], feed_dict={self.obs: obs})
        # samples the guassian distribution
        act = np.random.normal(loc=action_dist_mu, scale=np.exp(action_dist_logstd))
        #act = action_dist_mu + np.exp(action_dist_logstd)*np.random.randn(*action_dist_logstd.shape)
        return act.ravel(), action_dist_mu, action_dist_logstd

    def save_model(self, path):
        self.saver.save(self.session, path)

    def load_model(self, path):
        self.saver.restore(self.session, path)

    def initilize_weights(self):
        self.session.run(tf.global_variables_initializer())
