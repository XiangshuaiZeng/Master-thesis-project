import numpy as np
import tensorflow as tf
from utils import *
import random
from GAE.gae import GAE

# the policy net is set to be with two hidden layers

class TRPO():
    def __init__(self, args, observation_size, action_size, hidden_size1 = 128, hidden_size2 = 64, hidden_size3 = 32):   # __init__ will build the computation graph for TRPO
        self.observation_size = observation_size
        self.action_size = action_size
        self.hidden_size1 = hidden_size1
        self.hidden_size2 = hidden_size2
        self.hidden_size3 = hidden_size3
        self.args = args

        # initilizer for weights and biases in NN
        weight_init = tf.random_uniform_initializer(-0.1, 0.1)
        bias_init = tf.constant_initializer(0)

        self.session = tf.Session()

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
            action_dist_logstd_param = tf.Variable((.01*np.random.randn(1, self.action_size)).astype(np.float32), name="policy_logstd")
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
        # importance sampling of surrogate loss (L in paper)
        surr = -tf.reduce_mean(ratio * self.advantage)
        var_list = tf.trainable_variables()

        eps = 1e-8
        batch_size_float = tf.cast(batch_size, tf.float32)
        # kl divergence and shannon entropy
        kl = gauss_KL(self.oldaction_dist_mu, self.oldaction_dist_logstd, self.action_dist_mu, self.action_dist_logstd) / batch_size_float
        ent = gauss_ent(self.action_dist_mu, self.action_dist_logstd) / batch_size_float

        self.losses = [surr, kl, ent]
        # policy gradient
        self.pg = flatgrad(surr, var_list)   # still, only the graph is built

        ############### build the computation graph for H matrix ###############
        # KL divergence w/ itself, with first argument kept constant.
        kl_firstfixed = gauss_selfKL_firstfixed(self.action_dist_mu, self.action_dist_logstd) / batch_size_float
        # gradient of KL w/ itself
        grads = tf.gradients(kl_firstfixed, var_list)
        # what vector we're multiplying by
        self.flat_tangent = tf.placeholder(tf.float32, [None])   # ????
        shapes = map(var_shape, var_list)   # contains the dimensions for each pair of layers in NN
        start = 0
        tangents = []
        for shape in shapes:
            size = np.prod(shape)
            param = tf.reshape(self.flat_tangent[start:(start + size)], shape)
            tangents.append(param)
            start += size
        # gradient of KL w/ itself * tangent
        gvp = [tf.reduce_sum(g * t) for (g, t) in zip(grads, tangents)]
        # 2nd gradient of KL w/ itself * tangent
        self.fvp = flatgrad(gvp, var_list)
        # the actual parameter values (because the session is run)
        self.gf = GetFlat(self.session, var_list)
        # call this to set parameter values
        self.sff = SetFromFlat(self.session, var_list)
        self.session.run(tf.global_variables_initializer())   # initilize the weights and biaes in the policy net

        # General advantage estimation
        self.gae = GAE(self.session, self.observation_size, self.args["gamma"], self.args["lamda"], self.args["vf_constraint"])

        ############################### object for saving the model ########################################
        self.saver = tf.train.Saver()
        self.get_policy = GetPolicyWeights(self.session, var_list)


    def learn(self, paths):          # this function will actually run the computation graph and update the policy net by using input data

        # puts all the experiences in a matrix: total_timesteps x options
        action_dist_mu = np.concatenate([path["action_dists_mu"] for path in paths])
        action_dist_logstd = np.concatenate([path["action_dists_logstd"] for path in paths])
        obs_n = np.concatenate([path["obs"] for path in paths])
        action_n = np.concatenate([path["actions"] for path in paths])

        # Get advantage from gae
        advant_n = self.gae.get_advantage(paths)

        feed_dict = {self.obs: obs_n, self.action: action_n, self.advantage: advant_n, self.oldaction_dist_mu: action_dist_mu, self.oldaction_dist_logstd: action_dist_logstd}

        # parameters
        thprev = self.gf()

        # computes fisher vector product: F * [self.pg]
        def fisher_vector_product(p):
            feed_dict[self.flat_tangent] = p
            return self.session.run(self.fvp, feed_dict) + p * self.args["cg_damping"]

        # the actual values of policy gradients
        g = self.session.run(self.pg, feed_dict)

        # solve Ax = g, where A is Fisher information metrix and g is gradient of parameters
        # step_direction = A_inverse * g = x
        stepdir = conjugate_gradient(fisher_vector_product, -g)

        # let stepdir =  change in theta / direction that theta changes in
        # KL divergence approximated by 0.5 x stepdir_transpose * [Fisher Information Matrix] * stepdir
        # where the [Fisher Information Matrix] acts like a metric
        # ([Fisher Information Matrix] * stepdir) is computed using the function,
        # and then stepdir * [above] is computed manually.
        shs = 0.5 * stepdir.dot(fisher_vector_product(stepdir))

        lm = np.sqrt(shs / self.args["max_kl"])

        # the update step for theta
        fullstep = stepdir / lm
        negative_g_dot_steppdir = -g.dot(stepdir)

        def loss(th):
            self.sff(th)
            # surrogate loss: policy gradient loss
            return self.session.run(self.losses[0], feed_dict)

        # finds best parameter by starting with a big step and working backwards
        theta = linesearch(loss, thprev, fullstep, negative_g_dot_steppdir/ lm)

        # Update value function parameter
		# Policy update is perfomed using the old value function parameter
        self.gae.train()

        ################This is where the policy net is updated########################################
        self.sff(theta)

        surrogate_after, kl_after, entropy_after = self.session.run(self.losses,feed_dict)

        episoderewards = np.array(
            [path["rewards"].sum() for path in paths])
        stats = {}
        stats["Average sum of rewards per episode"] = episoderewards.mean()
        stats["Entropy"] = entropy_after
        stats["max KL"] = self.args["max_kl"]
        stats["Timesteps"] = sum([len(path["rewards"]) for path in paths])
        # stats["Time elapsed"] = "%.2f mins" % ((time.time() - start_time) / 60.0)
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
