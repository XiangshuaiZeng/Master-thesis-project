import tensorflow as tf
import numpy as np
import scipy.signal
import rospy


# function for collecting paths of data
def rollout(env, agent, max_pathlength, n_timesteps):
    paths = []
    timesteps_sofar = 0
    while timesteps_sofar < n_timesteps:   # the overall time-steps per batch, usually a large number
        obs, actions, rewards, action_dists_mu, action_dists_logstd = [], [], [], [], []
        dones = []
        ob = env.reset()        # reset env and get the initial state   # bug 1
        for _ in xrange(max_pathlength):
            obs.append(ob)
            # get action from the policy
            action, action_dist_mu, action_dist_logstd = agent.act(ob)
            actions.append(action)
            action_dists_mu.append(action_dist_mu)
            action_dists_logstd.append(action_dist_logstd)
            res = env.step(action)      # take the action
            ob = res[0]
            rewards.append(res[1])
            dones.append(res[2])
            if res[2]:                               # record a path when an episode is terminated
                break

        path = {"obs": np.concatenate(np.expand_dims(obs, 0)),
                     "action_dists_mu": np.concatenate(action_dists_mu),
                     "action_dists_logstd": np.concatenate(action_dists_logstd),
                     "rewards": np.array(rewards),
                     "actions":  np.array(actions),
                     "done" : np.array(dones)}
        paths.append(path)
        rospy.loginfo("One episode is over.")

        timesteps_sofar += len(path["rewards"])
        rospy.loginfo("The time steps so far: " + str(timesteps_sofar) + "\n")
    return paths

# KL divergence with itself, holding first argument fixed
def gauss_selfKL_firstfixed(mu, logstd):
    mu1, logstd1 = map(tf.stop_gradient, [mu, logstd])
    mu2, logstd2 = mu, logstd

    return gauss_KL(mu1, logstd1, mu2, logstd2)

# probability to take action x, given paramaterized guassian distribution
def gauss_log_prob(mu, logstd, x):
    var = tf.exp(2*logstd)
    gp = -tf.square(x - mu)/(2*var) - .5*tf.log(tf.constant(2*np.pi)) - logstd
    return  tf.reduce_sum(gp, [1])

# KL divergence between two paramaterized guassian distributions
def gauss_KL(mu1, logstd1, mu2, logstd2):
    var1 = tf.exp(2*logstd1)
    var2 = tf.exp(2*logstd2)

    kl = tf.reduce_sum(logstd2 - logstd1 + (var1 + tf.square(mu1 - mu2))/(2*var2) - 0.5)
    return kl

# Shannon entropy for a paramaterized guassian distributions
def gauss_ent(mu, logstd):
    h = tf.reduce_sum(logstd + tf.constant(0.5*np.log(2*np.pi*np.e), tf.float32))
    return h

def discount(x, gamma):
    assert x.ndim >= 1
    return scipy.signal.lfilter([1], [1, -gamma], x[::-1], axis=0)[::-1]

def cat_sample(prob_nk):
    assert prob_nk.ndim == 2
    # prob_nk: batchsize x actions
    N = prob_nk.shape[0]
    csprob_nk = np.cumsum(prob_nk, axis=1)
    out = np.zeros(N, dtype='i')
    for (n, csprob_k, r) in zip(xrange(N), csprob_nk, np.random.rand(N)):
        for (k, csprob) in enumerate(csprob_k):
            if csprob > r:
                out[n] = k
                break
    return out

def slice_2d(x, inds0, inds1):
    inds0 = tf.cast(inds0, tf.int64)
    inds1 = tf.cast(inds1, tf.int64)
    shape = tf.cast(tf.shape(x), tf.int64)
    ncols = shape[1]
    x_flat = tf.reshape(x, [-1])
    return tf.gather(x_flat, inds0 * ncols + inds1)

def var_shape(x):
    out = [k.value for k in x.get_shape()]
    assert all(isinstance(a, int) for a in out), \
        "shape function assumes that shape is fully known"
    return out

def numel(x):
    return np.prod(var_shape(x))

def flatgrad(loss, var_list):
    grads = tf.gradients(loss, var_list)
    return tf.concat([tf.reshape(grad, [numel(v)]) for (v, grad) in zip(var_list, grads)], 0)

def conjugate_gradient(f_Ax, b, cg_iters=10, residual_tol=1e-10):
    # in numpy
    p = b.copy()
    r = b.copy()
    x = np.zeros_like(b)
    rdotr = r.dot(r)
    for i in xrange(cg_iters):
        z = f_Ax(p)
        v = rdotr / p.dot(z)
        x += v * p
        r -= v * z
        newrdotr = r.dot(r)
        mu = newrdotr / rdotr
        p = r + mu * p
        rdotr = newrdotr
        if rdotr < residual_tol:
            break
    return x

def linesearch(f, x, fullstep, expected_improve_rate):
    accept_ratio = .1
    max_backtracks = 10
    fval = f(x)
    for (_n_backtracks, stepfrac) in enumerate(.5**np.arange(max_backtracks)):
        xnew = x + stepfrac * fullstep
        newfval = f(xnew)
        actual_improve = fval - newfval
        expected_improve = expected_improve_rate * stepfrac
        ratio = actual_improve / expected_improve
        if ratio > accept_ratio and actual_improve > 0:
            return xnew
    return x

class SetFromFlat(object):

    def __init__(self, session, var_list):
        self.session = session
        assigns = []
        shapes = map(var_shape, var_list)
        total_size = sum(np.prod(shape) for shape in shapes)
        self.theta = theta = tf.placeholder(tf.float32, [total_size])
        start = 0
        assigns = []
        for (shape, v) in zip(shapes, var_list):
            size = np.prod(shape)
            assigns.append(tf.assign(v,tf.reshape(theta[start:start + size],shape)))
            start += size
        self.op = tf.group(*assigns)

    def __call__(self, theta):
        self.session.run(self.op, feed_dict={self.theta: theta})

class GetFlat(object):

    def __init__(self, session, var_list):
        self.session = session
        self.op = tf.concat([tf.reshape(v, [numel(v)]) for v in var_list], 0)

    def __call__(self):
        return self.op.eval(session=self.session)

class GetPolicyWeights(object):
    def __init__(self, session, var_list):
        self.session = session
        self.op = [var for var in var_list if 'policy' in var.name]
    def __call__(self):
        return self.session.run(self.op)

class SetPolicyWeights(object):
    def __init__(self, session, var_list):
        self.session = session
        self.policy_vars = [var for var in var_list if 'policy' in var.name]
        self.placeholders = {}
        self.assigns = []
        for var in self.policy_vars:
            self.placeholders[var.name] = tf.placeholder(tf.float32, var.get_shape())
            self.assigns.append(tf.assign(var,self.placeholders[var.name]))
    def __call__(self, weights):
        feed_dict = {}
        count = 0
        for var in self.policy_vars:
            feed_dict[self.placeholders[var.name]] = weights[count]
            count += 1
        self.session.run(self.assigns, feed_dict)

def xavier_initializer(self, shape):
    dim_sum = np.sum(shape)
    if len(shape) == 1:
        dim_sum += 1
    bound = np.sqrt(6.0 / dim_sum)
    return tf.random_uniform(shape, minval=-bound, maxval=bound)

def fully_connected(input_layer, input_size, output_size, weight_init, bias_init, scope):
    with tf.variable_scope(scope):
        w = tf.get_variable("w", [input_size, output_size], initializer=weight_init)  # this is the weights between 2 layers of NN
        # w = tf.Variable(xavier_initializer([input_size, output_size]), name="w")
        b = tf.get_variable("b", [output_size], initializer=bias_init)
    return tf.matmul(input_layer,w) + b
