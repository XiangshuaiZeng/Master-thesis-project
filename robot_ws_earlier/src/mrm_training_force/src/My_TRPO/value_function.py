import tensorflow as tf
import numpy as np
from utils import *

class VF(object):
    coeffs = None

    def __init__(self, session):
        self.net = None
        self.session = session

    def create_net(self, shape):
        hidden_size1 = 100
        hidden_size2 = 50
        hidden_size3 = 50
        self.x = tf.placeholder(tf.float32, shape=[None, shape], name="x")
        self.y = tf.placeholder(tf.float32, shape=[None], name="y")

        weight_init = tf.random_uniform_initializer(-0.02, 0.02)
        bias_init = tf.constant_initializer(0)

        with tf.variable_scope("VF"):
            h1 = tf.nn.tanh(fully_connected(self.x, shape, hidden_size1, weight_init, bias_init, "h1"))
            h2 = tf.nn.tanh(fully_connected(h1, hidden_size1, hidden_size2, weight_init, bias_init, "h2"))
            h3 = tf.nntanh(fully_connected(h2, hidden_size2, hidden_size3, weight_init, bias_init, "h3"))
            h4 = fully_connected(h3, hidden_size3, 1, weight_init, bias_init, "h4")
        self.net = tf.reshape(h4, (-1,))
        l2 = tf.nn.l2_loss(self.net - self.y)
        self.train = tf.train.AdamOptimizer().minimize(l2)
        self.session.run(tf.global_variables_initializer())

    def _features(self, path):
        o = path["obs"].astype('float32')
        o = o.reshape(o.shape[0], -1)
        act = path["action_dists_mu"].astype('float32')
        l = len(path["rewards"])
        al = np.arange(l).reshape(-1, 1) / 10.0
        # ret = np.concatenate([o, act, al, np.ones((l, 1))], axis=1)   # should the input be only the observation since it is value function?
        ret = np.concatenate([o, np.ones((l, 1))], axis=1)
        return ret

    def fit(self, paths):
        featmat = np.concatenate([self._features(path) for path in paths])
        if self.net is None:
            self.create_net(featmat.shape[1])
        returns = np.concatenate([path["returns"] for path in paths])
        for _ in range(50):
            self.session.run(self.train, {self.x: featmat, self.y: returns})

    def predict(self, path):
        if self.net is None:
            return np.zeros(len(path["rewards"]))
        else:
            ret = self.session.run(self.net, {self.x: self._features(path)})
            return np.reshape(ret, (ret.shape[0], ))


class LinearVF(object):
    coeffs = None

    def _features(self, path):
        o = path["obs"].astype('float32')
        o = o.reshape(o.shape[0], -1)
        l = len(path["rewards"])
        al = np.arange(l).reshape(-1, 1) / 100.0
        return np.concatenate([o, o**2, al, al**2, np.ones((l, 1))], axis=1)

    def fit(self, paths):
        featmat = np.concatenate([self._features(path) for path in paths])
        returns = np.concatenate([path["returns"] for path in paths])
        n_col = featmat.shape[1]
        lamb = 2.0
        self.coeffs = np.linalg.lstsq(featmat.T.dot(featmat) + lamb * np.identity(n_col), featmat.T.dot(returns))[0]

    def predict(self, path):
        return np.zeros(len(path["rewards"])) if self.coeffs is None else self._features(
            path).dot(self.coeffs)
