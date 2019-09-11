from baselines.common.mpi_running_mean_std import RunningMeanStd
import baselines.common.tf_util as U
import tensorflow as tf
import gym
from baselines.common.distributions import make_pdtype
import numpy as np


def apply_activation(x, activation):
    output = x
    if activation == 'relu':
        output = tf.nn.relu(x)
    elif activation == 'tanh':
        output = tf.nn.tanh(x)
    elif activation == 'softmax':
        output = tf.nn.softmax(x)
    elif activation == 'sigmoid':
        output = tf.nn.sigmoid(x)
    elif activation == 'elu':
        output = tf.nn.elu(x)
    elif activation == 'relu6':
        output = tf.nn.relu6(x)
    elif activation == 'crelu':
        output = tf.nn.crelu(x)
    elif activation == 'lrelu':
        output = tf.nn.leaky_relu(x)
    elif activation == 'softplus':
        output = tf.nn.softplus(x)
    elif activation == 'softsign':
        output = tf.nn.softsign(x)
    elif activation is not None and activation != '':
        raise NameError("Invalid activation type ({})".format(activation))
    return output


class MlpPolicy(object):
    def __init__(self, name, *args, **kwargs):
        with tf.variable_scope(name):
            self._init(*args, **kwargs)
            self.scope = tf.get_variable_scope().name

    def _init(self, ob_space, ac_space, hid_size, activation='tanh', interpolate=False):
        assert isinstance(ob_space, gym.spaces.Box)

        self.pdtype = pdtype = make_pdtype(ac_space)
        sequence_length = None

        ob = U.get_placeholder(name="ob", dtype=tf.float32, shape=[sequence_length] + list(ob_space.shape))

        # dropout = U.get_placeholder(name="do", dtype=tf.float32, shape=())

        init_std = 0.5
        regularizer = None  # tf.contrib.layers.l2_regularizer(scale=0.1)

        with tf.variable_scope("obfilter"):
            self.ob_rms = RunningMeanStd(shape=ob_space.shape)

        # obz = tf.clip_by_value((ob - self.ob_rms.mean) / self.ob_rms.std, -25, 25)
        # obz = tf.clip_by_value((ob - self.ob_rms.mean) / self.ob_rms.std, -7.5, 7.5)
        # obz = (ob - self.ob_rms.mean) / self.ob_rms.std
        obz = ob

        with tf.variable_scope('vf'):
            last_out = obz
            for i in range(len(hid_size)):
                last_out = tf.layers.dense(last_out, hid_size[i], name="v_fc%i" % (i + 1)
                                           , kernel_initializer=U.normc_initializer(init_std)
                                           , kernel_regularizer=regularizer)
                # last_out = tf.layers.dropout(last_out, dropout)
                last_out = apply_activation(last_out, activation)
            self.vpred = tf.layers.dense(last_out, 1, name='v_final'
                                         , kernel_initializer=U.normc_initializer(init_std)
                                         , kernel_regularizer=regularizer)[:,0]

        with tf.variable_scope('pol'):
            last_out = obz
            for i in range(len(hid_size)):
                last_out = tf.layers.dense(last_out, hid_size[i], name='p_fc%i' % (i + 1)
                                           , kernel_initializer=U.normc_initializer(init_std)
                                           , kernel_regularizer=regularizer)
                # last_out = tf.layers.dropout(last_out, dropout)
                last_out = apply_activation(last_out, activation)

            mean = tf.layers.dense(last_out, pdtype.param_shape()[0]//2, name='p_final'
                                   , kernel_initializer=U.normc_initializer(init_std)
                                   , kernel_regularizer=regularizer)
            if interpolate:
                mean = apply_activation(mean, 'tanh')
            self.net = mean
            logstd = tf.get_variable(name="logstd", shape=[1, pdtype.param_shape()[0]//2], initializer=tf.zeros_initializer())
            pdparam = tf.concat([mean, mean * 0.0 + logstd], axis=1)

        self.pd = pdtype.pdfromflat(pdparam)

        self.state_in = []
        self.state_out = []

        stochastic = tf.placeholder(dtype=tf.bool, shape=())
        ac = U.switch(stochastic, self.pd.sample(), self.pd.mode())
        self._act = U.function([stochastic, ob], [ac, self.vpred])

    def act_batch(self, stochastic, ob):
        return self._act(stochastic, ob)

    def get_variables(self):
        return tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, self.scope)

    def get_trainable_variables(self):
        return tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, self.scope)

    def get_initial_state(self):
        return []
