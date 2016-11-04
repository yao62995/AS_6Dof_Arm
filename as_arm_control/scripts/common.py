#!/usr/bin/python
#  -*- coding: utf-8 -*-

import os
import time
import sys
import abc
import random

import numpy as np

import tensorflow as tf


class NetTools(object):
    @staticmethod
    def get_variable(name, shape, initializer):
        var = tf.get_variable(name, shape, initializer=initializer)
        return var

    @staticmethod
    def variable_with_weight_decay(name, shape, val_range, wd, collect="losses"):
        var = NetTools.get_variable(name, shape,
                                    tf.random_uniform_initializer(minval=val_range[0], maxval=val_range[1]))
        if wd is not None:
            weight_decay = tf.mul(tf.nn.l2_loss(var), wd, name='weight_loss')
            tf.add_to_collection(collect, weight_decay)
        return var

    @staticmethod
    def conv2d(x, kernel_shape, variable_scope, stride=1, val_range=None, padding="SAME",
               with_param=False, weight_decay=None, collect="losses"):
        if val_range is None:
            min_val = -1.0 / float(np.sqrt(np.prod(kernel_shape[:-1])))
            val_range = (min_val, -min_val)
        with tf.variable_scope(variable_scope) as scope:
            kernel = NetTools.variable_with_weight_decay('weights', shape=kernel_shape,
                                                         val_range=val_range, wd=weight_decay, collect=collect)
            conv = tf.nn.conv2d(x, kernel, strides=[1, stride, stride, 1], padding=padding)
            biases = NetTools.get_variable('biases', [kernel_shape[-1]], tf.constant_initializer(0.0))
            bias = tf.nn.bias_add(conv, biases)
            conv = tf.nn.relu(bias, name=scope.name)
        if with_param:
            return conv, kernel, biases
        else:
            return conv

    @staticmethod
    def max_pool(x, ksize=2, stride=2, padding="SAME", name=None):
        return tf.nn.max_pool(x, ksize=[1, ksize, ksize, 1], strides=[1, stride, stride, 1], padding=padding, name=name)

    @staticmethod
    def avg_pool(x, ksize=2, stride=2, padding="SAME", name=None):
        return tf.nn.avg_pool(x, ksize=[1, ksize, ksize, 1], strides=[1, stride, stride, 1], padding=padding, name=name)

    @staticmethod
    def batch_normalized(x, mean=0.0, var=1.0, offset=None, scale=None, epsilon=1e-10, name=None):
        """
            error implement, please ref: tensorflow-Inception codes in
             "https://github.com/tensorflow/models/blob/master/inception/inception/slim/ops.py"
        """
        # TODO: solve bug here
        return tf.nn.batch_normalization(x, mean=mean, variance=var, offset=offset, scale=scale,
                                         variance_epsilon=epsilon, name=name)

    @staticmethod
    def full_connect(x, W_shape, variable_scope, val_range=None, activate="relu",
                     weight_decay=None, collect="losses", with_param=False, with_bias=True):
        if val_range is None:
            min_val = -1.0 / float(np.sqrt(np.prod(W_shape[:-1])))
            val_range = (min_val, -min_val)
        with tf.variable_scope(variable_scope) as scope:
            weights = NetTools.variable_with_weight_decay('weights', shape=W_shape,
                                                          val_range=val_range, wd=weight_decay, collect=collect)
            if with_bias:
                biases = NetTools.get_variable('biases', [W_shape[-1]], tf.constant_initializer(0.1))
                fc = tf.matmul(x, weights) + biases
            else:
                fc = tf.matmul(x, weights)
            if activate is not None:
                fc = tf.nn.relu(fc, name=scope.name)
        if with_param and with_bias:
            return fc, weights, biases
        if with_param and not with_bias:
            return fc, weights
        else:
            return fc

    @staticmethod
    def restore_model(sess, model_dir, saver, model_file=None):
        if model_file is not None:
            model_file_path = "%s/%s" % (model_dir, model_file)
            saver.restore(sess, model_file_path)
            print("Successfully loaded:", model_file_path)
        else:
            checkpoint = tf.train.get_checkpoint_state(model_dir)
            if checkpoint and checkpoint.model_checkpoint_path:
                saver.restore(sess, checkpoint.model_checkpoint_path)
                print("Successfully loaded:", checkpoint.model_checkpoint_path)
            else:
                print("Could not find old network weights")

    @staticmethod
    def save_model(sess, model_dir, saver, prefix, global_step=None):
        checkpoint_filename = saver.save(sess, model_dir + "/" + prefix, global_step=global_step)
        return checkpoint_filename

    @staticmethod
    def grad_histograms(grads_and_vars):
        s = []
        for grad, var in grads_and_vars:
            s.append(tf.histogram_summary(var.op.name + '', var))
            s.append(tf.histogram_summary(var.op.name + '/gradients', grad))
        return tf.merge_summary(s)


class Environment(object):
    def __init__(self):
        self.action = 3

    def reset(self):
        Environment.__init__(self)

    def get_state(self):
        state = None
        return state

    def random_action(self):
        return random.randint(0, self.action - 1)

    def terminal(self):
        # is game over
        return False

    def step_forward(self, action):
        # move forward
        reward = 0
        state = self.get_state()
        is_over = self.terminal()
        return state, reward, is_over


class OUNoise:
    """
        add noise to array list
        ref: https://github.com/rllab/rllab/blob/master/rllab/exploration_strategies/ou_strategy.py
    """

    def __init__(self, action_dimension, mu=0, theta=0.15, sigma=0.3):
        self.action_dimension = action_dimension
        self.mu = mu
        self.theta = theta
        self.sigma = sigma
        self.state = np.ones(self.action_dimension) * self.mu
        self.reset()

    def reset(self):
        self.state = np.ones(self.action_dimension) * self.mu

    def noise(self):
        x = self.state
        dx = self.theta * (self.mu - x) + self.sigma * np.random.randn(len(x))
        self.state = x + dx
        return self.state


class Base(object):
    __metaclass__ = abc.ABCMeta

    def __init__(self):
        self.time_step = 0

    @abc.abstractmethod
    def get_action(self, state):
        """
        :param state:
        :return: action of state
        """
        return

    @abc.abstractmethod
    def feedback(self, state, action, reward, terminal, state_n):
        """
        :param state:
        :param action:
        :param reward:
        :param terminal:
        :param state_n:
        :return: None
        """
        return


class Logger(object):
    def __init__(self, log_dir, debug=False):
        self._log_dir = log_dir
        if not os.path.exists(self._log_dir):
            os.makedirs(self._log_dir)
        self._debug = debug
        self.DATE_FORMAT = "%Y-%m-%d"
        self.DATETIME_FORMAT = "%Y-%m-%d %H:%M:%S"
        self._log_date = self._curdate()
        self._logfile = "%s/%s.log" % (self._log_dir, self._log_date)
        self._logger = open(self._logfile, 'a+')

    def _curdate(self):
        return time.strftime(self.DATE_FORMAT, time.localtime())

    def _curdatetime(self):
        return time.strftime(self.DATETIME_FORMAT, time.localtime())

    def _switch_log(self):
        if self._log_date != self._curdate():  # create new logfile
            # close old logfile
            self._logger.close()
            # make new log file
            self._log_date = self._curdate()
            self._logfile = "%s/%s.log" % (self._log_dir, self._log_date)
            self._logger = open(self._logfile, 'a+')

    def _writer(self, msg):
        self._switch_log()
        # maybe locker is needed here
        self._logger.write("%s\n" % msg)
        self._logger.flush()

    def debug(self, msg):
        if self._debug:
            msg = "%s [DEBUG] %s" % (self._curdatetime(), msg)
            self._writer(msg)

    def info(self, msg):
        msg = "%s [INFO] %s" % (self._curdatetime(), msg)
        print msg
        self._writer(msg)

    def warn(self, msg):
        msg = "%s [WARN] %s" % (self._curdatetime(), msg)
        print msg
        self._writer(msg)

    def error(self, msg, to_exit=False):
        msg = "%s [ERROR] %s" % (self._curdatetime(), msg)
        print msg
        self._writer(msg)
        if to_exit:
            sys.exit(-1)


# define logger
logger = Logger("./log")
# define network tools
conv2d = NetTools.conv2d
max_pool = NetTools.max_pool
avg_pool = NetTools.avg_pool
full_connect = NetTools.full_connect
restore_model = NetTools.restore_model
save_model = NetTools.save_model