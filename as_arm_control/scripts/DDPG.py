#!/usr/bin/env python
#  -*- coding: utf-8 -*-
# author: yao62995@gmail.com

from collections import deque

from common import *


class DDPG(Base):
    """
        Deep Deterministic Policy Gradient Model
        ref:
            paper "continuous control with deep reinforcement learning"
    """

    def __init__(self, states_dim, actions_dim, train_dir="./ddpg_models",
                 observe=1e3, replay_memory=1e4, update_frequency=1, train_repeat=1, gamma=0.99, tau=0.001,
                 batch_size=32, learn_rate=1e-3, dim=256):
        Base.__init__(self)
        self.states_dim = states_dim
        self.actions_dim = actions_dim
        self.image_size = (160, 120)
        # init train params
        self.observe = observe
        self.update_frequency = update_frequency
        self.train_repeat = train_repeat
        self.gamma = gamma
        self.tau = tau
        # init replay memory deque
        self.replay_memory_size = replay_memory
        self.replay_memory = deque()
        # init noise
        self.explore_noise = OUNoise(self.actions_dim)
        # train models dir
        self.train_dir = train_dir
        if not os.path.isdir(self.train_dir):
            os.mkdir(self.train_dir)
        # init network params
        self.learn_rate = learn_rate
        self.batch_size = batch_size
        # tensorflow graph variables
        self.sess = None
        self.saver = None
        self.global_step = None
        self.ops = dict()
        # build graph
        self.build_graph(dim=dim)

    def target_exponential_moving_average(self, theta):
        ema = tf.train.ExponentialMovingAverage(decay=1 - self.tau)
        update = ema.apply(var_list=theta)
        averages = [ema.average(x) for x in theta]
        return averages, update

    def get_variables(self, scope, shape, wd=0.01, val_range=None, collect="losses"):
        with tf.variable_scope(scope):
            if val_range is None:
                val_range = (-10 / np.sqrt(shape[0]), 10 / np.sqrt(shape[0]))
            weights = tf.Variable(tf.random_uniform(shape, val_range[0], val_range[1]), name='weights')
            biases = tf.Variable(tf.random_uniform([shape[-1]], val_range[0], val_range[1]), name='biases')
            if wd is not None:
                weight_decay = tf.mul(tf.nn.l2_loss(weights), wd, name='weight_loss')
                tf.add_to_collection(collect, weight_decay)
            return weights, biases

    def actor_variables(self, scope, wd=0.01, dim=512):
        with tf.variable_scope(scope):
            # joint part
            w1, b1 = self.get_variables("fc1", (self.states_dim, dim), wd=wd, collect=scope)
            w2, b2 = self.get_variables("fc2", (dim, dim), wd=wd, collect=scope)
            # view part
            vw1, vb1 = self.get_variables("v1", shape=[8, 8, 2, 32])
            vw2, vb2 = self.get_variables("v2", shape=[4, 4, 32, 16])
            # concat
            w3, b3 = self.get_variables("fc3", (1536, self.actions_dim * 9), wd=wd, val_range=(-3e-4, 3e-4),
                                        collect=scope)
            return [w1, b1, w2, b2, vw1, vb1, vw2, vb2, w3, b3]

    def critic_variables(self, scope, wd=0.01, dim=512):
        with tf.variable_scope(scope):
            w1, b1 = self.get_variables("fc1", (self.states_dim, dim), wd=wd, collect=scope)
            w2, b2 = self.get_variables("fc2", (dim, dim), wd=wd, collect=scope)
            w3, b3 = self.get_variables("fc3", (dim + self.actions_dim, dim), wd=wd, collect=scope)
            w4, b4 = self.get_variables("fc4", (dim, 1), wd=wd, val_range=(-3e-4, 3e-4), collect=scope)
            return [w1, b1, w2, b2, w3, b3, w4, b4]

    def actor_network(self, op_scope, joint_state, view_state, theta):
        w1, b1, w2, b2, vw1, vb1, vw2, vb2, w3, b3 = theta
        with tf.variable_scope(op_scope, "actor", [joint_state, view_state]) as scope:
            # joint part
            fc1 = tf.nn.relu(tf.matmul(joint_state, w1) + b1)
            fc2 = tf.nn.relu(tf.matmul(fc1, w2) + b2)
            # view part,
            # (160, 120, 2) => (40, 30, 32)
            conv1 = tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(view_state, vw1, strides=[1, 4, 4, 1], padding="SAME"), vb1))
            pool1 = max_pool(conv1)  # (40, 30, 32) => (20, 15, 32)
            # (20, 15, 32) => (10, 8, 16)
            conv2 = tf.nn.relu(tf.nn.bias_add(tf.nn.conv2d(pool1, vw2, strides=[1, 2, 2, 1], padding="SAME"), vb2))
            flat1 = tf.reshape(conv2, [-1, 1280])
            # concat
            concat1 = tf.concat(1, [fc2, flat1], name="concat_state")
            fc3 = tf.matmul(concat1, w3) + b3
            logits = tf.arg_max(tf.reshape(fc3, (-1, self.actions_dim, 9)), dimension=2) - 4
            return logits

    def critic_network(self, op_scope, state, action, theta):
        weight = [theta[idx] for idx in xrange(0, len(theta), 2)]
        bias = [theta[idx] for idx in xrange(1, len(theta), 2)]
        with tf.variable_scope(op_scope, "critic", [state, action]) as scope:
            action = tf.cast(action, tf.float32)
            # reshape
            flat1 = tf.reshape(state, (-1, self.states_dim), name="flat1")
            fc1 = tf.nn.relu(tf.matmul(flat1, weight[0]) + bias[0])
            fc2 = tf.nn.relu(tf.matmul(fc1, weight[1]) + bias[1])
            h_concat = tf.concat(1, [fc2, action])
            fc3 = tf.nn.relu(tf.matmul(h_concat, weight[2]) + bias[2])
            fc4 = tf.matmul(fc3, weight[3]) + bias[3]
            logits = tf.squeeze(fc4, [1], name='out')
            return logits

    def build_graph(self, dim=256):
        with tf.Graph().as_default(), tf.device('/cpu:0'):
            self.global_step = tf.get_variable('global_step', [],
                                               initializer=tf.constant_initializer(0), trainable=False)
            # init variables
            theta_p = self.actor_variables("actor", dim=dim, wd=0.01)
            theta_q = self.critic_variables("critic", dim=dim, wd=0.01)
            theta_pt, update_pt = self.target_exponential_moving_average(theta_p)
            theta_qt, update_qt = self.target_exponential_moving_average(theta_q)
            # actor network
            joint_state = tf.placeholder(tf.float32, shape=(None, self.states_dim))
            view_state = tf.placeholder(tf.float32, shape=(None, 160, 120, 2))
            act_logit = self.actor_network("actor", joint_state, view_state, theta_p)
            cri_logit = self.critic_network("critic", joint_state, act_logit, theta_q)
            # actor optimizer
            l2_loss = tf.add_n(tf.get_collection("actor"))
            p_loss = -tf.reduce_mean(cri_logit) + l2_loss
            opt_p = tf.train.AdamOptimizer(1e-4)
            grad_var_theta_p = opt_p.compute_gradients(p_loss, var_list=theta_p)
            optimizer_p = opt_p.apply_gradients(grad_var_theta_p)
            with tf.control_dependencies([optimizer_p]):
                train_p = tf.group(update_pt)

            # train critic network
            q_target = tf.placeholder(tf.float32, shape=(None), name="critic_target")
            act_train = tf.placeholder(tf.float32, shape=(None, self.actions_dim), name="act_train")
            cri_train = self.critic_network("train_critic", joint_state, act_train, theta_q)
            # target network
            joint_state2 = tf.placeholder(tf.float32, shape=(None, self.states_dim))
            view_state2 = tf.placeholder(tf.float32, shape=(None, 160, 120, 2))
            act_logit2 = self.actor_network("target_actor", joint_state2, view_state2, theta_pt)
            cri_logit2 = self.critic_network("target_critic", joint_state2, act_logit2, theta_qt)
            # train critic optimizer
            l2_loss = tf.add_n(tf.get_collection("critic"))
            q_loss = tf.reduce_mean(tf.square(cri_train - q_target)) + l2_loss
            opt_q = tf.train.AdamOptimizer(1e-3)
            grad_var_theta_q = opt_q.compute_gradients(q_loss, var_list=theta_q)
            optimizer_q = opt_q.apply_gradients(grad_var_theta_q, global_step=self.global_step)
            with tf.control_dependencies([optimizer_q]):
                train_q = tf.group(update_qt)

            # init session and saver
            self.saver = tf.train.Saver()
            self.sess = tf.Session(config=tf.ConfigProto(
                allow_soft_placement=True,
                log_device_placement=False)
            )
            self.sess.run(tf.initialize_all_variables())
        # restore model
        restore_model(self.sess, self.train_dir, self.saver)
        self.ops["act_logit"] = lambda joint_obs, view_obs: \
            self.sess.run(act_logit, feed_dict={joint_state: joint_obs, view_state: view_obs})
        self.ops["cri_logit2"] = lambda joint_obs, view_obs: \
            self.sess.run(cri_logit2, feed_dict={joint_state2: joint_obs, view_state2: view_obs})
        self.ops["train_p"] = lambda joint_obs, view_obs: \
            self.sess.run([train_p, p_loss], feed_dict={joint_state: joint_obs, view_state: view_obs})
        self.ops["train_q"] = lambda joint_obs, act, q_t: \
            self.sess.run([train_q, self.global_step, q_loss], feed_dict={joint_state: joint_obs, act_train: act,
                                                                          q_target: q_t})

    def get_action(self, state, with_noise=False):
        joint_state, view_state = state
        action = self.ops["act_logit"]([joint_state], [view_state])[0]
        if with_noise:
            action = action + self.explore_noise.noise()
        action = np.clip(action, -4, 4).astype(int)
        return action

    def feedback(self, state, action, reward, terminal, state_n):
        self.time_step += 1
        self.replay_memory.append((state, action, reward, terminal, state_n))
        if len(self.replay_memory) > self.replay_memory_size:
            self.replay_memory.popleft()
        if self.time_step > self.observe and self.time_step % self.update_frequency == 0:
            for _ in xrange(self.train_repeat):
                # train mini-batch from replay memory
                mini_batch = random.sample(self.replay_memory, self.batch_size)
                batch_joint_state, batch_view_state, batch_action = [], [], []
                batch_target_q = []
                for batch_i, sample in enumerate(mini_batch):
                    b_state, b_action, b_reward, b_terminal, b_state_n = sample
                    joint_state, view_state = b_state
                    joint_state_n, view_state_n = b_state_n
                    if b_terminal:
                        target_q = b_reward
                    else:  # compute target q values
                        target_q = b_reward + self.gamma * \
                                              self.ops["cri_logit2"]([joint_state_n], [view_state_n])[0]
                    batch_joint_state.append(joint_state)
                    batch_view_state.append(view_state)
                    batch_action.append(b_action)
                    batch_target_q.append(target_q)
                # update actor network (theta_p)
                _, p_loss = self.ops["train_p"](batch_joint_state, batch_view_state)
                # update critic network (theta_q)
                _, global_step, q_loss = self.ops["train_q"](batch_joint_state, batch_action, batch_target_q)
                if self.time_step % 1e3 == 0:
                    # logger.info("step=%d, p_loss=%.6f, q_loss=%.6f" % (global_step, p_loss, q_loss))
                    logger.info("step=%d, q_loss=%.6f" % (global_step, q_loss))
        if self.time_step % 3e4 == 0:
            save_model(self.sess, self.train_dir, self.saver, "ddpg-", global_step=self.global_step)
