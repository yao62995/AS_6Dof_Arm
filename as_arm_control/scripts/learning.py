#!/usr/bin/python
#  -*- coding: utf-8 -*-
# author: yao62995@gmail.com

import os
import sys
import signal
import argparse

from ddpg import DDPG
from arm_env import ArmEnv
from common import logger

is_exit = False


class Experiment(object):
    def __init__(self, env, agent, t_max):
        self.env = env
        self.agent = agent
        self.t_max = t_max
        self.state = None

    def reset(self):
        self.agent.explore_noise.reset()
        return self.env.reset()

    def run_episode(self, test=True):
        self.state = self.reset()
        R = 0  # return
        t = 1
        term = False
        print "========episode start"
        while not term:
            action = self.agent.get_action(self.state, with_noise=not test)
            # action = self.env.random_action()
            state_n, reward, term = self.env.step_forward(action)
            if test:
                term = (t >= 200) or term
            else:
                term = (t >= self.t_max) or term
            if term:
                print "==========episode end"
            if not test:
                self.agent.feedback(self.state, action, reward, term, state_n)
            self.state = state_n
            t += 1
            R += reward
            # print "step:", t, ", reward:", reward, ", total_reward:", R
            if is_exit:
                del self.env
                sys.exit(-1)
        return R, t


def create_dir(dir_name):
    if not os.path.isdir(dir_name):
        os.makedirs(dir_name)
    return dir_name


def signal_handler(signum, frame):
    global is_exit
    is_exit = True
    print "receive a signal %d, is_exit = %d" % (signum, is_exit)


def run(args):
    state_dim = 5
    action_dim = 6
    train_dir = create_dir('./result')
    agent = DDPG(state_dim, action_dim, train_dir=train_dir, gamma=args.gamma)
    agent.explore_noise.theta = 1.0
    agent.explore_noise.sigma = 2.0
    env = ArmEnv(image_shape=agent.image_size, max_move_step=args.tmax, gamma=args.gamma)

    t_train, t_test = 0, 0
    experiment = Experiment(env, agent, args.tmax)
    while True:
        # test
        T = t_test
        R = []
        while t_test - T < args.test:
            r, t = experiment.run_episode(test=True)
            R.append(r)
            t_test += t
        if len(R) > 0:
            avr = sum(R) / len(R)
            logger.info('Average test return\t{} after {} timesteps of training.'.format(avr, t_train))
        # train
        T = t_train
        R = []
        while t_train - T < args.train:
            r, t = experiment.run_episode(test=False)
            R.append(r)
            t_train += t
        if len(R) > 0:
            avr = sum(R) / len(R)
            logger.info('Average train return\t{} after {} timesteps of training'.format(avr, t_train))


def parser_argument():
    parse = argparse.ArgumentParser()
    parse.add_argument("--train", type=int, default=5000, help="train time step")
    parse.add_argument("--test", type=int, default=400, help="test time step")
    parse.add_argument("--tmax", type=int, default=200, help="time step max")
    parse.add_argument("--gamma", type=float, default=0.99, help="gamma")
    args = parse.parse_args()
    return args


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    run(parser_argument())
