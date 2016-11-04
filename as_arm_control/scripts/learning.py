#!/usr/bin/python
#  -*- coding: utf-8 -*-
# author: yao62995@gmail.com

import os
import argparse
import gym
import numpy as np

from DDPG_deep_deterministic_policy_gradient import DDPG
from common import logger
import filter_env


class Experiment(object):
    def __init__(self, env, agent, t_max):
        self.env = env
        self.agent = agent
        self.t_max = t_max
        self.state = None

    def reset(self):
        self.agent.explore_noise.reset()
        return self.env.reset()

    def run_episode(self, test=True, monitor=False):
        # env.monitor.configure(lambda _: test and monitor)
        self.state = self.reset()
        R = 0  # return
        t = 1
        term = False
        while not term:
            # env.render()
            action = self.agent.get_action(self.state, with_noise=not test)
            # action = env.action_space.sample()
            state_n, reward, term, info = self.env.step(action)
            if test:
                term = (t >= 1000) or term
            else:
                term = (t >= self.t_max) or term
            if not test:
                self.agent.feedback(self.state, action, reward, term, state_n)
            self.state = state_n
            t += 1
            R += reward
        return R, t


def run(args):
    # experiment = "InvertedPendulum-v1"
    env = filter_env.makeFilteredEnv(gym.make(args.game))
    print "reward_threshold:", env.spec.reward_threshold, ", timestep_limit:", env.spec.timestep_limit

    save_dir = './result/%s/monitor/' % args.game
    if not os.path.isdir(save_dir):
        os.makedirs(save_dir)
    # env.monitor.start(save_dir, video_callable=lambda _: False, force=True)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    action_range = (env.action_space.low, env.action_space.high)
    print "action space range:", action_range
    train_dir = "./result/%s/tf/" % args.game
    agent = DDPG(state_dim, action_dim, train_dir=train_dir,
                 gpu_id=args.gpu, dim=args.dim)
    t_train, t_test = 0, 0
    experiment = Experiment(env, agent, args.tmax)
    while True:
        # test
        T = t_test
        R = []
        # env.monitor.start(save_dir, video_callable=lambda _: False, resume=True)
        while t_test - T < args.test:
            r, t = experiment.run_episode(test=True, monitor=(len(R) == 0))
            R.append(r)
            t_test += t
        if len(R) > 0:
            avr = sum(R) / len(R)
            logger.info('Average test return\t{} after {} timesteps of training target: ({})'.format(avr, t_train,
                                                                                                 env.spec.reward_threshold))
        # env.monitor.close()
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

        # env.monitor.close()


def parser_argument():
    str2bool = lambda v: v.lower() in ("yes", "true", "t", "1")
    parse = argparse.ArgumentParser()
    parse.add_argument("--game", type=str, help="game name")
    parse.add_argument("--gpu", type=int, default=0, help="gpu number")
    parse.add_argument("--dim", type=int, default=256, help="layer dim")
    parse.add_argument("--train", type=int, default=1e3, help="train time step")
    parse.add_argument("--test", type=int, default=1e3, help="test time step")
    parse.add_argument("--tmax", type=int, default=1e3, help="time step max")
    args = parse.parse_args()
    return args


if __name__ == "__main__":
    run(parser_argument())