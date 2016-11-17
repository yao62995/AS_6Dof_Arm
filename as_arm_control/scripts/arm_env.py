#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995<yao_62995@163.com>

import random
import sys
import copy
from math import sqrt, pow, exp

import rospy

from common import Environment, logger
from simulate_state import ArmJointManager, CubesManager, CameraListener, DEG_TO_RAD


class ArmEnv(Environment):
    JointMap = {'base_joint': {"idx": 0, 'bias': 90, 'range': [-90, 90], 'init': 0},
                'shoulder_joint': {"idx": 1, 'bias': 95, 'range': [-85, 85], 'init': 0.2834},
                'elbow_joint': {"idx": 2, 'bias': 90, 'range': [-60, 90], 'init': -0.9736},
                'wrist_flex_joint': {"idx": 3, 'bias': 90, 'range': [-90, 0], 'init': -1.4648},
                'wrist_rot_joint': {"idx": 4, 'bias': 90, 'range': [-90, 0], 'init': 0},
                'finger_joint1': {"idx": 5, 'bias': 90, 'range': [-90, 0], 'init': 0},
                'finger_joint2': {"idx": 5, 'bias': 90, 'range': [0, 90], 'init': 0}
                }
    CubeMap = {'cube1': {'init': [-0.18, 0, 0.05]}}
    # pose error threshold
    PoseErrorThreshold = 1e-2

    def __init__(self, image_shape, max_move_step=200, gamma=0.99):
        self.gamma = gamma
        # init ros node
        rospy.init_node('as_arm_move_group', anonymous=True)
        # set arm joint manager
        self.joint_mgr = ArmJointManager()
        # set cube manager
        self.cube_mgr = CubesManager()
        # set camera
        img_width, img_height = image_shape
        self.camera = CameraListener(img_width, img_height)
        # wait for first callback of subscribers
        rospy.sleep(2)
        # init joint name
        self.joint_names = ArmJointManager.ArmJointNames
        # target pose, where cube to be placed, format: [x, y, z, rot_x, rot_y, rot_z]
        self.target_pose = [0, -0.18, 0.05, 0, 0, 0]
        # self.target_pose = [-0.18, 0, 0.05, 0, 0, 0]
        # set move counter
        self.move_counter = 0
        # set max move step
        self.max_step_step = max_move_step
        # record whether game is over
        self.terminal = False
        # check whether cube grasped
        self.cube_grasped = False
        # init set JointMap Angles to DEG format
        print "======joint range:"
        for name in self.joint_names:
            v = self.JointMap[name]
            v['range'][0] *= DEG_TO_RAD
            v['range'][1] *= DEG_TO_RAD
            print '\t', name, v['range']

    def __del__(self):
        self.cube_mgr.remove_cube("Grasp_Object")

    def reset(self):
        # reset arm joints
        joint_values = [self.JointMap[name]['init'] for name in self.joint_names[:5]]
        self.joint_mgr.move_arm_joints(self.joint_names[:5], joint_values)
        # close gripper
        self.joint_mgr.open_gripper()
        # reset cube position
        for k, v in self.CubeMap.items():
            self.cube_mgr.set_cube_pose(k, v['init'])
        if self.cube_grasped:
            self.joint_mgr.remove_attach_cube("Grasp_Object")
        # reset cube
        self.cube_mgr.reset_cube()
        rospy.sleep(5.0)
        # reset move counter
        self.move_counter = 0
        self.terminal = False
        self.cube_grasped = False
        # return state
        return self.get_state()

    def get_state(self):
        # angles of five arm joints
        joints = self.joint_mgr.read_joints(self.joint_names[:5])
        # camera images
        image = self.camera.get_image()
        return joints, image

    def filter_joints(self, names, values):
        # control joints value within proper range
        for name, value in zip(names, values):
            _index = self.JointMap[name]['idx']
            _range = self.JointMap[name]['range']
            if value > _range[1]:
                values[_index] = _range[1]
            elif value < _range[0]:
                values[_index] = _range[0]
            # if name == "finger_joint1":
            #     values[_index] = - abs(values[_index] / (_range[1] - _range[0]) * 0.015)
        return values

    def random_action(self):
        joint_angles = self.joint_mgr.read_arm_joints(self.joint_names[:5])
        logger.debug("random joint: %s" % str(joint_angles))
        joint_values = map(lambda x: (x + random.randint(-5, 5)) * DEG_TO_RAD, joint_angles)
        return joint_values

    def distance(self, pos1, pos2, size=3):
        return sqrt(sum(map(lambda x: pow(pos1[x] - pos2[x], 2), range(size))))

    def check_cube_in_sink(self):
        # cube range
        p = self.cube_mgr.read_cube_pose("cube1")
        if p[2] > 0.015:  # position z
            return False
        x, y = p[0], p[1]
        if x < 0.7 or x > 0.27 or abs(y) > 0.1:  # when out of sink
            return False
        return True

    def check_cube_in_gripper(self):
        pc = self.cube_mgr.read_cube_pose("cube1")
        pg = self.joint_mgr.read_gripper_frame()
        if self.distance(pc, pg) < self.PoseErrorThreshold:  # grasped
            logger.info("success pickup cube in step[%d]" % self.move_counter)
            self.joint_mgr.attach_cube("Grasp_Object")
            return True
        return False

    def check_terminal(self):
        # check move count
        self.move_counter += 1
        if self.move_counter >= self.max_step_step:  # out of step limit
            return True
        # check whether cube in gripper
        if not self.cube_grasped:
            return False
        # check whether reach goal
        arm_pose = self.joint_mgr.read_gripper_frame()
        if self.distance(self.target_pose, arm_pose, size=3) < self.PoseErrorThreshold:  # reach goal
            self.joint_mgr.open_gripper()
            self.joint_mgr.remove_attach_cube("Grasp_Object")
            logger.info("success place cube in step[%d]" % self.move_counter)
            return True
        return False

    def get_reward(self):
        if self.terminal:  # when terminal
            if self.move_counter < self.max_step_step:  # when arm reach goal
                return 100
            else:
                return -1
        if not self.cube_grasped:  # when not grasped
            if self.check_cube_in_gripper():  # when cube just grasped
                self.cube_grasped = True
                self.joint_mgr.close_gripper([-0.05, 0.05])
                return 100
            else:  # rewarded by distance between gripper and cube
                arm_pose = self.joint_mgr.read_gripper_frame()
                cube_pose = self.cube_mgr.read_cube_pose("cube1")
                dist = self.distance(arm_pose, cube_pose, size=3)
                # print "distance:", dist, ", arm:", arm_pose[:3], ", cube:", cube_pose[:3]
                return - exp(-1 * self.gamma * dist)
        else:  # rewarded by distance between gripper and target pose
            arm_pose = self.joint_mgr.read_gripper_frame()
            dist = self.distance(arm_pose, self.target_pose, size=3)
            return - exp(-1 * self.gamma * dist)

    def check_collision(self, action):
        valid_action = copy.deepcopy(action)
        try_time = 0
        status = 0
        while self.joint_mgr.check_collision(valid_action):
            valid_action = map(lambda x: x + random.randint(-5, 5) * DEG_TO_RAD, action)
            valid_action = self.filter_joints(self.joint_names[:5], valid_action)
            try_time += 1
            if try_time > 100:  # attemp too much time, recover init position
                valid_action = [self.JointMap[name]['init'] for name in self.joint_names[:5]]
                status = -1
                break
        return valid_action, status

    def step_forward(self, action):
        """
        :param action:  a list of joint values, (float type)
        """
        values = self.joint_mgr.read_joints(self.joint_names[:5])
        action = [float((a - 1) * 2 * DEG_TO_RAD + v) for a, v in zip(action, values)]
        # filter joint values
        action = self.filter_joints(self.joint_names[:5], action)
        # check collision
        action, status = self.check_collision(action)
        self.joint_mgr.move_arm_joints(self.joint_names[:5], action[:5])
        # set cube pose when grasped
        if self.cube_grasped:
            self.cube_mgr.set_cube_pose("cube1", self.joint_mgr.read_gripper_frame())
        if status == -1:  # move to init position need some time
            rospy.sleep(3)
        else:
            rospy.sleep(0.1)
        # print "joints:", self.joint_mgr.read_joints(self.joint_names)
        # check terminal
        self.terminal = self.check_terminal()
        # get reward
        reward = self.get_reward()
        state = self.get_state()
        return state, reward, self.terminal
