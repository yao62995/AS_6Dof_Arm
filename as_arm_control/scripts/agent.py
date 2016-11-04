#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao_62995@163.com

import random
import sys
from math import sqrt, pow

import rospy
import moveit_commander

from common import Environment
from simulate_state import ArmJointManager, CubesManager, CameraListener, DEG_TO_RAD


class ArmAgent(Environment):
    JointMap = {'base_joint': {"idx": 0, 'bias': 90, 'range': (-90, 90), 'init': 0},
                'shoulder_joint': {"idx": 1, 'bias': 95, 'range': (-85, 85), 'init': -0.9736},
                'elbow_joint': {"idx": 2, 'bias': 90, 'range': (-60, 90), 'init': 0.2834},
                'wrist_flex_joint': {"idx": 3, 'bias': 90, 'range': (-90, 0), 'init': -1.4648},
                'wrist_rot_joint': {"idx": 4, 'bias': 90, 'range': (-90, 0), 'init': 0},
                'finger_joint1': {"idx": 5, 'bias': 90, 'range': (-90, 0), 'init': 0}
                }
    CubeMap = {'cube1': {'init': [-0.18, 0, 0.02]}}
    # pose error threshold
    PoseErrorThreshold = 1e-4
    # max move steps
    MaxMoveStep = 200

    def __init__(self):
        # init moveit interface
        moveit_commander.roscpp_initialize(sys.argv)
        # init ros node
        rospy.init_node('as_arm_move_group', anonymous=True)
        # get moveit plan group
        arm_group = moveit_commander.MoveGroupCommander("arm")
        gripper_group = moveit_commander.MoveGroupCommander("gripper")
        # set arm joint manager
        self.joint_mgr = ArmJointManager(arm_group, gripper_group)
        # set cube manager
        self.cube_mgr = CubesManager()
        # set camera
        self.camera = CameraListener(640, 480)
        # init joint name
        self.joint_names = ['base_joint', 'elbow_joint', 'shoulder_joint', 'wrist_flex_joint', 'wrist_rot_joint',
                            'finger_joint1']
        # target pose, where cube to be placed, format: [x, y, z, rot_x, rot_y, rot_z]
        # self.target_pose = [0, 0.2, 0.1, 0, 0, 0]
        self.target_pose = [-0.18, 0, 0.02, 0, 0, 0]
        # set move counter
        self.move_counter = 0
        # record whether game is over
        self.terminal = False
        # check whether cube grasped
        self.cube_grasped = False
        # init set JointMap Angles to DEG format
        for _, v in self.JointMap.items():
            v['range'][0] *= DEG_TO_RAD
            v['range'][1] *= DEG_TO_RAD

    def reset(self):
        # reset arm joints
        joint_values = [self.JointMap[name]['init'] for name in self.joint_names[:5]]
        self.joint_mgr.move_arm_joints(self.joint_names[:5], joint_values)
        # close gripper
        self.joint_mgr.close_gripper()
        # reset cube position
        for k, v in self.CubeMap.items():
            self.cube_mgr.set_cube_pose(k, v['init'])
        # reset move counter
        self.move_counter = 0
        self.terminal = False
        self.cube_grasped = False

    def get_state(self):
        # angles of five arm joints
        joints = self.joint_mgr.read_joints(self.joint_names)
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
        return values

    def random_action(self):
        joint_angles = self.joint_mgr.read_arm_joints(self.joint_names)
        joint_values = map(lambda x: (x + random.randint(-3, 3)) * DEG_TO_RAD, joint_angles)
        self.step_forward(joint_values)

    def distance(self, pos1, pos2):
        return sqrt(sum(map(lambda x: pow(pos1[x] - pos2[x], 2), range(6))))

    def check_cube_in_sink(self):
        # cube range
        p = self.cube_mgr.get_cube_pose("cube1")
        if p.position.z > 0.015:
            return False
        x, y = p.position.x, p.position.y
        if x < 0.7 or x > 0.27 or abs(y) > 0.1:  # when out of sink
            return False
        return True

    def check_cube_in_gripper(self):
        p = self.cube_mgr.get_cube_pose("cube1")
        if p.position.z > 0.015:
            return True
        return False

    def check_terminal(self):
        # check move count
        self.move_counter += 1
        if self.move_counter >= self.MaxMoveStep:  # out of step limit
            return True
        # check whether cube in gripper
        if not self.check_cube_in_gripper():
            return False
        # check whether reach goal
        arm_pose = self.joint_mgr.read_gripper_frame()
        if self.distance(self.target_pose, arm_pose) < self.PoseErrorThreshold:  # reach goal
            return True
        return False

    def get_reward(self):
        if self.terminal and self.move_counter < self.MaxMoveStep:  # when arm reach goal
            return 200
        if not self.cube_grasped and self.check_cube_in_gripper():  # when cube just grasped
            self.cube_grasped = True
            return



    def step_forward(self, action):
        """
        :param action:  a list of joint values, (float type)
        """
        # filter joint values
        action = self.filter_joints(self.joint_names, action)
        # execute arm joints
        self.joint_mgr.move_arm_joints(self.joint_names[:5], action[:5])
        # execute gripper joints
        self.joint_mgr.move_gripper_joints(action[5])
        # check terminal
        self.terminal = self.check_terminal()
        # get reward
        reward = self.get_reward()
        # get state
        image, joints = self.get_state()
        return image, joints, reward, self.terminal
