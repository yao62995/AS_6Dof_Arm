#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao62995 <yao_62995@163.com>

import sys
import cv2
import random
import numpy as np

import rospy
import tf
from gazebo_msgs.msg import LinkStates, LinkState
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import CompressedImage, JointState
import moveit_commander
from as_arm_description.srv import CheckCollisionValid
from common import logger

DEG_TO_RAD = 0.0174533  # PI/180

# init moveit commander
moveit_commander.roscpp_initialize(sys.argv)
# init robot model
ros_robot = moveit_commander.RobotCommander()
# init robot scene
ros_scene = moveit_commander.PlanningSceneInterface()
# init moveit group
arm_group = ros_robot.get_group("arm")


class ArmJointManager(object):
    ArmJointNames = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint',
                     'finger_joint1', 'finger_joint2']
    MaxGripperJointValue = 0.015

    def __init__(self):
        self.joint_state = JointState()
        self.arm_map = dict()
        self.joint_pub = rospy.Publisher("/as_arm/set_joints_states", JointState, queue_size=1)
        self.pose_sub = rospy.Subscriber("/as_arm/joint_states", JointState, callback=self.callback_joint, queue_size=1)
        # end effector service
        self.check_collision_client = rospy.ServiceProxy('/check_collision', CheckCollisionValid)
        self.tf_listener = tf.TransformListener()

    def callback_joint(self, data):
        if len(self.arm_map) == 0:
            for joint in self.ArmJointNames:
                self.arm_map[joint] = [data.name.index(joint), 0]
        for _, v in self.arm_map.items():
            v[1] = data.position[v[0]]

    def move_joints(self, names, values):
        """
        :param names: a list of joint names (type of string)
        :param values: a list of joint angles (type of int)
        """
        self.joint_state.name = names
        self.joint_state.position = values
        logger.debug("move joints: %s" % str(values))
        self.joint_pub.publish(self.joint_state)

    def move_arm_joints(self, names, values, repeat=1):
        while repeat > 0:
            self.move_joints(names, values)
            repeat -= 1

    def move_gripper_joints(self, value):
        values = [-abs(value), abs(value)]
        self.move_joints(self.ArmJointNames[5:], values)

    def read_joints(self, names):
        """
        :param names: a list of joint names (type of string)
        :return: a list of joint angles (type of float)
        """
        return [self.arm_map[name][1] for name in names]

    def open_gripper(self, repeat=1):
        while repeat > 0:
            self.move_joints(self.ArmJointNames[5:], [0, 0])
            repeat -= 1

    def close_gripper(self, values=None, repeat=1):
        if values is None:
            values = [-self.MaxGripperJointValue, self.MaxGripperJointValue]
        while repeat > 0:
            self.move_joints(self.ArmJointNames[5:], values)
            repeat -= 1

    def read_arm_joints(self, names):
        # return map(lambda x: int(x / DEG_TO_RAD), self.read_joints(names))
        return self.read_joints(names)

    def read_gripper_joints(self):
        return self.read_joints(self.ArmJointNames[5:])

    def read_gripper_frame(self):
        """
        :return: a list of pose, format: [x, y, z, rot_x, rot_y, rot_z]
        """
        (trans, rot) = self.tf_listener.lookupTransform("/world", "/grasp_frame_link", rospy.Time(0))
        ret = list(trans)
        # ret += list(euler_from_quaternion(list(rot)))
        return ret

    def check_collision(self, joint_values):
        """
        :param joint_values: a list of joint values
        :return: true if collision
        """
        joint_values += self.read_gripper_joints()
        return self.check_collision_client(joint_values).valid

    def move_plannar(self, names, source_joints, target_joints):
        source_joints = dict((n, j) for n, j in zip(names, source_joints))
        target_joints = dict((n, j) for n, j in zip(names, target_joints))
        joint_poses = []
        try_time = 0
        while len(joint_poses) == 0 and try_time < 20:
            try:
                arm_group.go(joints=source_joints, wait=True)
                plan_msg = arm_group.plan(joints=target_joints)
                for p in plan_msg.joint_trajectory.points:
                    joint_poses.append(p.positions)
            except:
                joint_poses = []
                try_time += 1
        return joint_poses

    def attach_cube(self, name):
        ros_scene.attach_box("grasp_frame_link", name)

    def remove_attach_cube(self, name):
        ros_scene.remove_attached_object("grasp_frame_link", name)


class CubesManager(object):
    CubeMap = {'cube1': {'init': [-0.18, 0, 0.046]}}

    def __init__(self):
        """
        :param cubes_name: a list of string type of all cubes
        """
        self.cubes_pose = LinkState()
        self.cubes_state = dict()
        # pos publisher
        self.pose_pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)
        self.pose_sub = rospy.Subscriber("/gazebo/cubes", LinkStates, callback=self.callback_state, queue_size=1)

    def reset_cube(self, rand=False):
        logger.info("random set cube object")
        if not rand:
            for k, v in self.CubeMap.items():
                self.set_cube_pose(k, v['init'])
        else:
            for k, v in self.CubeMap.items():
                pose = [-0.17, 0, v["init"][2]]
                pose[0] += - 0.05 + random.random() * 0.06
                pose[1] += - 0.05 + random.random() * 0.1
                self.set_cube_pose(k, pose)

    def callback_state(self, data):
        for idx, cube in enumerate(data.name):
            self.cubes_state.setdefault(cube, [0] * 3)
            pose = self.cubes_state[cube]
            cube_init = self.CubeMap[cube]["init"]
            pose[0] = data.pose[idx].position.x + cube_init[0]
            pose[1] = data.pose[idx].position.y + cube_init[1]
            pose[2] = data.pose[idx].position.z + cube_init[2]

    # def add_cube(self, name):
    #     p = PoseStamped()
    #     p.header.frame_id = ros_robot.get_planning_frame()
    #     p.header.stamp = rospy.Time.now()
    #
    #     # p.pose = self._arm.get_random_pose().pose
    #     p.pose.position.x = -0.18
    #     p.pose.position.y = 0
    #     p.pose.position.z = 0.046
    #
    #     q = quaternion_from_euler(0.0, 0.0, 0.0)
    #     p.pose.orientation = Quaternion(*q)
    #     ros_scene.add_box(name, p, (0.02, 0.02, 0.02))
    #
    # def remove_cube(self, name):
    #     ros_scene.remove_world_object(name)

    def read_cube_pose(self, name=None):
        if name is not None:
            return self.cubes_state[name]
        else:
            return self.cubes_state

    def set_cube_pose(self, name, pose, orient=None):
        """
        :param name: cube name, a string
        :param pose: cube position, a list of three float, [x, y, z]
        :param orient: cube orientation, a list of three float, [ix, iy, iz]
        :return:
        """
        self.cubes_pose.link_name = "cubes::" + name
        p = self.cubes_pose.pose
        cube_init = self.CubeMap[name]["init"]
        p.position.x = pose[0] - cube_init[0]
        p.position.y = pose[1] - cube_init[1]
        p.position.z = pose[2] - cube_init[2]
        if orient is None:
            orient = [0, 0, 0]
        q = quaternion_from_euler(orient[0], orient[1], orient[2])
        p.orientation = Quaternion(*q)
        self.pose_pub.publish(self.cubes_pose)


class CameraListener(object):
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.image = None
        self.image_y = None
        self.camera_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage,
                                           callback=self.callback_camera, queue_size=1)
        self.camera_sub_y = rospy.Subscriber("/camera_y/image_raw/compressed", CompressedImage,
                                             callback=self.callback_camera_y, queue_size=1)

    def callback_camera(self, data):
        # format: rgb8; jpeg compressed bgr8
        np_img = np.fromstring(data.data, dtype=np.uint8)
        img = cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)
        img = cv2.cvtColor(cv2.resize(img, (self.width, self.height)), cv2.COLOR_BGR2GRAY)
        self.image = np.reshape(img, newshape=(self.width, self.height, 1)) / 256.0

    def callback_camera_y(self, data):
        # format: rgb8; jpeg compressed bgr8
        np_img = np.fromstring(data.data, dtype=np.uint8)
        img = cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)
        img = cv2.cvtColor(cv2.resize(img, (self.width, self.height)), cv2.COLOR_BGR2GRAY)
        self.image_y = np.reshape(img, newshape=(self.width, self.height, 1)) / 256.0

    def get_image(self):
        return np.concatenate((self.image, self.image_y), axis=2)
