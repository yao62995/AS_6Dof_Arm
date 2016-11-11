#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: yao_62995@163.com

import sys
import cv2
import numpy as np

import rospy
import tf
from gazebo_msgs.msg import LinkStates, LinkState
from gazebo_msgs.srv import GetLinkState
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import CompressedImage, JointState
from as_arm_description.srv import CheckCollisionValid

DEG_TO_RAD = 0.0174533  # PI/180


class ArmJointManager(object):
    ArmJointNames = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_flex_joint', 'wrist_rot_joint',
                     'finger_joint1', 'finger_joint2']
    MaxGripperJointValue = 0.015

    def __init__(self):
        # init moveit commander
        # moveit_commander.roscpp_initialize(sys.argv)
        # init robot model
        # robot = moveit_commander.RobotCommander()
        # init move group
        # self.arm_group = moveit_commander.MoveGroupCommander("arm")

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
        # print "move joints:", values
        self.joint_pub.publish(self.joint_state)

    def move_arm_joints(self, names, values):
        self.move_joints(names, values)

    def move_gripper_joints(self, value):
        values = [-abs(value), abs(value)]
        self.move_joints(self.ArmJointNames[5:], values)

    def read_joints(self, names):
        """
        :param names: a list of joint names (type of string)
        :return: a list of joint angles (type of float)
        """
        return [self.arm_map[name][1] for name in names]

    def open_gripper(self):
        values = [-self.MaxGripperJointValue, self.MaxGripperJointValue]
        return self.move_joints(self.ArmJointNames[5:], values)

    def close_gripper(self):
        return self.move_joints(self.ArmJointNames[5:], [0] * 2)

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
        ret += list(euler_from_quaternion(list(rot)))
        return ret

    def check_collision(self, joint_values):
        """
        :param joint_values: a list of joint values
        :return: true if collision
        """
        if len(joint_values) == 6:
            joint_values = joint_values + [-joint_values[5]]
        return self.check_collision_client(joint_values).valid



class CubesManager(object):
    def __init__(self):
        """
        :param cubes_name: a list of string type of all cubes
        """
        self.cubes_pose = LinkState()
        self.cubes_state = dict()
        # pos publisher
        self.pose_pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=1)
        self.pose_sub = rospy.Subscriber("/gazebo/cubes", LinkStates, callback=self.callback_state, queue_size=1)
        # set base_link to (0,0,0)

    def callback_state(self, data):
        for idx, link in enumerate(data.name):
            self.cubes_state.setdefault(link, [0] * 3)
            pose = self.cubes_state[link]
            pose[0] = data.pose[idx].position.x - 0.18
            pose[1] = data.pose[idx].position.y
            pose[2] = data.pose[idx].position.z + 0.05

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
        p.position.x = pose[0] + 0.18
        p.position.y = pose[1]
        p.position.z = pose[2] - 0.05
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
        self.camera_sub = rospy.Subscriber("/camera/image_raw/compressed", CompressedImage,
                                           callback=self.callback_camera, queue_size=1)

    def callback_camera(self, data):
        # format: rgb8; jpeg compressed bgr8
        np_img = np.fromstring(data.data, dtype=np.uint8)
        img = cv2.imdecode(np_img, cv2.CV_LOAD_IMAGE_COLOR)
        img = cv2.cvtColor(cv2.resize(img, (self.width, self.height)), cv2.COLOR_BGR2GRAY)
        # cv2.imwrite("/home/yj/ss.jpg", self.image)
        self.image = np.reshape(img, newshape=(self.width, self.height, 1)) / 256.0

    def get_image(self):
        return self.image


if __name__ == "__main__":
    try:
        rospy.init_node('gazebo_listern', anonymous=True)
        cube_obj = CubesManager()
        rospy.sleep(2)
        print "===== cube pose:", cube_obj.get_cube_pose()
        cube_obj.set_cube_pose("cubes::cube1", [0, -0.3, 1])
        print "===== cube pose:", cube_obj.get_cube_pose()

        # listener = CameraListener(640, 480)
        while not rospy.is_shutdown():
            rospy.sleep(2)
    except rospy.ROSInterruptException:
        pass
